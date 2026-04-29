#include "winusb_transport_win.h"

#include <algorithm>
#include <cstdio>
#include <cwctype>
#include <cstring>
#include <string>
#include <vector>

#include <windows.h>
#include <setupapi.h>
#include <usb.h>
#include <winusb.h>

namespace {

// Must match the DeviceInterfaceGUIDs value emitted by the firmware MS OS 2.0 descriptor.
constexpr GUID kPicoJ2534InterfaceGuid = {
    0xa9f78e2a, 0x39a0, 0x4a36, {0xa6, 0xdf, 0x6d, 0x80, 0xc9, 0x6f, 0x54, 0xe1}
};

// Windows' default WinUSB device interface class. Used if DeviceInterfaceGUIDs
// from the MS OS descriptor did not get registered by the host.
constexpr GUID kDefaultWinUsbInterfaceGuid = {
    0xdee824ef, 0x729b, 0x4a0e, {0x9c, 0x14, 0xb7, 0x11, 0x7d, 0x33, 0xa8, 0x17}
};

constexpr uint8_t kInvalidPipe = 0;

std::string windowsError(const char* prefix, DWORD error) {
    char buffer[160]{};
    std::snprintf(buffer, sizeof(buffer), "%s (GetLastError=%lu)", prefix, static_cast<unsigned long>(error));
    return buffer;
}

std::wstring lowerPath(const wchar_t* path) {
    std::wstring lowered = path ? path : L"";
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](wchar_t ch) {
        return static_cast<wchar_t>(std::towlower(ch));
    });
    return lowered;
}

bool isPicoJ2534InterfacePath(const wchar_t* path) {
    const std::wstring lowered = lowerPath(path);
    return lowered.find(L"vid_1209&pid_2534") != std::wstring::npos &&
           lowered.find(L"mi_02") != std::wstring::npos;
}

}

WinUsbTransport::WinUsbTransport()
    : deviceHandle_(INVALID_HANDLE_VALUE), winusbHandle_(nullptr), bulkIn_(kInvalidPipe), bulkOut_(kInvalidPipe), seq_(1) {}

WinUsbTransport::~WinUsbTransport() {
    close();
}

bool WinUsbTransport::open() {
    std::lock_guard<std::mutex> lock(ioMutex_);
    close();

    auto openByGuid = [this](const GUID& guid, bool requirePicoPath) {
        HDEVINFO devInfo = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (devInfo == INVALID_HANDLE_VALUE) {
            return false;
        }

        SP_DEVICE_INTERFACE_DATA ifData{};
        ifData.cbSize = sizeof(ifData);

        for (DWORD index = 0; SetupDiEnumDeviceInterfaces(devInfo, nullptr, &guid, index, &ifData); ++index) {
            DWORD needed = 0;
            SetupDiGetDeviceInterfaceDetailW(devInfo, &ifData, nullptr, 0, &needed, nullptr);
            if (needed == 0) {
                continue;
            }

            std::vector<uint8_t> detailBuf(needed);
            auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(detailBuf.data());
            detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
            if (!SetupDiGetDeviceInterfaceDetailW(devInfo, &ifData, detail, needed, nullptr, nullptr)) {
                continue;
            }
            if (requirePicoPath && !isPicoJ2534InterfacePath(detail->DevicePath)) {
                continue;
            }

            HANDLE device = CreateFileW(detail->DevicePath,
                                        GENERIC_READ | GENERIC_WRITE,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        nullptr,
                                        OPEN_EXISTING,
                                        FILE_ATTRIBUTE_NORMAL,
                                        nullptr);
            if (device == INVALID_HANDLE_VALUE) {
                continue;
            }

            WINUSB_INTERFACE_HANDLE usb = nullptr;
            if (!WinUsb_Initialize(device, &usb)) {
                CloseHandle(device);
                continue;
            }

            USB_INTERFACE_DESCRIPTOR iface{};
            if (!WinUsb_QueryInterfaceSettings(usb, 0, &iface)) {
                WinUsb_Free(usb);
                CloseHandle(device);
                continue;
            }

            uint8_t bulkIn = kInvalidPipe;
            uint8_t bulkOut = kInvalidPipe;
            for (uint8_t i = 0; i < iface.bNumEndpoints; ++i) {
                WINUSB_PIPE_INFORMATION pipe{};
                if (!WinUsb_QueryPipe(usb, 0, i, &pipe)) {
                    continue;
                }
                if (pipe.PipeType == UsbdPipeTypeBulk && USB_ENDPOINT_DIRECTION_IN(pipe.PipeId)) {
                    bulkIn = pipe.PipeId;
                } else if (pipe.PipeType == UsbdPipeTypeBulk && USB_ENDPOINT_DIRECTION_OUT(pipe.PipeId)) {
                    bulkOut = pipe.PipeId;
                }
            }

            if (bulkIn != kInvalidPipe && bulkOut != kInvalidPipe) {
                deviceHandle_ = device;
                winusbHandle_ = usb;
                bulkIn_ = bulkIn;
                bulkOut_ = bulkOut;
                SetupDiDestroyDeviceInfoList(devInfo);
                return true;
            }

            WinUsb_Free(usb);
            CloseHandle(device);
        }

        SetupDiDestroyDeviceInfoList(devInfo);
        return false;
    };

    if (openByGuid(kPicoJ2534InterfaceGuid, false) ||
        openByGuid(kDefaultWinUsbInterfaceGuid, true)) {
        return true;
    }

    setError("Pico J2534 WinUSB interface not found");
    return false;
}

void WinUsbTransport::close() {
    if (winusbHandle_) {
        WinUsb_Free(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_));
        winusbHandle_ = nullptr;
    }
    if (deviceHandle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(static_cast<HANDLE>(deviceHandle_));
        deviceHandle_ = INVALID_HANDLE_VALUE;
    }
    bulkIn_ = kInvalidPipe;
    bulkOut_ = kInvalidPipe;
}

bool WinUsbTransport::isOpen() const {
    return winusbHandle_ != nullptr;
}

bool WinUsbTransport::transact(uint8_t cmd, const void* outData, uint8_t outLen, picoj_packet_t& response, unsigned timeoutMs) {
    if (outLen > PICOJ_PACKET_PAYLOAD_SIZE) {
        setError("USB payload too large");
        return false;
    }

    std::lock_guard<std::mutex> lock(ioMutex_);
    if (!isOpen()) {
        setError("WinUSB device is closed");
        return false;
    }

    picoj_packet_t request{};
    request.magic = PICOJ_PACKET_MAGIC;
    request.seq = seq_++;
    request.cmd = cmd;
    request.len = outLen;
    if (outData && outLen) {
        std::memcpy(request.payload, outData, outLen);
    }

    if (!writePacketUnlocked(request, timeoutMs)) {
        return false;
    }

    const DWORD start = GetTickCount();
    for (;;) {
        const DWORD elapsed = GetTickCount() - start;
        if (elapsed >= timeoutMs) {
            setError("WinUSB transaction timed out");
            return false;
        }

        if (!readPacketUnlocked(response, timeoutMs - elapsed)) {
            return false;
        }

        if (response.magic == PICOJ_PACKET_MAGIC && response.seq == request.seq) {
            return true;
        }
    }
}

bool WinUsbTransport::readPacket(picoj_packet_t& packet, unsigned timeoutMs) {
    std::lock_guard<std::mutex> lock(ioMutex_);
    return readPacketUnlocked(packet, timeoutMs);
}

bool WinUsbTransport::writePacketUnlocked(const picoj_packet_t& packet, unsigned timeoutMs) {
    ULONG timeout = timeoutMs;
    WinUsb_SetPipePolicy(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_), bulkOut_, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);

    ULONG transferred = 0;
    if (!WinUsb_WritePipe(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                          bulkOut_,
                          const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&packet)),
                          sizeof(packet),
                          &transferred,
                          nullptr) ||
        transferred != sizeof(packet)) {
        setError(windowsError("WinUSB bulk write failed", GetLastError()));
        return false;
    }
    return true;
}

bool WinUsbTransport::readPacketUnlocked(picoj_packet_t& packet, unsigned timeoutMs) {
    ULONG timeout = timeoutMs;
    WinUsb_SetPipePolicy(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_), bulkIn_, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);

    ULONG transferred = 0;
    if (!WinUsb_ReadPipe(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                         bulkIn_,
                         reinterpret_cast<uint8_t*>(&packet),
                         sizeof(packet),
                         &transferred,
                         nullptr) ||
        transferred != sizeof(packet)) {
        setError(windowsError("WinUSB bulk read failed", GetLastError()));
        return false;
    }

    if (packet.magic != PICOJ_PACKET_MAGIC || packet.len > PICOJ_PACKET_PAYLOAD_SIZE) {
        setError("Invalid USB packet");
        return false;
    }
    return true;
}

void WinUsbTransport::setError(const std::string& error) {
    lastError_ = error;
}
