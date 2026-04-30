#include "winusb_transport_win.h"

#include <cstdio>
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

constexpr uint8_t kInvalidPipe = 0;
constexpr size_t kMaxPendingPackets = 256;

std::string windowsError(const char* prefix, DWORD error) {
    char buffer[160]{};
    std::snprintf(buffer, sizeof(buffer), "%s (GetLastError=%lu)", prefix, static_cast<unsigned long>(error));
    return buffer;
}

bool isTimeoutError(DWORD error) {
    return error == ERROR_SEM_TIMEOUT || error == ERROR_TIMEOUT;
}

bool isDisconnectError(DWORD error) {
    return error == ERROR_DEVICE_NOT_CONNECTED || error == ERROR_DEV_NOT_EXIST || error == ERROR_INVALID_HANDLE ||
           error == ERROR_GEN_FAILURE || error == ERROR_NOT_READY;
}

}

WinUsbTransport::WinUsbTransport()
    : deviceHandle_(INVALID_HANDLE_VALUE),
      winusbHandle_(nullptr),
      bulkIn_(kInvalidPipe),
      bulkOut_(kInvalidPipe),
      seq_(1),
      lastErrorWasTimeout_(false) {}

WinUsbTransport::~WinUsbTransport() {
    close();
}

bool WinUsbTransport::open() {
    std::lock_guard<std::mutex> lock(ioMutex_);
    closeUnlocked();
    pendingPackets_.clear();

    std::string openError = "Pico J2534 WinUSB interface not found";
    auto openByGuid = [this, &openError](const GUID& guid) {
        HDEVINFO devInfo = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (devInfo == INVALID_HANDLE_VALUE) {
            openError = windowsError("SetupDiGetClassDevs failed", GetLastError());
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
            HANDLE device = CreateFileW(detail->DevicePath,
                                        GENERIC_READ | GENERIC_WRITE,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        nullptr,
                                        OPEN_EXISTING,
                                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                                        nullptr);
            if (device == INVALID_HANDLE_VALUE) {
                openError = windowsError("CreateFile failed for Pico J2534 WinUSB interface", GetLastError());
                continue;
            }

            WINUSB_INTERFACE_HANDLE usb = nullptr;
            if (!WinUsb_Initialize(device, &usb)) {
                openError = windowsError("WinUsb_Initialize failed for Pico J2534 interface", GetLastError());
                CloseHandle(device);
                continue;
            }

            USB_INTERFACE_DESCRIPTOR iface{};
            if (!WinUsb_QueryInterfaceSettings(usb, 0, &iface)) {
                openError = windowsError("WinUsb_QueryInterfaceSettings failed for Pico J2534 interface", GetLastError());
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
                seq_ = 1;
                SetupDiDestroyDeviceInfoList(devInfo);
                return true;
            }

            openError = "Pico J2534 WinUSB interface has no bulk IN/OUT pipes";
            WinUsb_Free(usb);
            CloseHandle(device);
        }

        SetupDiDestroyDeviceInfoList(devInfo);
        return false;
    };

    if (openByGuid(kPicoJ2534InterfaceGuid)) {
        return true;
    }

    setError(openError);
    return false;
}

void WinUsbTransport::close() {
    std::lock_guard<std::mutex> lock(ioMutex_);
    closeUnlocked();
}

void WinUsbTransport::closeUnlocked() {
    pendingPackets_.clear();
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

void WinUsbTransport::flushInputUnlocked() {
    pendingPackets_.clear();
    if (!isOpen() || bulkIn_ == kInvalidPipe) {
        return;
    }
    if (!WinUsb_FlushPipe(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_), bulkIn_)) {
        const DWORD error = GetLastError();
        if (isDisconnectError(error)) {
            closeUnlocked();
        }
    }
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

    flushInputUnlocked();

    picoj_packet_t request{};
    request.magic = PICOJ_PACKET_MAGIC;
    request.seq = seq_;
    seq_ = static_cast<uint8_t>(seq_ == 0xFF ? 1 : seq_ + 1);
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
            lastErrorWasTimeout_ = true;
            return false;
        }

        if (!readPacketUnlocked(response, timeoutMs - elapsed)) {
            return false;
        }

        if (response.cmd == PICOJ_CMD_CAN_RX) {
            if (pendingPackets_.size() >= kMaxPendingPackets) {
                pendingPackets_.pop_front();
            }
            pendingPackets_.push_back(response);
            continue;
        }
        if (response.magic == PICOJ_PACKET_MAGIC && response.seq == request.seq) {
            return true;
        }
    }
}

bool WinUsbTransport::readPacket(picoj_packet_t& packet, unsigned timeoutMs) {
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (!pendingPackets_.empty()) {
        packet = pendingPackets_.front();
        pendingPackets_.pop_front();
        return true;
    }
    return readPacketUnlocked(packet, timeoutMs);
}

void WinUsbTransport::clearPending() {
    std::lock_guard<std::mutex> lock(ioMutex_);
    flushInputUnlocked();
}

void WinUsbTransport::restorePending(const std::vector<picoj_packet_t>& packets) {
    std::lock_guard<std::mutex> lock(ioMutex_);
    pendingPackets_.insert(pendingPackets_.begin(), packets.begin(), packets.end());
    while (pendingPackets_.size() > kMaxPendingPackets) {
        pendingPackets_.pop_back();
    }
}

bool WinUsbTransport::writePacketUnlocked(const picoj_packet_t& packet, unsigned timeoutMs) {
    lastErrorWasTimeout_ = false;
    ULONG timeout = timeoutMs;
    if (!WinUsb_SetPipePolicy(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                              bulkOut_,
                              PIPE_TRANSFER_TIMEOUT,
                              sizeof(timeout),
                              &timeout)) {
        const DWORD error = GetLastError();
        setError(windowsError("WinUSB set bulk OUT timeout failed", error));
        if (isDisconnectError(error)) {
            closeUnlocked();
        }
        return false;
    }

    ULONG transferred = 0;
    if (!WinUsb_WritePipe(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                          bulkOut_,
                          const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&packet)),
                          sizeof(packet),
                          &transferred,
                          nullptr)) {
        const DWORD error = GetLastError();
        setError(windowsError("WinUSB bulk write failed", error));
        if (isTimeoutError(error)) {
            lastErrorWasTimeout_ = true;
        } else if (isDisconnectError(error)) {
            closeUnlocked();
        }
        return false;
    }
    if (transferred != sizeof(packet)) {
        char buffer[160]{};
        std::snprintf(buffer,
                      sizeof(buffer),
                      "WinUSB bulk write transferred %lu of %zu bytes",
                      static_cast<unsigned long>(transferred),
                      sizeof(packet));
        setError(buffer);
        lastErrorWasTimeout_ = true;
        return false;
    }
    return true;
}

bool WinUsbTransport::readPacketUnlocked(picoj_packet_t& packet, unsigned timeoutMs) {
    lastErrorWasTimeout_ = false;
    std::memset(&packet, 0, sizeof(packet));
    auto* out = reinterpret_cast<uint8_t*>(&packet);
    ULONG total = 0;
    const DWORD start = GetTickCount();

    while (total < sizeof(packet)) {
        const DWORD elapsed = GetTickCount() - start;
        if (elapsed >= timeoutMs) {
            char buffer[160]{};
            std::snprintf(buffer, sizeof(buffer), "WinUSB bulk read timed out after %lu of %zu bytes",
                          static_cast<unsigned long>(total), sizeof(packet));
            setError(buffer);
            lastErrorWasTimeout_ = true;
            return false;
        }

        ULONG timeout = timeoutMs - elapsed;
        if (!WinUsb_SetPipePolicy(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                                  bulkIn_,
                                  PIPE_TRANSFER_TIMEOUT,
                                  sizeof(timeout),
                                  &timeout)) {
            const DWORD error = GetLastError();
            setError(windowsError("WinUSB set bulk IN timeout failed", error));
            if (isDisconnectError(error)) {
                closeUnlocked();
            }
            return false;
        }

        ULONG transferred = 0;
        if (!WinUsb_ReadPipe(static_cast<WINUSB_INTERFACE_HANDLE>(winusbHandle_),
                             bulkIn_,
                             out + total,
                             static_cast<ULONG>(sizeof(packet) - total),
                             &transferred,
                             nullptr)) {
            const DWORD error = GetLastError();
            setError(windowsError("WinUSB bulk read failed", error));
            if (isTimeoutError(error)) {
                lastErrorWasTimeout_ = true;
            } else if (isDisconnectError(error)) {
                closeUnlocked();
            }
            return false;
        }

        if (transferred == 0) {
            Sleep(1);
            continue;
        }
        total += transferred;
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
