#include "j2534.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "isotp.h"
#include "winusb_transport_win.h"

namespace {

constexpr unsigned long kDeviceId = 1;
constexpr unsigned long kChannelId = 1;
constexpr unsigned kDefaultTimeoutMs = 1000;

struct ChannelState {
    bool connected = false;
    unsigned long protocol = CAN;
    unsigned long flags = 0;
    unsigned long baudrate = 500000;
    bool flowControlEnabled = false;
    uint32_t flowControlCanId = 0;
    IsoTpReassembler isoRx;
};

std::mutex g_lock;
WinUsbTransport g_usb;
ChannelState g_channel;
std::string g_lastError = "No error";

void setLastError(const char* text) {
    g_lastError = text ? text : "Unknown error";
}

void setLastError(const std::string& text) {
    g_lastError = text;
}

uint32_t readCanId(const PASSTHRU_MSG& msg) {
    if (msg.DataSize < 4) {
        return 0;
    }
    return (static_cast<uint32_t>(msg.Data[0]) << 24) |
           (static_cast<uint32_t>(msg.Data[1]) << 16) |
           (static_cast<uint32_t>(msg.Data[2]) << 8) |
           static_cast<uint32_t>(msg.Data[3]);
}

void writeCanId(PASSTHRU_MSG& msg, uint32_t canId) {
    msg.Data[0] = static_cast<unsigned char>((canId >> 24) & 0xFF);
    msg.Data[1] = static_cast<unsigned char>((canId >> 16) & 0xFF);
    msg.Data[2] = static_cast<unsigned char>((canId >> 8) & 0xFF);
    msg.Data[3] = static_cast<unsigned char>(canId & 0xFF);
}

long ensureDevice(unsigned long deviceId) {
    if (deviceId != kDeviceId) {
        setLastError("Invalid device id");
        return ERR_INVALID_DEVICE_ID;
    }
    if (!g_usb.isOpen()) {
        setLastError("Pico J2534 device is not open");
        return ERR_DEVICE_NOT_CONNECTED;
    }
    return STATUS_NOERROR;
}

long ensureChannel(unsigned long channelId) {
    if (channelId != kChannelId || !g_channel.connected) {
        setLastError("Invalid or disconnected channel id");
        return ERR_INVALID_CHANNEL_ID;
    }
    return STATUS_NOERROR;
}

long sendCan(const picoj_can_frame_t& frame, unsigned timeoutMs) {
    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_CAN_TX, &frame, sizeof(frame), response, timeoutMs ? timeoutMs : kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        return ERR_FAILED;
    }
    if (response.cmd == PICOJ_CMD_STATUS && response.len >= sizeof(picoj_status_t)) {
        picoj_status_t status{};
        std::memcpy(&status, response.payload, sizeof(status));
        if (status.code != 0) {
            setLastError("Firmware rejected CAN transmit");
            return ERR_FAILED;
        }
    }
    return STATUS_NOERROR;
}

long sendIsoTpFlowControl(const picoj_can_frame_t& responseFrame) {
    if (!g_channel.flowControlEnabled) {
        return STATUS_NOERROR;
    }

    picoj_can_frame_t frame{};
    frame.can_id = g_channel.flowControlCanId;
    frame.flags = responseFrame.flags & PICOJ_CAN_EXTENDED;
    frame.dlc = 8;
    frame.data[0] = 0x30; // Continue To Send, block size 0, STmin 0.

    return sendCan(frame, kDefaultTimeoutMs);
}

bool decodeCanPacket(const picoj_packet_t& packet, picoj_can_frame_t& frame) {
    if (packet.cmd != PICOJ_CMD_CAN_RX || packet.len < sizeof(picoj_can_frame_t)) {
        return false;
    }
    std::memcpy(&frame, packet.payload, sizeof(frame));
    return true;
}

void fillCanMessage(PASSTHRU_MSG& msg, unsigned long protocol, const picoj_can_frame_t& frame) {
    std::memset(&msg, 0, sizeof(msg));
    msg.ProtocolID = protocol;
    msg.Timestamp = GetTickCount();
    msg.DataSize = 4 + frame.dlc;
    msg.ExtraDataIndex = msg.DataSize;
    writeCanId(msg, frame.can_id);
    std::memcpy(msg.Data + 4, frame.data, frame.dlc);
    if (frame.flags & PICOJ_CAN_EXTENDED) {
        msg.RxStatus |= CAN_29BIT_ID;
    }
}

} // namespace

extern "C" long WINAPI PassThruOpen(void*, unsigned long* pDeviceID) {
    if (!pDeviceID) {
        setLastError("Null device id pointer");
        return ERR_NULL_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    if (g_usb.isOpen()) {
        *pDeviceID = kDeviceId;
        return STATUS_NOERROR;
    }

    if (!g_usb.open()) {
        setLastError(g_usb.lastError());
        return ERR_DEVICE_NOT_CONNECTED;
    }

    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_HELLO, nullptr, 0, response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        g_usb.close();
        return ERR_DEVICE_NOT_CONNECTED;
    }

    *pDeviceID = kDeviceId;
    setLastError("No error");
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruClose(unsigned long DeviceID) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureDevice(DeviceID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    g_channel = ChannelState{};
    g_usb.close();
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags, unsigned long Baudrate, unsigned long* pChannelID) {
    if (!pChannelID) {
        setLastError("Null channel id pointer");
        return ERR_NULL_PARAMETER;
    }
    if (ProtocolID != CAN && ProtocolID != ISO15765) {
        setLastError("Only CAN and ISO15765 are supported");
        return ERR_INVALID_PROTOCOL_ID;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureDevice(DeviceID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (g_channel.connected) {
        setLastError("Channel already connected");
        return ERR_CHANNEL_IN_USE;
    }

    picoj_bitrate_t bitrate{Baudrate};
    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_SET_BITRATE, &bitrate, sizeof(bitrate), response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        return ERR_FAILED;
    }

    g_channel.connected = true;
    g_channel.protocol = ProtocolID;
    g_channel.flags = Flags;
    g_channel.baudrate = Baudrate;
    *pChannelID = kChannelId;
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruDisconnect(unsigned long ChannelID) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    g_channel = ChannelState{};
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout) {
    if (!pMsg || !pNumMsgs) {
        setLastError("Null read message argument");
        return ERR_NULL_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }

    const unsigned long requested = *pNumMsgs;
    *pNumMsgs = 0;
    const DWORD start = GetTickCount();

    while (*pNumMsgs < requested) {
        DWORD elapsed = GetTickCount() - start;
        if (elapsed >= Timeout && *pNumMsgs == 0) {
            setLastError("Read timed out");
            return ERR_BUFFER_EMPTY;
        }
        if (elapsed >= Timeout) {
            return STATUS_NOERROR;
        }

        picoj_packet_t packet{};
        if (!g_usb.readPacket(packet, Timeout - elapsed)) {
            setLastError(g_usb.lastError());
            return *pNumMsgs ? STATUS_NOERROR : ERR_BUFFER_EMPTY;
        }

        picoj_can_frame_t frame{};
        if (!decodeCanPacket(packet, frame)) {
            continue;
        }

        if (g_channel.protocol == ISO15765) {
            if (frame.dlc >= 2 && (frame.data[0] >> 4) == 0x1) {
                status = sendIsoTpFlowControl(frame);
                if (status != STATUS_NOERROR) {
                    return status;
                }
            }

            IsoTpFrame iso{};
            if (!g_channel.isoRx.accept(frame, iso)) {
                continue;
            }
            PASSTHRU_MSG& msg = pMsg[*pNumMsgs];
            std::memset(&msg, 0, sizeof(msg));
            msg.ProtocolID = ISO15765;
            msg.Timestamp = GetTickCount();
            msg.DataSize = static_cast<unsigned long>(4 + iso.payload.size());
            msg.ExtraDataIndex = msg.DataSize;
            writeCanId(msg, iso.canId);
            std::memcpy(msg.Data + 4, iso.payload.data(), iso.payload.size());
            if (iso.extended) {
                msg.RxStatus |= CAN_29BIT_ID;
            }
            ++(*pNumMsgs);
        } else {
            fillCanMessage(pMsg[*pNumMsgs], CAN, frame);
            ++(*pNumMsgs);
        }
    }

    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout) {
    if (!pMsg || !pNumMsgs) {
        setLastError("Null write message argument");
        return ERR_NULL_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }

    const unsigned long requested = *pNumMsgs;
    *pNumMsgs = 0;

    for (unsigned long i = 0; i < requested; ++i) {
        const PASSTHRU_MSG& msg = pMsg[i];
        if (msg.DataSize < 4 || msg.DataSize > sizeof(msg.Data)) {
            setLastError("Invalid J2534 message size");
            return ERR_INVALID_MSG;
        }

        const uint32_t canId = readCanId(msg);
        const bool extended = (msg.TxFlags & CAN_29BIT_ID) || (g_channel.flags & CAN_29BIT_ID);

        if (g_channel.protocol == ISO15765) {
            auto frames = isotpSegment(canId, extended, msg.Data + 4, msg.DataSize - 4);
            for (const auto& frame : frames) {
                status = sendCan(frame, Timeout);
                if (status != STATUS_NOERROR) {
                    return status;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        } else {
            if (msg.DataSize - 4 > 8) {
                setLastError("Classic CAN payload exceeds 8 bytes");
                return ERR_INVALID_MSG;
            }
            picoj_can_frame_t frame{};
            frame.can_id = canId;
            frame.flags = extended ? PICOJ_CAN_EXTENDED : 0;
            frame.dlc = static_cast<uint8_t>(msg.DataSize - 4);
            std::memcpy(frame.data, msg.Data + 4, frame.dlc);
            status = sendCan(frame, Timeout);
            if (status != STATUS_NOERROR) {
                return status;
            }
        }

        ++(*pNumMsgs);
    }

    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruStartPeriodicMsg(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long) {
    setLastError("Periodic messages are not implemented");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruStopPeriodicMsg(unsigned long, unsigned long) {
    setLastError("Periodic messages are not implemented");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG*, PASSTHRU_MSG*, PASSTHRU_MSG* pFlowControlMsg, unsigned long* pFilterID) {
    if (!pFilterID) {
        setLastError("Null filter id pointer");
        return ERR_NULL_PARAMETER;
    }
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (FilterType != PASS_FILTER && FilterType != FLOW_CONTROL_FILTER) {
        setLastError("Only pass and flow-control filters are accepted");
        return ERR_NOT_SUPPORTED;
    }
    if (FilterType == FLOW_CONTROL_FILTER) {
        if (!pFlowControlMsg) {
            setLastError("Null flow-control message");
            return ERR_NULL_PARAMETER;
        }
        if (pFlowControlMsg->DataSize < 4) {
            setLastError("Invalid flow-control message");
            return ERR_INVALID_MSG;
        }
        g_channel.flowControlEnabled = true;
        g_channel.flowControlCanId = readCanId(*pFlowControlMsg);
    }
    *pFilterID = 1;
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruStopMsgFilter(unsigned long ChannelID, unsigned long) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status == STATUS_NOERROR) {
        g_channel.flowControlEnabled = false;
        g_channel.flowControlCanId = 0;
    }
    return status;
}

extern "C" long WINAPI PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long, unsigned long) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureDevice(DeviceID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    setLastError("Programming voltage is not supported");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruReadVersion(unsigned long DeviceID, char* pFirmwareVersion, char* pDllVersion, char* pApiVersion) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureDevice(DeviceID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (pFirmwareVersion) {
        std::strcpy(pFirmwareVersion, "pico_j2534 0.1");
    }
    if (pDllVersion) {
        std::strcpy(pDllVersion, "pico_j2534_dll 0.1");
    }
    if (pApiVersion) {
        std::strcpy(pApiVersion, "04.04");
    }
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruGetLastError(char* pErrorDescription) {
    if (!pErrorDescription) {
        return ERR_NULL_PARAMETER;
    }
    std::lock_guard<std::mutex> lock(g_lock);
    std::strncpy(pErrorDescription, g_lastError.c_str(), 79);
    pErrorDescription[79] = '\0';
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void*, void*) {
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (IoctlID == CLEAR_RX_BUFFER || IoctlID == CLEAR_TX_BUFFER || IoctlID == CLEAR_MSG_FILTERS || IoctlID == CLEAR_PERIODIC_MSGS) {
        g_channel.isoRx.reset();
        return STATUS_NOERROR;
    }
    setLastError("Ioctl is not implemented");
    return ERR_INVALID_IOCTL_ID;
}
