#include "j2534.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdarg>
#include <cstdio>
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

void logEvent(const char* function, const char* format, ...) {
    char details[512]{};
    va_list args;
    va_start(args, format);
    std::vsnprintf(details, sizeof(details), format, args);
    va_end(args);

    SYSTEMTIME now{};
    GetLocalTime(&now);

    char tempPath[MAX_PATH]{};
    DWORD tempLen = GetTempPathA(static_cast<DWORD>(sizeof(tempPath)), tempPath);
    std::string path = (tempLen > 0 && tempLen < sizeof(tempPath)) ? tempPath : ".\\";
    path += "pico_j2534.log";

    FILE* file = nullptr;
    if (fopen_s(&file, path.c_str(), "a") == 0 && file) {
        std::fprintf(file,
                     "%04u-%02u-%02u %02u:%02u:%02u.%03u %s: %s\n",
                     now.wYear,
                     now.wMonth,
                     now.wDay,
                     now.wHour,
                     now.wMinute,
                     now.wSecond,
                     now.wMilliseconds,
                     function,
                     details);
        std::fclose(file);
    }

    char debugLine[640]{};
    std::snprintf(debugLine, sizeof(debugLine), "pico_j2534 %s: %s\n", function, details);
    OutputDebugStringA(debugLine);
}

void logUnsupported(const char* function, const char* format, ...) {
    char details[512]{};
    va_list args;
    va_start(args, format);
    std::vsnprintf(details, sizeof(details), format, args);
    va_end(args);
    logEvent(function, "unsupported: %s", details);
}

void logReadMsgsCall(unsigned long channelId, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long timeout) {
    if (timeout != 0) {
        logEvent("PassThruReadMsgs",
                 "ChannelID=%lu Msg=%p NumMsgs=%p Requested=%lu Timeout=%lu",
                 channelId,
                 static_cast<void*>(pMsg),
                 static_cast<void*>(pNumMsgs),
                 pNumMsgs ? *pNumMsgs : 0,
                 timeout);
        return;
    }

    static std::mutex logLock;
    static DWORD lastLogMs = 0;
    static unsigned long suppressedCalls = 0;

    std::lock_guard<std::mutex> lock(logLock);
    const DWORD now = GetTickCount();
    if (lastLogMs == 0 || now - lastLogMs >= 1000) {
        logEvent("PassThruReadMsgs",
                 "ChannelID=%lu Msg=%p NumMsgs=%p Requested=%lu Timeout=0 SuppressedZeroTimeoutReads=%lu",
                 channelId,
                 static_cast<void*>(pMsg),
                 static_cast<void*>(pNumMsgs),
                 pNumMsgs ? *pNumMsgs : 0,
                 suppressedCalls);
        suppressedCalls = 0;
        lastLogMs = now;
    } else {
        ++suppressedCalls;
    }
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
    if (!g_usb.isOpen()) {
        g_channel = ChannelState{};
        setLastError("Pico J2534 device is not connected");
        return ERR_DEVICE_NOT_CONNECTED;
    }
    return STATUS_NOERROR;
}

long sendCan(const picoj_can_frame_t& frame, unsigned timeoutMs) {
    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_CAN_TX, &frame, sizeof(frame), response, timeoutMs ? timeoutMs : kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        if (!g_usb.isOpen()) {
            g_channel = ChannelState{};
            return ERR_DEVICE_NOT_CONNECTED;
        }
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

long setBitrate(unsigned long bitrate) {
    picoj_bitrate_t request{bitrate};
    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_SET_BITRATE, &request, sizeof(request), response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        if (!g_usb.isOpen()) {
            g_channel = ChannelState{};
            return ERR_DEVICE_NOT_CONNECTED;
        }
        return ERR_FAILED;
    }
    if (response.cmd == PICOJ_CMD_STATUS && response.len >= sizeof(picoj_status_t)) {
        picoj_status_t status{};
        std::memcpy(&status, response.payload, sizeof(status));
        if (status.code != 0) {
            setLastError("Firmware rejected CAN bitrate");
            return ERR_FAILED;
        }
    }
    g_channel.baudrate = bitrate;
    return STATUS_NOERROR;
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

long openUsbDevice() {
    if (!g_usb.open()) {
        setLastError(g_usb.lastError());
        logEvent("OpenUsbDevice", "failed to open USB: %s", g_lastError.c_str());
        return ERR_DEVICE_NOT_CONNECTED;
    }

    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_HELLO, nullptr, 0, response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        logEvent("OpenUsbDevice", "HELLO failed: %s", g_lastError.c_str());
        g_usb.close();
        return ERR_DEVICE_NOT_CONNECTED;
    }

    return STATUS_NOERROR;
}

} // namespace

extern "C" long WINAPI PassThruOpen(void*, unsigned long* pDeviceID) {
    logEvent("PassThruOpen", "DeviceIDOut=%p", static_cast<void*>(pDeviceID));
    if (!pDeviceID) {
        setLastError("Null device id pointer");
        return ERR_NULL_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    if (g_usb.isOpen()) {
        picoj_packet_t response{};
        if (!g_channel.connected && !g_usb.transact(PICOJ_CMD_HELLO, nullptr, 0, response, kDefaultTimeoutMs)) {
            setLastError(g_usb.lastError());
            logEvent("PassThruOpen", "existing USB handle failed HELLO: %s", g_lastError.c_str());
            g_usb.close();
            g_channel = ChannelState{};
        } else {
            *pDeviceID = kDeviceId;
            return STATUS_NOERROR;
        }
    }

    if (g_usb.isOpen()) {
        *pDeviceID = kDeviceId;
        return STATUS_NOERROR;
    }

    long status = openUsbDevice();
    if (status != STATUS_NOERROR) {
        return status;
    }

    *pDeviceID = kDeviceId;
    setLastError("No error");
    logEvent("PassThruOpen", "ok DeviceID=%lu", *pDeviceID);
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruClose(unsigned long DeviceID) {
    logEvent("PassThruClose", "DeviceID=%lu", DeviceID);
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
    logEvent("PassThruConnect",
             "DeviceID=%lu ProtocolID=0x%08lX Flags=0x%08lX Baudrate=%lu ChannelIDOut=%p",
             DeviceID,
             ProtocolID,
             Flags,
             Baudrate,
             static_cast<void*>(pChannelID));
    if (!pChannelID) {
        setLastError("Null channel id pointer");
        return ERR_NULL_PARAMETER;
    }
    if (ProtocolID != CAN && ProtocolID != ISO15765) {
        logUnsupported("PassThruConnect",
                       "unsupported ProtocolID=0x%08lX Flags=0x%08lX Baudrate=%lu",
                       ProtocolID,
                       Flags,
                       Baudrate);
        setLastError("Only CAN and ISO15765 are supported");
        return ERR_INVALID_PROTOCOL_ID;
    }

    std::lock_guard<std::mutex> lock(g_lock);
    if (DeviceID != kDeviceId) {
        setLastError("Invalid device id");
        return ERR_INVALID_DEVICE_ID;
    }
    if (!g_usb.isOpen()) {
        long openStatus = openUsbDevice();
        if (openStatus != STATUS_NOERROR) {
            return openStatus;
        }
    }
    if (g_channel.connected) {
        setLastError("Channel already connected");
        return ERR_CHANNEL_IN_USE;
    }

    g_channel.connected = true;
    g_channel.protocol = ProtocolID;
    g_channel.flags = Flags;
    long bitrateStatus = setBitrate(Baudrate);
    if (bitrateStatus != STATUS_NOERROR) {
        g_channel = ChannelState{};
        return bitrateStatus;
    }

    *pChannelID = kChannelId;
    logEvent("PassThruConnect", "ok ChannelID=%lu", *pChannelID);
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruDisconnect(unsigned long ChannelID) {
    logEvent("PassThruDisconnect", "ChannelID=%lu", ChannelID);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    g_channel = ChannelState{};
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout) {
    logReadMsgsCall(ChannelID, pMsg, pNumMsgs, Timeout);
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
    const bool nonBlocking = Timeout == 0;

    while (*pNumMsgs < requested) {
        DWORD elapsed = GetTickCount() - start;
        if (!nonBlocking) {
            if (elapsed >= Timeout && *pNumMsgs == 0) {
                setLastError("Read timed out");
                return ERR_BUFFER_EMPTY;
            }
            if (elapsed >= Timeout) {
                return STATUS_NOERROR;
            }
        }

        picoj_packet_t packet{};
        const unsigned readTimeout = nonBlocking ? 1u : Timeout - elapsed;
        if (!g_usb.readPacket(packet, readTimeout)) {
            setLastError(g_usb.lastError());
            if (!g_usb.isOpen()) {
                g_channel = ChannelState{};
                return *pNumMsgs ? STATUS_NOERROR : ERR_DEVICE_NOT_CONNECTED;
            }
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
    logEvent("PassThruWriteMsgs",
             "ChannelID=%lu Msg=%p NumMsgs=%p Requested=%lu Timeout=%lu",
             ChannelID,
             static_cast<void*>(pMsg),
             static_cast<void*>(pNumMsgs),
             pNumMsgs ? *pNumMsgs : 0,
             Timeout);
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

extern "C" long WINAPI PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pMsgID, unsigned long TimeInterval) {
    logUnsupported("PassThruStartPeriodicMsg",
                   "ChannelID=%lu Msg=%p MsgID=%p TimeInterval=%lu",
                   ChannelID,
                   static_cast<void*>(pMsg),
                   static_cast<void*>(pMsgID),
                   TimeInterval);
    setLastError("Periodic messages are not implemented");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID) {
    logUnsupported("PassThruStopPeriodicMsg", "ChannelID=%lu MsgID=%lu", ChannelID, MsgID);
    setLastError("Periodic messages are not implemented");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG*, PASSTHRU_MSG*, PASSTHRU_MSG* pFlowControlMsg, unsigned long* pFilterID) {
    logEvent("PassThruStartMsgFilter",
             "ChannelID=%lu FilterType=%lu FlowControlMsg=%p FilterIDOut=%p",
             ChannelID,
             FilterType,
             static_cast<void*>(pFlowControlMsg),
             static_cast<void*>(pFilterID));
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
        logUnsupported("PassThruStartMsgFilter", "ChannelID=%lu unsupported FilterType=%lu", ChannelID, FilterType);
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
    logEvent("PassThruStopMsgFilter", "ChannelID=%lu", ChannelID);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status == STATUS_NOERROR) {
        g_channel.flowControlEnabled = false;
        g_channel.flowControlCanId = 0;
    }
    return status;
}

extern "C" long WINAPI PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
    logEvent("PassThruSetProgrammingVoltage",
             "DeviceID=%lu PinNumber=%lu Voltage=%lu",
             DeviceID,
             PinNumber,
             Voltage);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureDevice(DeviceID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    logUnsupported("PassThruSetProgrammingVoltage",
                   "DeviceID=%lu PinNumber=%lu Voltage=%lu",
                   DeviceID,
                   PinNumber,
                   Voltage);
    setLastError("Programming voltage is not supported");
    return ERR_NOT_SUPPORTED;
}

extern "C" long WINAPI PassThruReadVersion(unsigned long DeviceID, char* pFirmwareVersion, char* pDllVersion, char* pApiVersion) {
    logEvent("PassThruReadVersion",
             "DeviceID=%lu FirmwareOut=%p DllOut=%p ApiOut=%p",
             DeviceID,
             static_cast<void*>(pFirmwareVersion),
             static_cast<void*>(pDllVersion),
             static_cast<void*>(pApiVersion));
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
    logEvent("PassThruGetLastError", "ErrorOut=%p LastError=%s", static_cast<void*>(pErrorDescription), g_lastError.c_str());
    if (!pErrorDescription) {
        return ERR_NULL_PARAMETER;
    }
    std::lock_guard<std::mutex> lock(g_lock);
    std::strncpy(pErrorDescription, g_lastError.c_str(), 79);
    pErrorDescription[79] = '\0';
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void* pInput, void* pOutput) {
    logEvent("PassThruIoctl",
             "ChannelID=%lu IoctlID=0x%08lX Input=%p Output=%p",
             ChannelID,
             IoctlID,
             pInput,
             pOutput);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (IoctlID == GET_CONFIG) {
        auto* list = static_cast<SCONFIG_LIST*>(pInput);
        if (!list || !list->ConfigPtr) {
            setLastError("Null GET_CONFIG input");
            return ERR_NULL_PARAMETER;
        }
        for (unsigned long i = 0; i < list->NumOfParams; ++i) {
            SCONFIG& config = list->ConfigPtr[i];
            switch (config.Parameter) {
            case DATA_RATE:
                config.Value = g_channel.baudrate;
                break;
            case LOOPBACK:
                config.Value = 0;
                break;
            case ISO15765_BS:
            case ISO15765_STMIN:
            case ISO15765_WFT_MAX:
                config.Value = 0;
                break;
            case J1962_PINS:
                config.Value = 0x060E; // CAN-H pin 6, CAN-L pin 14.
                break;
            default:
                logUnsupported("PassThruIoctl(GET_CONFIG)",
                               "ChannelID=%lu unsupported Parameter=0x%08lX",
                               ChannelID,
                               config.Parameter);
                config.Value = 0;
                break;
            }
        }
        return STATUS_NOERROR;
    }
    if (IoctlID == SET_CONFIG) {
        auto* list = static_cast<SCONFIG_LIST*>(pInput);
        if (!list || !list->ConfigPtr) {
            setLastError("Null SET_CONFIG input");
            return ERR_NULL_PARAMETER;
        }
        for (unsigned long i = 0; i < list->NumOfParams; ++i) {
            const SCONFIG& config = list->ConfigPtr[i];
            switch (config.Parameter) {
            case DATA_RATE: {
                status = setBitrate(config.Value);
                if (status != STATUS_NOERROR) {
                    return status;
                }
                break;
            }
            case LOOPBACK:
            case ISO15765_BS:
            case ISO15765_STMIN:
            case ISO15765_WFT_MAX:
            case J1962_PINS:
                break;
            default:
                logUnsupported("PassThruIoctl(SET_CONFIG)",
                               "ChannelID=%lu ignored Parameter=0x%08lX Value=0x%08lX",
                               ChannelID,
                               config.Parameter,
                               config.Value);
                break;
            }
        }
        return STATUS_NOERROR;
    }
    if (IoctlID == READ_VBATT || IoctlID == READ_PROG_VOLTAGE) {
        if (!pOutput) {
            setLastError("Null voltage output");
            return ERR_NULL_PARAMETER;
        }
        *static_cast<unsigned long*>(pOutput) = 12000;
        return STATUS_NOERROR;
    }
    if (IoctlID == CLEAR_RX_BUFFER || IoctlID == CLEAR_TX_BUFFER || IoctlID == CLEAR_MSG_FILTERS || IoctlID == CLEAR_PERIODIC_MSGS) {
        g_channel.isoRx.reset();
        return STATUS_NOERROR;
    }
    logUnsupported("PassThruIoctl",
                   "ChannelID=%lu unsupported IoctlID=0x%08lX Input=%p Output=%p",
                   ChannelID,
                   IoctlID,
                   pInput,
                   pOutput);
    setLastError("Ioctl is not implemented");
    return ERR_INVALID_IOCTL_ID;
}
