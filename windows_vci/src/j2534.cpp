#include "j2534.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "winusb_transport_win.h"

namespace {

constexpr unsigned long kDeviceId = 1;
constexpr unsigned long kChannelId = 1;
constexpr unsigned kDefaultTimeoutMs = 1000;
constexpr unsigned kDefaultCanTxTimeoutMs = 1000;
constexpr unsigned kReadPollSliceMs = 20;
constexpr size_t kMaxFilters = 64;
constexpr size_t kMaxPeriodicMsgs = 16;
constexpr size_t kMaxRxQueueFrames = 4096;
constexpr size_t kMaxIsoRxQueueFrames = 256;
constexpr const char* kDllVersion = "pico_j2534_dll 0.2.1";
constexpr const char* kFrameTraceEnv = "PICO_J2534_TRACE_FRAMES";

struct IsoTpFrame {
    uint32_t canId = 0;
    bool extended = false;
    std::vector<uint8_t> payload;
};

struct QueuedCanFrame {
    picoj_can_frame_t frame{};
    unsigned long rxStatus = 0;
};

struct MessageFilter {
    bool active = false;
    unsigned long id = 0;
    unsigned long type = 0;
    uint32_t maskCanId = 0;
    uint32_t patternCanId = 0;
    bool maskExtended = false;
    bool patternExtended = false;
    bool flowControl = false;
    uint32_t flowControlCanId = 0;
    bool flowControlExtended = false;
};

struct PeriodicMessage {
    bool active = false;
    unsigned long id = 0;
    PASSTHRU_MSG msg{};
    unsigned long intervalMs = 0;
    DWORD nextDueMs = 0;
};

struct ChannelState {
    bool connected = false;
    unsigned long protocol = CAN;
    unsigned long flags = 0;
    unsigned long baudrate = 500000;
    bool loopback = false;
    unsigned long nextFilterId = 1;
    unsigned long nextPeriodicId = 1;
    std::array<MessageFilter, kMaxFilters> filters{};
    std::array<PeriodicMessage, kMaxPeriodicMsgs> periodic{};
    std::deque<QueuedCanFrame> rxQueue;
    std::deque<IsoTpFrame> isoRxQueue;
    std::vector<uint8_t> isoChunkBuffer;
    uint16_t isoChunkExpected = 0;
    uint16_t isoChunkReceived = 0;
    uint32_t isoChunkCanId = 0;
    bool isoChunkExtended = false;
    bool firmwareIsoTpConfigured = false;
    uint32_t firmwareIsoTpTxCanId = 0;
    uint32_t firmwareIsoTpRxCanId = 0;
    uint32_t firmwareIsoTpFlowControlCanId = 0;
    bool firmwareIsoTpTxExtended = false;
    bool firmwareIsoTpRxExtended = false;
};

std::mutex g_lock;
std::condition_variable g_periodicCv;
std::thread g_periodicThread;
bool g_periodicStop = false;
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
    char details[2048]{};
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

    char debugLine[2300]{};
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

bool traceFrameTraffic() {
    static int cached = -1;
    if (cached >= 0) {
        return cached != 0;
    }

    char value[16]{};
    DWORD len = GetEnvironmentVariableA(kFrameTraceEnv, value, static_cast<DWORD>(sizeof(value)));
    cached = (len > 0 && value[0] != '\0' && value[0] != '0') ? 1 : 0;
    logEvent("LOG_CONFIG", "FrameTrace=%u Env=%s", cached ? 1u : 0u, kFrameTraceEnv);
    return cached != 0;
}

bool isCanProtocol(unsigned long protocol) {
    return protocol == CAN || protocol == ISO15765;
}

uint32_t readCanId(const PASSTHRU_MSG& msg);

const char* protocolName(unsigned long protocol) {
    switch (protocol) {
    case CAN:
        return "CAN";
    case ISO15765:
        return "ISO15765";
    case J1850VPW:
        return "J1850VPW";
    case J1850PWM:
        return "J1850PWM";
    case ISO9141:
        return "ISO9141";
    case ISO14230:
        return "ISO14230";
    default:
        return "UNKNOWN";
    }
}

const char* filterTypeName(unsigned long type) {
    switch (type) {
    case PASS_FILTER:
        return "PASS";
    case BLOCK_FILTER:
        return "BLOCK";
    case FLOW_CONTROL_FILTER:
        return "FLOW_CONTROL";
    default:
        return "UNKNOWN";
    }
}

const char* ioctlName(unsigned long ioctlId) {
    switch (ioctlId) {
    case GET_CONFIG:
        return "GET_CONFIG";
    case SET_CONFIG:
        return "SET_CONFIG";
    case READ_VBATT:
        return "READ_VBATT";
    case CLEAR_TX_BUFFER:
        return "CLEAR_TX_BUFFER";
    case CLEAR_RX_BUFFER:
        return "CLEAR_RX_BUFFER";
    case CLEAR_PERIODIC_MSGS:
        return "CLEAR_PERIODIC_MSGS";
    case CLEAR_MSG_FILTERS:
        return "CLEAR_MSG_FILTERS";
    case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
        return "CLEAR_FUNCT_MSG_LOOKUP_TABLE";
    case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
        return "ADD_TO_FUNCT_MSG_LOOKUP_TABLE";
    case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
        return "DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE";
    case READ_PROG_VOLTAGE:
        return "READ_PROG_VOLTAGE";
    default:
        return "UNKNOWN";
    }
}

const char* configParameterName(unsigned long parameter) {
    switch (parameter) {
    case DATA_RATE:
        return "DATA_RATE";
    case LOOPBACK:
        return "LOOPBACK";
    case NODE_ADDRESS:
        return "NODE_ADDRESS";
    case NETWORK_LINE:
        return "NETWORK_LINE";
    case P1_MIN:
        return "P1_MIN";
    case P1_MAX:
        return "P1_MAX";
    case P2_MIN:
        return "P2_MIN";
    case P2_MAX:
        return "P2_MAX";
    case P3_MIN:
        return "P3_MIN";
    case P3_MAX:
        return "P3_MAX";
    case P4_MIN:
        return "P4_MIN";
    case P4_MAX:
        return "P4_MAX";
    case W0:
        return "W0";
    case W1:
        return "W1";
    case W2:
        return "W2";
    case W3:
        return "W3";
    case W4:
        return "W4";
    case W5:
        return "W5";
    case TIDLE:
        return "TIDLE";
    case TINIL:
        return "TINIL";
    case TWUP:
        return "TWUP";
    case PARITY:
        return "PARITY";
    case BIT_SAMPLE_POINT:
        return "BIT_SAMPLE_POINT";
    case SYNC_JUMP_WIDTH:
        return "SYNC_JUMP_WIDTH";
    case ISO15765_BS:
        return "ISO15765_BS";
    case ISO15765_STMIN:
        return "ISO15765_STMIN";
    case ISO15765_WFT_MAX:
        return "ISO15765_WFT_MAX";
    case CAN_MIXED_FORMAT:
        return "CAN_MIXED_FORMAT";
    case J1962_PINS:
        return "J1962_PINS";
    default:
        return "UNKNOWN";
    }
}

std::string hexBytes(const unsigned char* data, size_t size, size_t maxBytes = 64) {
    if (!data || size == 0) {
        return "";
    }
    const size_t n = std::min(size, maxBytes);
    std::string out;
    out.reserve(n * 3 + 32);
    char byteText[4]{};
    for (size_t i = 0; i < n; ++i) {
        if (i != 0) {
            out.push_back(' ');
        }
        std::snprintf(byteText, sizeof(byteText), "%02X", static_cast<unsigned>(data[i]));
        out += byteText;
    }
    if (size > maxBytes) {
        char suffix[40]{};
        std::snprintf(suffix, sizeof(suffix), " ...(+%lu)", static_cast<unsigned long>(size - maxBytes));
        out += suffix;
    }
    return out;
}

std::string passthruPayloadHex(const PASSTHRU_MSG& msg, size_t maxBytes = 64) {
    if (msg.DataSize <= 4) {
        return "";
    }
    const size_t safeDataSize = std::min<size_t>(msg.DataSize, sizeof(msg.Data));
    if (safeDataSize <= 4) {
        return "";
    }
    return hexBytes(msg.Data + 4, safeDataSize - 4, maxBytes);
}

void logPassThruMsg(const char* function, const char* direction, unsigned long channelId, const PASSTHRU_MSG& msg) {
    const uint32_t canId = readCanId(msg);
    const unsigned long payloadSize = msg.DataSize >= 4 ? msg.DataSize - 4 : 0;
    const bool extended = ((msg.TxFlags | msg.RxStatus) & CAN_29BIT_ID) != 0;
    logEvent(function,
             "%s ChannelID=%lu Protocol=%s(0x%08lX) CAN=0x%08lX Extended=%u TxFlags=0x%08lX RxStatus=0x%08lX DataSize=%lu PayloadLen=%lu Payload=[%s]",
             direction,
             channelId,
             protocolName(msg.ProtocolID),
             msg.ProtocolID,
             static_cast<unsigned long>(canId),
             extended ? 1u : 0u,
             msg.TxFlags,
             msg.RxStatus,
             msg.DataSize,
             payloadSize,
             passthruPayloadHex(msg).c_str());
}

void logCanFrame(const char* function, const char* direction, unsigned long channelId, const picoj_can_frame_t& frame) {
    logEvent(function,
             "%s ChannelID=%lu CAN=0x%08lX Extended=%u RTR=%u DLC=%u Data=[%s]",
             direction,
             channelId,
             static_cast<unsigned long>(frame.can_id),
             (frame.flags & PICOJ_CAN_EXTENDED) ? 1u : 0u,
             (frame.flags & PICOJ_CAN_RTR) ? 1u : 0u,
             frame.dlc,
             hexBytes(frame.data, frame.dlc, 8).c_str());
}

void logFilterMsg(const char* label, const PASSTHRU_MSG* msg) {
    if (!msg) {
        logEvent("PassThruStartMsgFilter", "%s=null", label);
        return;
    }
    logEvent("PassThruStartMsgFilter",
             "%s Protocol=%s(0x%08lX) CAN=0x%08lX TxFlags=0x%08lX RxStatus=0x%08lX DataSize=%lu Payload=[%s]",
             label,
             protocolName(msg->ProtocolID),
             msg->ProtocolID,
             static_cast<unsigned long>(readCanId(*msg)),
             msg->TxFlags,
             msg->RxStatus,
             msg->DataSize,
             passthruPayloadHex(*msg).c_str());
}

void logConfigList(const char* direction, unsigned long channelId, const SCONFIG_LIST* list) {
    if (!list) {
        logEvent("PassThruIoctl", "%s ChannelID=%lu ConfigList=null", direction, channelId);
        return;
    }
    logEvent("PassThruIoctl",
             "%s ChannelID=%lu NumOfParams=%lu ConfigPtr=%p",
             direction,
             channelId,
             list->NumOfParams,
             static_cast<void*>(list->ConfigPtr));
    if (!list->ConfigPtr) {
        return;
    }
    for (unsigned long i = 0; i < list->NumOfParams; ++i) {
        const SCONFIG& config = list->ConfigPtr[i];
        logEvent("PassThruIoctl",
                 "%s[%lu] ChannelID=%lu Parameter=%s(0x%08lX) Value=0x%08lX(%lu)",
                 direction,
                 i,
                 channelId,
                 configParameterName(config.Parameter),
                 config.Parameter,
                 config.Value,
                 config.Value);
    }
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

bool frameMatchesFilter(const picoj_can_frame_t& frame, const MessageFilter& filter) {
    if (!filter.active) {
        return false;
    }
    if (((frame.can_id ^ filter.patternCanId) & filter.maskCanId) != 0) {
        return false;
    }
    if (!filter.maskExtended) {
        return true;
    }
    return ((frame.flags & PICOJ_CAN_EXTENDED) != 0) == filter.patternExtended;
}

void logFilteredFrame(unsigned long channelId, const picoj_can_frame_t& frame, const char* reason) {
    static DWORD lastLogMs = 0;
    static unsigned long suppressed = 0;

    const DWORD now = GetTickCount();
    if (lastLogMs == 0 || now - lastLogMs >= 1000) {
        logEvent("PassThruReadMsgs",
                 "filtered ChannelID=%lu CAN=0x%08lX Extended=%u DLC=%u Data=[%s] Reason=%s SuppressedFilteredFrames=%lu",
                 channelId,
                 static_cast<unsigned long>(frame.can_id),
                 (frame.flags & PICOJ_CAN_EXTENDED) ? 1u : 0u,
                 frame.dlc,
                 hexBytes(frame.data, frame.dlc, 8).c_str(),
                 reason,
                 suppressed);
        suppressed = 0;
        lastLogMs = now;
    } else {
        ++suppressed;
    }
}

bool framePassesFilters(const std::array<MessageFilter, kMaxFilters>& filters,
                        unsigned long channelId,
                        const picoj_can_frame_t& frame) {
    for (const auto& filter : filters) {
        if (filter.active && filter.type == BLOCK_FILTER && frameMatchesFilter(frame, filter)) {
            if (traceFrameTraffic()) {
                logEvent("PassThruReadMsgs",
                         "RX BLOCKED ChannelID=%lu FilterID=%lu CAN=0x%08lX Pattern=0x%08lX Mask=0x%08lX Data=[%s]",
                         channelId,
                         filter.id,
                         static_cast<unsigned long>(frame.can_id),
                         static_cast<unsigned long>(filter.patternCanId),
                         static_cast<unsigned long>(filter.maskCanId),
                         hexBytes(frame.data, frame.dlc, 8).c_str());
            }
            logFilteredFrame(channelId, frame, "BLOCK_FILTER");
            return false;
        }
    }

    bool hasPassFilter = false;
    for (const auto& filter : filters) {
        if (!filter.active || filter.type != PASS_FILTER) {
            continue;
        }
        hasPassFilter = true;
        if (frameMatchesFilter(frame, filter)) {
            if (traceFrameTraffic()) {
                logEvent("PassThruReadMsgs",
                         "RX PASS ChannelID=%lu FilterID=%lu CAN=0x%08lX Pattern=0x%08lX Mask=0x%08lX Data=[%s]",
                         channelId,
                         filter.id,
                         static_cast<unsigned long>(frame.can_id),
                         static_cast<unsigned long>(filter.patternCanId),
                         static_cast<unsigned long>(filter.maskCanId),
                         hexBytes(frame.data, frame.dlc, 8).c_str());
            }
            return true;
        }
    }
    if (hasPassFilter) {
        logFilteredFrame(channelId, frame, "PASS_FILTER_MISS");
    }
    return !hasPassFilter;
}

void logRxQueueOverflow(unsigned long channelId) {
    static DWORD lastLogMs = 0;
    static unsigned long suppressed = 0;

    const DWORD now = GetTickCount();
    if (lastLogMs == 0 || now - lastLogMs >= 1000) {
        logEvent("PassThruReadMsgs",
                 "RX queue overflow ChannelID=%lu SuppressedOverflows=%lu",
                 channelId,
                 suppressed);
        suppressed = 0;
        lastLogMs = now;
    } else {
        ++suppressed;
    }
}

void queueRxFrame(std::deque<QueuedCanFrame>& queue, unsigned long channelId, const picoj_can_frame_t& frame, unsigned long rxStatus = 0) {
    if (queue.size() >= kMaxRxQueueFrames) {
        queue.pop_front();
        logRxQueueOverflow(channelId);
    }
    QueuedCanFrame queued{};
    queued.frame = frame;
    queued.rxStatus = rxStatus;
    queue.push_back(queued);
    if ((rxStatus & TX_MSG_TYPE) || traceFrameTraffic()) {
        logEvent("PassThruReadMsgs",
                 "%s ChannelID=%lu CAN=0x%08lX Extended=%u RTR=%u DLC=%u RxStatus=0x%08lX Data=[%s]",
                 (rxStatus & TX_MSG_TYPE) ? "TX LOOPBACK QUEUED" : "RX QUEUED",
                 channelId,
                 static_cast<unsigned long>(frame.can_id),
                 (frame.flags & PICOJ_CAN_EXTENDED) ? 1u : 0u,
                 (frame.flags & PICOJ_CAN_RTR) ? 1u : 0u,
                 frame.dlc,
                 rxStatus,
                 hexBytes(frame.data, frame.dlc, 8).c_str());
    }
}

void queueTxLoopbackFrame(unsigned long channelId, const picoj_can_frame_t& frame) {
    // J2534 LOOPBACK returns successfully transmitted messages to the read queue with TX_MSG_TYPE.
    queueRxFrame(g_channel.rxQueue, channelId, frame, TX_MSG_TYPE);
}

void queueIsoTpFrameLocked(IsoTpFrame&& frame) {
    if (g_channel.isoRxQueue.size() >= kMaxIsoRxQueueFrames) {
        g_channel.isoRxQueue.pop_front();
        logRxQueueOverflow(kChannelId);
    }
    g_channel.isoRxQueue.push_back(std::move(frame));
}

void fanoutCanFrameLocked(const picoj_can_frame_t& frame) {
    if (traceFrameTraffic()) {
        logCanFrame("PassThruReadMsgs", "RX USB_CAN", kChannelId, frame);
    }
    if (g_channel.connected && framePassesFilters(g_channel.filters, kChannelId, frame)) {
        queueRxFrame(g_channel.rxQueue, kChannelId, frame);
    }
}

bool popRxFrameLocked(unsigned long channelId, QueuedCanFrame& frame) {
    std::deque<QueuedCanFrame>* queue = nullptr;
    if (channelId == kChannelId && g_channel.connected) {
        queue = &g_channel.rxQueue;
    }
    if (!queue || queue->empty()) {
        return false;
    }
    frame = queue->front();
    queue->pop_front();
    return true;
}

unsigned remainingTimeoutMs(DWORD start, unsigned timeoutMs) {
    const DWORD elapsed = GetTickCount() - start;
    return elapsed >= timeoutMs ? 0 : timeoutMs - elapsed;
}

const MessageFilter* findTxFlowControlFilter(uint32_t requestCanId, bool extended) {
    for (const auto& filter : g_channel.filters) {
        if (filter.active && filter.flowControl && filter.flowControlCanId == requestCanId &&
            filter.flowControlExtended == extended) {
            return &filter;
        }
    }
    return nullptr;
}

bool hasActiveFlowControlFilter() {
    return std::any_of(g_channel.filters.begin(), g_channel.filters.end(), [](const MessageFilter& filter) {
        return filter.active && filter.flowControl;
    });
}

long configureFirmwareIsoTp(uint32_t txCanId, bool txExtended, uint32_t rxCanId, bool rxExtended, uint32_t flowControlCanId) {
    if (g_channel.firmwareIsoTpConfigured && g_channel.firmwareIsoTpTxCanId == txCanId &&
        g_channel.firmwareIsoTpRxCanId == rxCanId && g_channel.firmwareIsoTpFlowControlCanId == flowControlCanId &&
        g_channel.firmwareIsoTpTxExtended == txExtended && g_channel.firmwareIsoTpRxExtended == rxExtended) {
        return STATUS_NOERROR;
    }

    picoj_isotp_config_t config{};
    config.tx_can_id = txCanId;
    config.rx_can_id = rxCanId;
    config.flow_control_can_id = flowControlCanId;
    config.tx_flags = txExtended ? PICOJ_CAN_EXTENDED : 0;
    config.rx_flags = rxExtended ? PICOJ_CAN_EXTENDED : 0;

    logEvent("ISOTP_CONFIG",
             "TX_CAN=0x%08lX TX_Extended=%u RX_CAN=0x%08lX RX_Extended=%u FC_CAN=0x%08lX",
             static_cast<unsigned long>(txCanId),
             txExtended ? 1u : 0u,
             static_cast<unsigned long>(rxCanId),
             rxExtended ? 1u : 0u,
             static_cast<unsigned long>(flowControlCanId));

    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_ISOTP_CONFIG, &config, sizeof(config), response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        if (!g_usb.isOpen()) {
            g_channel = ChannelState{};
            return ERR_DEVICE_NOT_CONNECTED;
        }
        return g_usb.lastErrorWasTimeout() ? ERR_TIMEOUT : ERR_FAILED;
    }
    if (response.cmd == PICOJ_CMD_STATUS && response.len >= sizeof(picoj_status_t)) {
        picoj_status_t status{};
        std::memcpy(&status, response.payload, sizeof(status));
        if (status.code == 0) {
            if (txCanId == 0 || rxCanId == 0) {
                logEvent("ISOTP_CONFIG", "firmware ISO-TP disabled");
                g_channel.firmwareIsoTpConfigured = false;
                g_channel.firmwareIsoTpTxCanId = 0;
                g_channel.firmwareIsoTpRxCanId = 0;
                g_channel.firmwareIsoTpFlowControlCanId = 0;
                g_channel.firmwareIsoTpTxExtended = false;
                g_channel.firmwareIsoTpRxExtended = false;
                return STATUS_NOERROR;
            }
            g_channel.firmwareIsoTpConfigured = true;
            g_channel.firmwareIsoTpTxCanId = txCanId;
            g_channel.firmwareIsoTpRxCanId = rxCanId;
            g_channel.firmwareIsoTpFlowControlCanId = flowControlCanId;
            g_channel.firmwareIsoTpTxExtended = txExtended;
            g_channel.firmwareIsoTpRxExtended = rxExtended;
            logEvent("ISOTP_CONFIG", "firmware ISO-TP configured ok");
            return STATUS_NOERROR;
        }
        logEvent("ISOTP_CONFIG", "firmware rejected code=%ld detail=0x%08lX", static_cast<long>(status.code), static_cast<unsigned long>(status.detail));
    }
    setLastError("Firmware rejected ISO-TP configuration");
    return ERR_FAILED;
}

long sendCan(const picoj_can_frame_t& frame, unsigned timeoutMs) {
    const unsigned totalTimeout = timeoutMs ? timeoutMs : kDefaultCanTxTimeoutMs;
    const DWORD start = GetTickCount();

    for (;;) {
        const unsigned remaining = remainingTimeoutMs(start, totalTimeout);
        if (remaining == 0) {
            setLastError("Timed out waiting for CAN transmit buffer");
            logCanFrame("CAN_TX", "TIMEOUT", kChannelId, frame);
            return ERR_TIMEOUT;
        }

        picoj_packet_t response{};
        logCanFrame("CAN_TX", "USB_TX_REQUEST", kChannelId, frame);
        if (!g_usb.transact(PICOJ_CMD_CAN_TX, &frame, sizeof(frame), response, remaining)) {
            setLastError(g_usb.lastError());
            logEvent("CAN_TX",
                     "USB transact failed CAN=0x%08lX RemainingTimeout=%u LastError=%s",
                     static_cast<unsigned long>(frame.can_id),
                     remaining,
                     g_usb.lastError().c_str());
            if (!g_usb.isOpen()) {
                g_channel = ChannelState{};
                return ERR_DEVICE_NOT_CONNECTED;
            }
            if (g_usb.lastErrorWasTimeout()) {
                return ERR_TIMEOUT;
            }
            return ERR_FAILED;
        }

        if (response.cmd == PICOJ_CMD_STATUS && response.len >= sizeof(picoj_status_t)) {
            picoj_status_t status{};
            std::memcpy(&status, response.payload, sizeof(status));
            logEvent("CAN_TX",
                     "firmware status CAN=0x%08lX Code=%ld Detail=0x%08lX RemainingTimeout=%u",
                     static_cast<unsigned long>(frame.can_id),
                     static_cast<long>(status.code),
                     static_cast<unsigned long>(status.detail),
                     remaining);
            if (status.code == 0) {
                logCanFrame("CAN_TX", "OK", kChannelId, frame);
                return STATUS_NOERROR;
            }
            if (status.code == -3) {
                logEvent("CAN_TX", "firmware TX busy CAN=0x%08lX; retrying", static_cast<unsigned long>(frame.can_id));
                Sleep(1);
                continue;
            }
            if (status.code == -4) {
                setLastError("CAN transmit failed; check bus bitrate, MCP2515 oscillator setting, wiring, and bus acknowledgement");
                return ERR_FAILED;
            }
            setLastError("Firmware rejected CAN transmit");
            return ERR_FAILED;
        }

        return STATUS_NOERROR;
    }
}

long sendIsoTp(uint32_t canId, bool extended, const uint8_t* data, size_t size, unsigned timeoutMs);

long sendPeriodicMessage(const PASSTHRU_MSG& msg, unsigned long protocol, unsigned long channelFlags) {
    logPassThruMsg("PassThruStartPeriodicMsg", "PERIODIC_TX_DUE", kChannelId, msg);
    if (msg.DataSize < 4 || msg.DataSize > sizeof(msg.Data)) {
        setLastError("Invalid periodic message size");
        return ERR_INVALID_MSG;
    }

    const uint32_t canId = readCanId(msg);
    const bool extended = (msg.TxFlags & CAN_29BIT_ID) || (channelFlags & CAN_29BIT_ID);

    if (protocol == ISO15765) {
        if (msg.DataSize - 4 > 7) {
            setLastError("Periodic ISO-TP messages must fit in one CAN frame");
            return ERR_INVALID_MSG;
        }
        return sendIsoTp(canId, extended, msg.Data + 4, msg.DataSize - 4, kDefaultCanTxTimeoutMs);
    }

    if (msg.DataSize - 4 > 8) {
        setLastError("Periodic CAN payload exceeds 8 bytes");
        return ERR_INVALID_MSG;
    }

    picoj_can_frame_t frame{};
    frame.can_id = canId;
    frame.flags = extended ? PICOJ_CAN_EXTENDED : 0;
    frame.dlc = static_cast<uint8_t>(msg.DataSize - 4);
    std::memcpy(frame.data, msg.Data + 4, frame.dlc);
    logCanFrame("PassThruStartPeriodicMsg", "PERIODIC_CAN_FRAME", kChannelId, frame);
    return sendCan(frame, kDefaultCanTxTimeoutMs);
}

void periodicWorker() {
    std::unique_lock<std::mutex> lock(g_lock);
    while (!g_periodicStop) {
        const DWORD now = GetTickCount();
        DWORD nextWakeMs = 1000;
        bool foundDue = false;
        PeriodicMessage due{};
        unsigned long protocol = CAN;
        unsigned long flags = 0;

        if (g_channel.connected && g_usb.isOpen()) {
            for (auto& periodic : g_channel.periodic) {
                if (!periodic.active) {
                    continue;
                }
                const DWORD elapsedSinceDue = now - periodic.nextDueMs;
                if (elapsedSinceDue < 0x80000000ul) {
                    due = periodic;
                    periodic.nextDueMs = now + periodic.intervalMs;
                    protocol = g_channel.protocol;
                    flags = g_channel.flags;
                    foundDue = true;
                    break;
                }
                const DWORD untilDue = periodic.nextDueMs - now;
                nextWakeMs = std::min<DWORD>(nextWakeMs, untilDue);
            }
        }

        if (!foundDue) {
            g_periodicCv.wait_for(lock, std::chrono::milliseconds(nextWakeMs));
            continue;
        }

        lock.unlock();
        sendPeriodicMessage(due.msg, protocol, flags);
        lock.lock();
    }
}

void ensurePeriodicThread() {
    if (!g_periodicThread.joinable()) {
        g_periodicStop = false;
        g_periodicThread = std::thread(periodicWorker);
    }
}

void stopPeriodicThread() {
    {
        std::lock_guard<std::mutex> lock(g_lock);
        g_periodicStop = true;
        g_periodicCv.notify_all();
    }
    if (g_periodicThread.joinable()) {
        g_periodicThread.join();
    }
}

struct PeriodicThreadShutdown {
    ~PeriodicThreadShutdown() {
        stopPeriodicThread();
    }
};

PeriodicThreadShutdown g_periodicThreadShutdown;

long setBitrate(unsigned long bitrate) {
    picoj_bitrate_t request{bitrate};
    picoj_packet_t response{};
    if (!g_usb.transact(PICOJ_CMD_SET_BITRATE, &request, sizeof(request), response, kDefaultTimeoutMs)) {
        setLastError(g_usb.lastError());
        if (!g_usb.isOpen()) {
            g_channel = ChannelState{};
            return ERR_DEVICE_NOT_CONNECTED;
        }
        if (g_usb.lastErrorWasTimeout()) {
            return ERR_TIMEOUT;
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

long sendIsoTp(uint32_t canId, bool extended, const uint8_t* data, size_t size, unsigned timeoutMs) {
    if (size > 4095) {
        setLastError("ISO-TP payload exceeds 4095 bytes");
        logEvent("ISOTP_TX",
                 "invalid payload length CAN=0x%08lX Extended=%u PayloadLen=%lu",
                 static_cast<unsigned long>(canId),
                 extended ? 1u : 0u,
                 static_cast<unsigned long>(size));
        return ERR_INVALID_MSG;
    }

    logEvent("ISOTP_TX",
             "request CAN=0x%08lX Extended=%u PayloadLen=%lu Payload=[%s]",
             static_cast<unsigned long>(canId),
             extended ? 1u : 0u,
             static_cast<unsigned long>(size),
             hexBytes(data, size).c_str());

    const MessageFilter* flowFilter = findTxFlowControlFilter(canId, extended);
    if (flowFilter) {
        logEvent("ISOTP_TX",
                 "using FC FilterID=%lu RequestCAN=0x%08lX RequestExtended=%u ResponseCAN=0x%08lX ResponseExtended=%u FlowControlCAN=0x%08lX FlowControlExtended=%u",
                 flowFilter->id,
                 static_cast<unsigned long>(canId),
                 extended ? 1u : 0u,
                 static_cast<unsigned long>(flowFilter->patternCanId),
                 flowFilter->patternExtended ? 1u : 0u,
                 static_cast<unsigned long>(flowFilter->flowControlCanId),
                 flowFilter->flowControlExtended ? 1u : 0u);
        long status = configureFirmwareIsoTp(canId,
                                             extended,
                                             flowFilter->patternCanId,
                                             flowFilter->patternExtended,
                                             flowFilter->flowControlCanId);
        if (status != STATUS_NOERROR) {
            return status;
        }

        const unsigned totalTimeout = timeoutMs ? timeoutMs : kDefaultTimeoutMs;
        const DWORD start = GetTickCount();
        uint16_t offset = 0;
        do {
            picoj_isotp_chunk_t chunk{};
            chunk.can_id = canId;
            chunk.total_len = static_cast<uint16_t>(size);
            chunk.offset = offset;
            chunk.flags = extended ? PICOJ_CAN_EXTENDED : 0;
            const size_t remainingSize = size - offset;
            chunk.chunk_len = static_cast<uint8_t>(std::min<size_t>(PICOJ_ISOTP_CHUNK_DATA_SIZE, remainingSize));
            if (chunk.chunk_len != 0) {
                std::memcpy(chunk.data, data + offset, chunk.chunk_len);
            }

            picoj_packet_t response{};
            const unsigned remaining = remainingTimeoutMs(start, totalTimeout);
            if (remaining == 0) {
                setLastError("ISO-TP USB transmit timed out");
                logEvent("ISOTP_TX",
                         "timeout before chunk CAN=0x%08lX Total=%u Offset=%u ChunkLen=%u",
                         static_cast<unsigned long>(chunk.can_id),
                         chunk.total_len,
                         chunk.offset,
                         chunk.chunk_len);
                return ERR_TIMEOUT;
            }
            logEvent("ISOTP_TX",
                     "USB_CHUNK_TX CAN=0x%08lX Extended=%u Total=%u Offset=%u ChunkLen=%u Data=[%s]",
                     static_cast<unsigned long>(chunk.can_id),
                     (chunk.flags & PICOJ_CAN_EXTENDED) ? 1u : 0u,
                     chunk.total_len,
                     chunk.offset,
                     chunk.chunk_len,
                     hexBytes(chunk.data, chunk.chunk_len).c_str());
            if (!g_usb.transact(PICOJ_CMD_ISOTP_TX, &chunk, sizeof(chunk), response, remaining)) {
                setLastError(g_usb.lastError());
                logEvent("ISOTP_TX",
                         "USB chunk transmit failed CAN=0x%08lX Total=%u Offset=%u ChunkLen=%u Error=%s",
                         static_cast<unsigned long>(chunk.can_id),
                         chunk.total_len,
                         chunk.offset,
                         chunk.chunk_len,
                         g_usb.lastError().c_str());
                if (!g_usb.isOpen()) {
                    g_channel = ChannelState{};
                    return ERR_DEVICE_NOT_CONNECTED;
                }
                return g_usb.lastErrorWasTimeout() ? ERR_TIMEOUT : ERR_FAILED;
            }
            if (response.cmd == PICOJ_CMD_STATUS && response.len >= sizeof(picoj_status_t)) {
                picoj_status_t txStatus{};
                std::memcpy(&txStatus, response.payload, sizeof(txStatus));
                logEvent("ISOTP_TX",
                         "firmware chunk status CAN=0x%08lX Total=%u Offset=%u ChunkLen=%u Code=%ld Detail=0x%08lX",
                         static_cast<unsigned long>(chunk.can_id),
                         chunk.total_len,
                         chunk.offset,
                         chunk.chunk_len,
                         static_cast<long>(txStatus.code),
                         static_cast<unsigned long>(txStatus.detail));
                if (txStatus.code != 0) {
                    setLastError(txStatus.code == -3 ? "Firmware ISO-TP transmitter is busy" : "Firmware rejected ISO-TP transmit");
                    return txStatus.code == -3 ? ERR_TIMEOUT : ERR_FAILED;
                }
            }
            if (size == 0) {
                break;
            }
            offset = static_cast<uint16_t>(offset + chunk.chunk_len);
        } while (offset < size);

        logEvent("ISOTP_TX",
                 "complete CAN=0x%08lX Extended=%u PayloadLen=%lu",
                 static_cast<unsigned long>(canId),
                 extended ? 1u : 0u,
                 static_cast<unsigned long>(size));
        return STATUS_NOERROR;
    }

    setLastError("No ISO-TP flow-control filter for firmware ISO-TP transmit");
    logEvent("ISOTP_TX",
             "no flow-control filter CAN=0x%08lX Extended=%u",
             static_cast<unsigned long>(canId),
             extended ? 1u : 0u);
    return ERR_NO_FLOW_CONTROL;
}

bool decodeCanPacket(const picoj_packet_t& packet, picoj_can_frame_t& frame) {
    if (packet.cmd != PICOJ_CMD_CAN_RX || packet.len < sizeof(picoj_can_frame_t)) {
        return false;
    }
    std::memcpy(&frame, packet.payload, sizeof(frame));
    return true;
}

bool acceptIsoTpPacketLocked(const picoj_packet_t& packet) {
    if (packet.cmd != PICOJ_CMD_ISOTP_RX || packet.len < sizeof(picoj_isotp_chunk_t) || !g_channel.connected ||
        g_channel.protocol != ISO15765) {
        return false;
    }

    picoj_isotp_chunk_t chunk{};
    std::memcpy(&chunk, packet.payload, sizeof(chunk));
    logEvent("PassThruReadMsgs",
             "RX USB_ISOTP_CHUNK CAN=0x%08lX Total=%u Offset=%u ChunkLen=%u Extended=%u Data=[%s]",
             static_cast<unsigned long>(chunk.can_id),
             chunk.total_len,
             chunk.offset,
             chunk.chunk_len,
             (chunk.flags & PICOJ_CAN_EXTENDED) ? 1u : 0u,
             hexBytes(chunk.data, chunk.chunk_len).c_str());
    if (chunk.total_len > PICOJ_ISOTP_MAX_PAYLOAD || chunk.chunk_len > PICOJ_ISOTP_CHUNK_DATA_SIZE ||
        chunk.offset > chunk.total_len || static_cast<uint16_t>(chunk.offset + chunk.chunk_len) > chunk.total_len) {
        logEvent("PassThruReadMsgs",
                 "invalid ISO-TP chunk CanId=0x%08lX Total=%u Offset=%u Chunk=%u",
                 static_cast<unsigned long>(chunk.can_id),
                 chunk.total_len,
                 chunk.offset,
                 chunk.chunk_len);
        return false;
    }

    if (chunk.offset == 0) {
        g_channel.isoChunkBuffer.assign(chunk.total_len, 0);
        g_channel.isoChunkExpected = chunk.total_len;
        g_channel.isoChunkReceived = 0;
        g_channel.isoChunkCanId = chunk.can_id;
        g_channel.isoChunkExtended = (chunk.flags & PICOJ_CAN_EXTENDED) != 0;
    }
    if (chunk.total_len != g_channel.isoChunkExpected || chunk.offset != g_channel.isoChunkReceived ||
        chunk.can_id != g_channel.isoChunkCanId) {
        g_channel.isoChunkBuffer.clear();
        g_channel.isoChunkExpected = 0;
        g_channel.isoChunkReceived = 0;
        setLastError("Out-of-order ISO-TP USB chunk");
        return false;
    }
    if (chunk.chunk_len != 0) {
        std::memcpy(g_channel.isoChunkBuffer.data() + chunk.offset, chunk.data, chunk.chunk_len);
    }
    g_channel.isoChunkReceived = static_cast<uint16_t>(g_channel.isoChunkReceived + chunk.chunk_len);

    if (g_channel.isoChunkReceived >= g_channel.isoChunkExpected) {
        IsoTpFrame iso{};
        iso.canId = g_channel.isoChunkCanId;
        iso.extended = g_channel.isoChunkExtended;
        iso.payload = g_channel.isoChunkBuffer;
        logEvent("PassThruReadMsgs",
                 "RX ISOTP_COMPLETE CAN=0x%08lX Extended=%u PayloadLen=%lu Payload=[%s]",
                 static_cast<unsigned long>(iso.canId),
                 iso.extended ? 1u : 0u,
                 static_cast<unsigned long>(iso.payload.size()),
                 hexBytes(iso.payload.data(), iso.payload.size()).c_str());
        queueIsoTpFrameLocked(std::move(iso));
        g_channel.isoChunkBuffer.clear();
        g_channel.isoChunkExpected = 0;
        g_channel.isoChunkReceived = 0;
    }
    return true;
}

bool popIsoTpFrameLocked(IsoTpFrame& frame) {
    if (!g_channel.connected || g_channel.isoRxQueue.empty()) {
        return false;
    }
    frame = std::move(g_channel.isoRxQueue.front());
    g_channel.isoRxQueue.pop_front();
    return true;
}

void fillIsoTpMessage(PASSTHRU_MSG& msg, const IsoTpFrame& iso) {
    std::memset(&msg, 0, sizeof(msg));
    msg.ProtocolID = ISO15765;
    msg.Timestamp = GetTickCount();
    msg.DataSize = static_cast<unsigned long>(4 + iso.payload.size());
    msg.ExtraDataIndex = msg.DataSize;
    writeCanId(msg, iso.canId);
    if (!iso.payload.empty()) {
        std::memcpy(msg.Data + 4, iso.payload.data(), iso.payload.size());
    }
    if (iso.extended) {
        msg.RxStatus |= CAN_29BIT_ID;
    }
}

void fillCanMessage(PASSTHRU_MSG& msg, unsigned long protocol, const picoj_can_frame_t& frame, unsigned long rxStatus = 0) {
    std::memset(&msg, 0, sizeof(msg));
    msg.ProtocolID = protocol;
    msg.RxStatus = rxStatus;
    msg.Timestamp = GetTickCount();
    msg.DataSize = 4 + frame.dlc;
    msg.ExtraDataIndex = msg.DataSize;
    writeCanId(msg, frame.can_id);
    std::memcpy(msg.Data + 4, frame.data, frame.dlc);
    if (frame.flags & PICOJ_CAN_EXTENDED) {
        msg.RxStatus |= CAN_29BIT_ID;
    }
}

long appendReadFrameLocked(unsigned long channelId, const QueuedCanFrame& queued, PASSTHRU_MSG& msg, bool& appended) {
    appended = false;
    if (g_channel.protocol == ISO15765) {
        return STATUS_NOERROR;
    }

    fillCanMessage(msg, CAN, queued.frame, queued.rxStatus);
    appended = true;
    return STATUS_NOERROR;
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
        const bool timedOut = g_usb.lastErrorWasTimeout();
        g_usb.close();
        return timedOut ? ERR_TIMEOUT : ERR_DEVICE_NOT_CONNECTED;
    }

    return STATUS_NOERROR;
}

} // namespace

extern "C" long WINAPI PassThruOpen(void*, unsigned long* pDeviceID) {
    logEvent("PassThruOpen", "DllVersion=%s DeviceIDOut=%p", kDllVersion, static_cast<void*>(pDeviceID));
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
    {
        std::lock_guard<std::mutex> lock(g_lock);
        long status = ensureDevice(DeviceID);
        if (status != STATUS_NOERROR) {
            return status;
        }
        g_channel = ChannelState{};
        g_periodicCv.notify_all();
    }
    stopPeriodicThread();
    {
        std::lock_guard<std::mutex> lock(g_lock);
        g_usb.close();
    }
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
    if (!isCanProtocol(ProtocolID)) {
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

    g_channel = ChannelState{};
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
    {
        std::lock_guard<std::mutex> lock(g_lock);
        long status = ensureChannel(ChannelID);
        if (status != STATUS_NOERROR) {
            return status;
        }
        g_channel = ChannelState{};
        g_periodicCv.notify_all();
    }
    stopPeriodicThread();
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout) {
    logReadMsgsCall(ChannelID, pMsg, pNumMsgs, Timeout);
    if (!pMsg || !pNumMsgs) {
        setLastError("Null read message argument");
        return ERR_NULL_PARAMETER;
    }

    {
        std::lock_guard<std::mutex> lock(g_lock);
        long status = ensureChannel(ChannelID);
        if (status != STATUS_NOERROR) {
            return status;
        }
    }

    const unsigned long requested = *pNumMsgs;
    *pNumMsgs = 0;

    const DWORD start = GetTickCount();
    const bool nonBlocking = Timeout == 0;

    while (*pNumMsgs < requested) {
        {
            std::lock_guard<std::mutex> lock(g_lock);
            long status = ensureChannel(ChannelID);
            if (status != STATUS_NOERROR) {
                return *pNumMsgs ? STATUS_NOERROR : status;
            }

            QueuedCanFrame queuedFrame{};
            IsoTpFrame queuedIso{};
            if (ChannelID == kChannelId && g_channel.protocol == ISO15765 && popIsoTpFrameLocked(queuedIso)) {
                fillIsoTpMessage(pMsg[*pNumMsgs], queuedIso);
                logPassThruMsg("PassThruReadMsgs", "DELIVER_ISOTP", ChannelID, pMsg[*pNumMsgs]);
                ++(*pNumMsgs);
                continue;
            }
            if (popRxFrameLocked(ChannelID, queuedFrame)) {
                bool appended = false;
                long status = appendReadFrameLocked(ChannelID, queuedFrame, pMsg[*pNumMsgs], appended);
                if (status != STATUS_NOERROR) {
                    return status;
                }
                if (appended) {
                    if (traceFrameTraffic() || (pMsg[*pNumMsgs].RxStatus & TX_MSG_TYPE)) {
                        logPassThruMsg("PassThruReadMsgs", "DELIVER_CAN", ChannelID, pMsg[*pNumMsgs]);
                    }
                    ++(*pNumMsgs);
                }
                continue;
            }
        }

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
        const unsigned readTimeout = nonBlocking ? 1u : std::min<unsigned>(kReadPollSliceMs, Timeout - elapsed);
        if (!g_usb.readPacket(packet, readTimeout)) {
            logEvent("PassThruReadMsgs",
                     "USB_READ_FAIL ChannelID=%lu Requested=%lu Delivered=%lu Timeout=%lu ReadTimeout=%u NonBlocking=%u UsbOpen=%u TimedOut=%u Error=%s",
                     ChannelID,
                     requested,
                     *pNumMsgs,
                     Timeout,
                     readTimeout,
                     nonBlocking ? 1u : 0u,
                     g_usb.isOpen() ? 1u : 0u,
                     g_usb.lastErrorWasTimeout() ? 1u : 0u,
                     g_usb.lastError().c_str());
            if (!g_usb.isOpen()) {
                std::lock_guard<std::mutex> lock(g_lock);
                setLastError(g_usb.lastError());
                g_channel = ChannelState{};
                return *pNumMsgs ? STATUS_NOERROR : ERR_DEVICE_NOT_CONNECTED;
            }
            if (g_usb.lastErrorWasTimeout() && !nonBlocking) {
                continue;
            }
            if (!g_usb.lastErrorWasTimeout()) {
                std::lock_guard<std::mutex> lock(g_lock);
                setLastError(g_usb.lastError());
            }
            return *pNumMsgs ? STATUS_NOERROR : ERR_BUFFER_EMPTY;
        }

        {
            std::lock_guard<std::mutex> lock(g_lock);
            if (!g_usb.isOpen()) {
                setLastError("Pico J2534 device is not connected");
                g_channel = ChannelState{};
                return *pNumMsgs ? STATUS_NOERROR : ERR_DEVICE_NOT_CONNECTED;
            }
            if (acceptIsoTpPacketLocked(packet)) {
                continue;
            }
            picoj_can_frame_t frame{};
            if (decodeCanPacket(packet, frame)) {
                fanoutCanFrameLocked(frame);
            }
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
        logPassThruMsg("PassThruWriteMsgs", "TX_REQUEST", ChannelID, msg);
        if (msg.DataSize < 4 || msg.DataSize > sizeof(msg.Data)) {
            setLastError("Invalid J2534 message size");
            logEvent("PassThruWriteMsgs", "TX_REJECT invalid DataSize=%lu Index=%lu", msg.DataSize, i);
            return ERR_INVALID_MSG;
        }

        const uint32_t canId = readCanId(msg);
        const bool extended = (msg.TxFlags & CAN_29BIT_ID) || (g_channel.flags & CAN_29BIT_ID);

        if (g_channel.protocol == ISO15765) {
            status = sendIsoTp(canId, extended, msg.Data + 4, msg.DataSize - 4, Timeout);
            if (status != STATUS_NOERROR) {
                logEvent("PassThruWriteMsgs",
                         "TX_RESULT Index=%lu Protocol=ISO15765 CAN=0x%08lX Status=0x%08lX Written=%lu",
                         i,
                         static_cast<unsigned long>(canId),
                         status,
                         *pNumMsgs);
                return status;
            }
        } else {
            if (msg.DataSize - 4 > 8) {
                setLastError("Classic CAN payload exceeds 8 bytes");
                logEvent("PassThruWriteMsgs",
                         "TX_REJECT classic CAN payload too large CAN=0x%08lX PayloadLen=%lu",
                         static_cast<unsigned long>(canId),
                         msg.DataSize - 4);
                return ERR_INVALID_MSG;
            }
            picoj_can_frame_t frame{};
            frame.can_id = canId;
            frame.flags = extended ? PICOJ_CAN_EXTENDED : 0;
            frame.dlc = static_cast<uint8_t>(msg.DataSize - 4);
            std::memcpy(frame.data, msg.Data + 4, frame.dlc);
            logCanFrame("PassThruWriteMsgs", "TX_CAN_FRAME", ChannelID, frame);
            status = sendCan(frame, Timeout);
            if (status != STATUS_NOERROR) {
                logEvent("PassThruWriteMsgs",
                         "TX_RESULT Index=%lu Protocol=CAN CAN=0x%08lX Status=0x%08lX Written=%lu",
                         i,
                         static_cast<unsigned long>(canId),
                         status,
                         *pNumMsgs);
                return status;
            }
            if (g_channel.loopback) {
                queueTxLoopbackFrame(ChannelID, frame);
            }
        }

        ++(*pNumMsgs);
        logEvent("PassThruWriteMsgs",
                 "TX_RESULT Index=%lu CAN=0x%08lX Status=0x%08lX Written=%lu",
                 i,
                 static_cast<unsigned long>(canId),
                 STATUS_NOERROR,
                 *pNumMsgs);
    }

    logEvent("PassThruWriteMsgs", "complete Requested=%lu Written=%lu Status=0x%08lX", requested, *pNumMsgs, STATUS_NOERROR);
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pMsgID, unsigned long TimeInterval) {
    logEvent("PassThruStartPeriodicMsg",
             "ChannelID=%lu Msg=%p MsgID=%p TimeInterval=%lu",
             ChannelID,
             static_cast<void*>(pMsg),
             static_cast<void*>(pMsgID),
             TimeInterval);
    if (!pMsg || !pMsgID) {
        setLastError("Null periodic message argument");
        return ERR_NULL_PARAMETER;
    }
    if (TimeInterval == 0) {
        setLastError("Invalid periodic message interval");
        return ERR_INVALID_TIME_INTERVAL;
    }
    if (pMsg->DataSize < 4 || pMsg->DataSize > sizeof(pMsg->Data)) {
        setLastError("Invalid periodic message size");
        return ERR_INVALID_MSG;
    }
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (g_channel.protocol == CAN && pMsg->DataSize - 4 > 8) {
        setLastError("Periodic CAN payload exceeds 8 bytes");
        return ERR_INVALID_MSG;
    }
    if (g_channel.protocol == ISO15765 && pMsg->DataSize - 4 > 7) {
        setLastError("Periodic ISO-TP messages must fit in one CAN frame");
        return ERR_INVALID_MSG;
    }

    auto slot = std::find_if(g_channel.periodic.begin(), g_channel.periodic.end(), [](const PeriodicMessage& periodic) {
        return !periodic.active;
    });
    if (slot == g_channel.periodic.end()) {
        setLastError("Periodic message limit exceeded");
        return ERR_EXCEEDED_LIMIT;
    }

    PeriodicMessage periodic{};
    periodic.active = true;
    periodic.id = g_channel.nextPeriodicId++;
    periodic.msg = *pMsg;
    periodic.intervalMs = TimeInterval;
    periodic.nextDueMs = GetTickCount() + TimeInterval;
    *slot = periodic;
    *pMsgID = periodic.id;

    ensurePeriodicThread();
    g_periodicCv.notify_all();
    logPassThruMsg("PassThruStartPeriodicMsg", "STARTED", ChannelID, *pMsg);
    logEvent("PassThruStartPeriodicMsg",
             "ok MsgID=%lu TimeInterval=%lu",
             periodic.id,
             TimeInterval);
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID) {
    logEvent("PassThruStopPeriodicMsg", "ChannelID=%lu MsgID=%lu", ChannelID, MsgID);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    for (auto& periodic : g_channel.periodic) {
        if (periodic.active && periodic.id == MsgID) {
            logPassThruMsg("PassThruStopPeriodicMsg", "STOPPED", ChannelID, periodic.msg);
            periodic = PeriodicMessage{};
            g_periodicCv.notify_all();
            return STATUS_NOERROR;
        }
    }
    setLastError("Invalid periodic message id");
    return ERR_INVALID_MSG_ID;
}

extern "C" long WINAPI PassThruStartMsgFilter(unsigned long ChannelID,
                                               unsigned long FilterType,
                                               PASSTHRU_MSG* pMaskMsg,
                                               PASSTHRU_MSG* pPatternMsg,
                                               PASSTHRU_MSG* pFlowControlMsg,
                                               unsigned long* pFilterID) {
    logEvent("PassThruStartMsgFilter",
             "ChannelID=%lu FilterType=%lu(%s) MaskMsg=%p PatternMsg=%p FlowControlMsg=%p FilterIDOut=%p",
             ChannelID,
             FilterType,
             filterTypeName(FilterType),
             static_cast<void*>(pMaskMsg),
             static_cast<void*>(pPatternMsg),
             static_cast<void*>(pFlowControlMsg),
             static_cast<void*>(pFilterID));
    logFilterMsg("MaskMsg", pMaskMsg);
    logFilterMsg("PatternMsg", pPatternMsg);
    logFilterMsg("FlowControlMsg", pFlowControlMsg);
    if (!pFilterID) {
        setLastError("Null filter id pointer");
        return ERR_NULL_PARAMETER;
    }
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (FilterType != PASS_FILTER && FilterType != BLOCK_FILTER && FilterType != FLOW_CONTROL_FILTER) {
        logUnsupported("PassThruStartMsgFilter", "ChannelID=%lu unsupported FilterType=%lu", ChannelID, FilterType);
        setLastError("Only pass, block, and flow-control filters are accepted");
        return ERR_NOT_SUPPORTED;
    }
    if (!pMaskMsg || !pPatternMsg || pMaskMsg->DataSize < 4 || pPatternMsg->DataSize < 4) {
        setLastError("Invalid filter mask or pattern message");
        return ERR_INVALID_MSG;
    }

    auto slot = std::find_if(g_channel.filters.begin(), g_channel.filters.end(), [](const MessageFilter& filter) {
        return !filter.active;
    });
    if (slot == g_channel.filters.end()) {
        setLastError("Message filter limit exceeded");
        return ERR_EXCEEDED_LIMIT;
    }

    MessageFilter filter{};
    filter.active = true;
    filter.id = g_channel.nextFilterId++;
    filter.type = FilterType;
    filter.maskCanId = readCanId(*pMaskMsg);
    filter.patternCanId = readCanId(*pPatternMsg);
    filter.maskExtended = ((pMaskMsg->TxFlags | pMaskMsg->RxStatus) & CAN_29BIT_ID) != 0;
    filter.patternExtended = ((pPatternMsg->TxFlags | pPatternMsg->RxStatus) & CAN_29BIT_ID) != 0;

    if (FilterType == FLOW_CONTROL_FILTER) {
        if (!pFlowControlMsg) {
            setLastError("Null flow-control message");
            return ERR_NULL_PARAMETER;
        }
        if (pFlowControlMsg->DataSize < 4) {
            setLastError("Invalid flow-control message");
            return ERR_INVALID_MSG;
        }
        filter.flowControl = true;
        filter.flowControlCanId = readCanId(*pFlowControlMsg);
        filter.flowControlExtended = ((pFlowControlMsg->TxFlags | pFlowControlMsg->RxStatus) & CAN_29BIT_ID) != 0;
    }

    *slot = filter;
    *pFilterID = filter.id;
    logEvent("PassThruStartMsgFilter",
             "ok FilterID=%lu Type=%lu(%s) MaskCanId=0x%08lX PatternCanId=0x%08lX MaskExtended=%u PatternExtended=%u FlowControlCanId=0x%08lX FlowControlExtended=%u",
             filter.id,
             filter.type,
             filterTypeName(filter.type),
             static_cast<unsigned long>(filter.maskCanId),
             static_cast<unsigned long>(filter.patternCanId),
             filter.maskExtended ? 1u : 0u,
             filter.patternExtended ? 1u : 0u,
             static_cast<unsigned long>(filter.flowControlCanId),
             filter.flowControlExtended ? 1u : 0u);
    return STATUS_NOERROR;
}

extern "C" long WINAPI PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID) {
    logEvent("PassThruStopMsgFilter", "ChannelID=%lu FilterID=%lu", ChannelID, FilterID);
    std::lock_guard<std::mutex> lock(g_lock);
    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    for (auto& filter : g_channel.filters) {
        if (filter.active && filter.id == FilterID) {
            const bool wasFlowControl = filter.flowControl;
            logEvent("PassThruStopMsgFilter",
                     "stopped FilterID=%lu Type=%lu(%s) MaskCanId=0x%08lX PatternCanId=0x%08lX MaskExtended=%u PatternExtended=%u FlowControlCanId=0x%08lX FlowControlExtended=%u",
                     filter.id,
                     filter.type,
                     filterTypeName(filter.type),
                     static_cast<unsigned long>(filter.maskCanId),
                     static_cast<unsigned long>(filter.patternCanId),
                     filter.maskExtended ? 1u : 0u,
                     filter.patternExtended ? 1u : 0u,
                     static_cast<unsigned long>(filter.flowControlCanId),
                     filter.flowControlExtended ? 1u : 0u);
            filter = MessageFilter{};
            if (wasFlowControl && !hasActiveFlowControlFilter()) {
                return configureFirmwareIsoTp(0, false, 0, false, 0);
            }
            return STATUS_NOERROR;
        }
    }
    setLastError("Invalid message filter id");
    return ERR_INVALID_FILTER_ID;
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
        std::strcpy(pDllVersion, kDllVersion);
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
             "ChannelID=%lu IoctlID=%s(0x%08lX) Input=%p Output=%p",
             ChannelID,
             ioctlName(IoctlID),
             IoctlID,
             pInput,
             pOutput);
    std::lock_guard<std::mutex> lock(g_lock);
    if (IoctlID == READ_VBATT || IoctlID == READ_PROG_VOLTAGE) {
        if (!g_usb.isOpen()) {
            setLastError("Pico J2534 device is not open");
            return ERR_DEVICE_NOT_CONNECTED;
        }
        if (!pOutput) {
            setLastError("Null voltage output pointer");
            return ERR_NULL_PARAMETER;
        }
        logUnsupported("PassThruIoctl",
                       "%s is unavailable because the hardware does not measure J1962 voltage",
                       ioctlName(IoctlID));
        setLastError("Vehicle voltage measurement is not supported");
        return ERR_NOT_SUPPORTED;
    }

    long status = ensureChannel(ChannelID);
    if (status != STATUS_NOERROR) {
        return status;
    }
    if (IoctlID == GET_CONFIG) {
        auto* list = static_cast<SCONFIG_LIST*>(pInput);
        logConfigList("GET_CONFIG request", ChannelID, list);
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
                config.Value = g_channel.loopback ? 1 : 0;
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
        logConfigList("GET_CONFIG response", ChannelID, list);
        return STATUS_NOERROR;
    }
    if (IoctlID == SET_CONFIG) {
        auto* list = static_cast<SCONFIG_LIST*>(pInput);
        logConfigList("SET_CONFIG request", ChannelID, list);
        if (!list || !list->ConfigPtr) {
            setLastError("Null SET_CONFIG input");
            return ERR_NULL_PARAMETER;
        }
        for (unsigned long i = 0; i < list->NumOfParams; ++i) {
            const SCONFIG& config = list->ConfigPtr[i];
            switch (config.Parameter) {
            case DATA_RATE: {
                if (config.Value == g_channel.baudrate) {
                    logEvent("PassThruIoctl",
                             "SET_CONFIG DATA_RATE unchanged ChannelID=%lu Value=%lu",
                             ChannelID,
                             config.Value);
                } else {
                    status = setBitrate(config.Value);
                    if (status != STATUS_NOERROR) {
                        return status;
                    }
                }
                break;
            }
            case LOOPBACK:
                g_channel.loopback = config.Value != 0;
                logEvent("PassThruIoctl",
                         "SET_CONFIG LOOPBACK ChannelID=%lu Enabled=%u",
                         ChannelID,
                         g_channel.loopback ? 1u : 0u);
                break;
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
        logEvent("PassThruIoctl", "SET_CONFIG complete ChannelID=%lu", ChannelID);
        return STATUS_NOERROR;
    }
    if (IoctlID == CLEAR_MSG_FILTERS) {
        logEvent("PassThruIoctl", "CLEAR_MSG_FILTERS ChannelID=%lu", ChannelID);
        g_channel.filters = {};
        g_channel.nextFilterId = 1;
        g_channel.isoRxQueue.clear();
        g_channel.isoChunkBuffer.clear();
        g_channel.isoChunkExpected = 0;
        g_channel.isoChunkReceived = 0;
        return configureFirmwareIsoTp(0, false, 0, false, 0);
    }
    if (IoctlID == CLEAR_RX_BUFFER) {
        logEvent("PassThruIoctl",
                 "CLEAR_RX_BUFFER ChannelID=%lu QueuedCAN=%lu QueuedISOTP=%lu PendingIsoBytes=%lu",
                 ChannelID,
                 static_cast<unsigned long>(g_channel.rxQueue.size()),
                 static_cast<unsigned long>(g_channel.isoRxQueue.size()),
                 static_cast<unsigned long>(g_channel.isoChunkBuffer.size()));
        g_channel.isoRxQueue.clear();
        g_channel.isoChunkBuffer.clear();
        g_channel.isoChunkExpected = 0;
        g_channel.isoChunkReceived = 0;
        g_channel.rxQueue.clear();
        g_usb.clearPending();
        picoj_packet_t response{};
        if (!g_usb.transact(PICOJ_CMD_CLEAR_RX, nullptr, 0, response, kDefaultTimeoutMs)) {
            setLastError(g_usb.lastError());
            logEvent("PassThruIoctl", "CLEAR_RX_BUFFER firmware clear failed Error=%s", g_usb.lastError().c_str());
            if (!g_usb.isOpen()) {
                g_channel = ChannelState{};
                return ERR_DEVICE_NOT_CONNECTED;
            }
            return g_usb.lastErrorWasTimeout() ? ERR_TIMEOUT : ERR_FAILED;
        }
        logEvent("PassThruIoctl", "CLEAR_RX_BUFFER complete ChannelID=%lu", ChannelID);
        return STATUS_NOERROR;
    }
    if (IoctlID == CLEAR_TX_BUFFER || IoctlID == CLEAR_PERIODIC_MSGS) {
        logEvent("PassThruIoctl", "%s ChannelID=%lu", ioctlName(IoctlID), ChannelID);
        if (IoctlID == CLEAR_PERIODIC_MSGS) {
            for (auto& periodic : g_channel.periodic) {
                periodic = PeriodicMessage{};
            }
            g_periodicCv.notify_all();
        }
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
