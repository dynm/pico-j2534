#include "j2534.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr unsigned long kDefaultReqId = 0x607;
constexpr unsigned long kDefaultRespId = 0x60F;
constexpr unsigned long kCanBaudrate = 500000;
constexpr unsigned long kReadSliceMs = 50;
constexpr unsigned long kStepTimeoutMs = 3500;
constexpr unsigned long kObserveMs = 20000;

using PassThruOpenFn = long(WINAPI*)(void*, unsigned long*);
using PassThruCloseFn = long(WINAPI*)(unsigned long);
using PassThruConnectFn = long(WINAPI*)(unsigned long, unsigned long, unsigned long, unsigned long, unsigned long*);
using PassThruDisconnectFn = long(WINAPI*)(unsigned long);
using PassThruReadMsgsFn = long(WINAPI*)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
using PassThruWriteMsgsFn = long(WINAPI*)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
using PassThruIoctlFn = long(WINAPI*)(unsigned long, unsigned long, void*, void*);
using PassThruGetLastErrorFn = long(WINAPI*)(char*);

struct J2534Api {
    HMODULE module = nullptr;
    PassThruOpenFn PassThruOpen = nullptr;
    PassThruCloseFn PassThruClose = nullptr;
    PassThruConnectFn PassThruConnect = nullptr;
    PassThruDisconnectFn PassThruDisconnect = nullptr;
    PassThruReadMsgsFn PassThruReadMsgs = nullptr;
    PassThruWriteMsgsFn PassThruWriteMsgs = nullptr;
    PassThruIoctlFn PassThruIoctl = nullptr;
    PassThruGetLastErrorFn PassThruGetLastError = nullptr;
};

struct IsoTpCapture {
    bool active = false;
    unsigned long canId = 0;
    unsigned expectedLen = 0;
    unsigned nextSn = 1;
    std::vector<unsigned char> payload;
};

void writeCanId(PASSTHRU_MSG& msg, unsigned long canId) {
    msg.Data[0] = static_cast<unsigned char>((canId >> 24) & 0xFF);
    msg.Data[1] = static_cast<unsigned char>((canId >> 16) & 0xFF);
    msg.Data[2] = static_cast<unsigned char>((canId >> 8) & 0xFF);
    msg.Data[3] = static_cast<unsigned char>(canId & 0xFF);
}

unsigned long readCanId(const PASSTHRU_MSG& msg) {
    if (msg.DataSize < 4) {
        return 0;
    }
    return (static_cast<unsigned long>(msg.Data[0]) << 24) |
           (static_cast<unsigned long>(msg.Data[1]) << 16) |
           (static_cast<unsigned long>(msg.Data[2]) << 8) |
           static_cast<unsigned long>(msg.Data[3]);
}

std::string windowsError(DWORD error) {
    char* text = nullptr;
    const DWORD size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                          FORMAT_MESSAGE_IGNORE_INSERTS,
                                      nullptr,
                                      error,
                                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                                      reinterpret_cast<LPSTR>(&text),
                                      0,
                                      nullptr);
    std::string result = "Windows error " + std::to_string(error);
    if (size != 0 && text) {
        result += ": ";
        result += text;
        while (!result.empty() && (result.back() == '\r' || result.back() == '\n')) {
            result.pop_back();
        }
    }
    if (text) {
        LocalFree(text);
    }
    return result;
}

template <typename Fn>
bool loadProc(HMODULE module, const char* name, Fn& out) {
    out = reinterpret_cast<Fn>(GetProcAddress(module, name));
    if (!out) {
        std::cerr << "Missing export: " << name << "\n";
        return false;
    }
    return true;
}

bool loadApi(const char* dllPath, J2534Api& api) {
    api.module = LoadLibraryA(dllPath);
    if (!api.module) {
        std::cerr << "LoadLibrary failed for '" << dllPath << "': " << windowsError(GetLastError()) << "\n";
        return false;
    }
    bool ok = true;
    ok &= loadProc(api.module, "PassThruOpen", api.PassThruOpen);
    ok &= loadProc(api.module, "PassThruClose", api.PassThruClose);
    ok &= loadProc(api.module, "PassThruConnect", api.PassThruConnect);
    ok &= loadProc(api.module, "PassThruDisconnect", api.PassThruDisconnect);
    ok &= loadProc(api.module, "PassThruReadMsgs", api.PassThruReadMsgs);
    ok &= loadProc(api.module, "PassThruWriteMsgs", api.PassThruWriteMsgs);
    ok &= loadProc(api.module, "PassThruIoctl", api.PassThruIoctl);
    ok &= loadProc(api.module, "PassThruGetLastError", api.PassThruGetLastError);
    return ok;
}

std::string lastJ2534Error(const J2534Api& api) {
    std::array<char, 80> buffer{};
    if (!api.PassThruGetLastError || api.PassThruGetLastError(buffer.data()) != STATUS_NOERROR) {
        return "PassThruGetLastError unavailable";
    }
    return buffer.data();
}

unsigned long parseCanId(const std::string& text) {
    unsigned long value = 0;
    std::stringstream stream(text);
    stream >> std::hex >> value;
    if (!stream || value > 0x1FFFFFFFul) {
        throw std::runtime_error("Invalid CAN ID: " + text);
    }
    return value;
}

void makeCanMsg(PASSTHRU_MSG& msg, unsigned long canId, const std::array<unsigned char, 8>& data) {
    std::memset(&msg, 0, sizeof(msg));
    msg.ProtocolID = CAN;
    msg.DataSize = 12;
    msg.ExtraDataIndex = msg.DataSize;
    writeCanId(msg, canId);
    std::copy(data.begin(), data.end(), msg.Data + 4);
}

void printCan(const char* prefix, const PASSTHRU_MSG& msg) {
    const unsigned long canId = readCanId(msg);
    std::cout << std::setw(8) << GetTickCount() << ' ' << prefix << " STD " << std::hex << std::setw(8)
              << std::setfill('0') << canId << std::dec << std::setfill(' ') << " [" << (msg.DataSize - 4) << "]";
    for (unsigned long i = 4; i < msg.DataSize; ++i) {
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(msg.Data[i]);
    }
    std::cout << std::dec << std::setfill(' ') << "\n";
}

bool writeCan(const J2534Api& api, unsigned long channelId, unsigned long canId, const std::array<unsigned char, 8>& data) {
    PASSTHRU_MSG msg{};
    makeCanMsg(msg, canId, data);
    printCan("TX", msg);
    unsigned long count = 1;
    const long status = api.PassThruWriteMsgs(channelId, &msg, &count, 1000);
    if (status != STATUS_NOERROR || count != 1) {
        std::cerr << "PassThruWriteMsgs failed with 0x" << std::hex << status << std::dec << ": "
                  << lastJ2534Error(api) << "\n";
        return false;
    }
    return true;
}

std::string parseVin(const std::vector<unsigned char>& payload) {
    if (payload.size() < 20 || payload[0] != 0x62 || payload[1] != 0xF1 || payload[2] != 0x90) {
        return {};
    }
    std::string vin;
    for (size_t i = 3; i < payload.size() && vin.size() < 17; ++i) {
        if (std::isalnum(payload[i])) {
            vin.push_back(static_cast<char>(payload[i]));
        }
    }
    return vin.size() == 17 ? vin : std::string{};
}

std::string consumeIsoTp(IsoTpCapture& capture, const PASSTHRU_MSG& msg, unsigned long responseId) {
    if (readCanId(msg) != responseId || msg.DataSize < 6) {
        return {};
    }

    const unsigned char* data = msg.Data + 4;
    const unsigned long len = msg.DataSize - 4;
    const unsigned pciType = data[0] >> 4;
    if (pciType == 0) {
        const unsigned payloadLen = data[0] & 0x0F;
        if (payloadLen == 0 || payloadLen > 7 || len < payloadLen + 1) {
            return {};
        }
        std::vector<unsigned char> payload(data + 1, data + 1 + payloadLen);
        if (payload.size() >= 3 && payload[0] == 0x7F) {
            std::cout << "UDS negative response: service=0x" << std::hex << static_cast<unsigned>(payload[1])
                      << " nrc=0x" << static_cast<unsigned>(payload[2]) << std::dec << "\n";
        }
        return parseVin(payload);
    }
    if (pciType == 1 && len == 8) {
        capture.active = true;
        capture.canId = responseId;
        capture.expectedLen = ((data[0] & 0x0F) << 8) | data[1];
        capture.nextSn = 1;
        capture.payload.assign(data + 2, data + 8);
        capture.payload.resize(std::min<unsigned>(capture.payload.size(), capture.expectedLen));
        std::cout << "ISO-TP first frame total=" << capture.expectedLen
                  << " waiting for consecutive frames. DLL should send FC now.\n";
        return {};
    }
    if (pciType == 2 && capture.active) {
        const unsigned sn = data[0] & 0x0F;
        if (sn != capture.nextSn) {
            std::cout << "ISO-TP sequence mismatch: expected " << capture.nextSn << " got " << sn << "\n";
            capture.active = false;
            return {};
        }
        capture.nextSn = (capture.nextSn + 1) & 0x0F;
        const unsigned remaining = capture.expectedLen - static_cast<unsigned>(capture.payload.size());
        const unsigned copyLen = std::min<unsigned>(remaining, len - 1);
        capture.payload.insert(capture.payload.end(), data + 1, data + 1 + copyLen);
        if (capture.payload.size() >= capture.expectedLen) {
            capture.active = false;
            return parseVin(capture.payload);
        }
    }
    return {};
}

std::string readFor(const J2534Api& api,
                    unsigned long channelId,
                    unsigned long responseId,
                    DWORD durationMs,
                    DWORD& lastRxMs,
                    unsigned long& totalRx) {
    const DWORD start = GetTickCount();
    DWORD lastNoticeMs = start;
    IsoTpCapture capture{};
    while (GetTickCount() - start < durationMs) {
        PASSTHRU_MSG msg{};
        unsigned long count = 1;
        const long status = api.PassThruReadMsgs(channelId, &msg, &count, kReadSliceMs);
        if (status == STATUS_NOERROR && count > 0) {
            ++totalRx;
            lastRxMs = GetTickCount();
            printCan("RX", msg);
            const std::string vin = consumeIsoTp(capture, msg, responseId);
            if (!vin.empty()) {
                return vin;
            }
            continue;
        }
        if (status != ERR_BUFFER_EMPTY && status != ERR_TIMEOUT) {
            std::cerr << "PassThruReadMsgs failed with 0x" << std::hex << status << std::dec << ": "
                      << lastJ2534Error(api) << "\n";
            return {};
        }
        const DWORD now = GetTickCount();
        if (now - lastNoticeMs >= 1000) {
            std::cout << "OBS no RX for " << (now - lastRxMs) << " ms, total_rx=" << totalRx << "\n";
            lastNoticeMs = now;
        }
    }
    return {};
}

void waitBeforeExit() {
    std::cout << "\nPress Enter to close this window...";
    std::string line;
    std::getline(std::cin, line);
}

void printUsage(const char* exe) {
    std::cerr << "Usage: " << exe << " [pico_j2534.dll] [--req 607] [--resp 60f]\n";
}

} // namespace

int main(int argc, char** argv) {
    const char* dllPath = "pico_j2534.dll";
    unsigned long requestId = kDefaultReqId;
    unsigned long responseId = kDefaultRespId;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            }
            if (arg == "--req" && i + 1 < argc) {
                requestId = parseCanId(argv[++i]);
                continue;
            }
            if (arg == "--resp" && i + 1 < argc) {
                responseId = parseCanId(argv[++i]);
                continue;
            }
            dllPath = argv[i];
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        waitBeforeExit();
        return 2;
    }

    J2534Api api{};
    if (!loadApi(dllPath, api)) {
        waitBeforeExit();
        return 2;
    }

    unsigned long deviceId = 0;
    long status = api.PassThruOpen(nullptr, &deviceId);
    if (status != STATUS_NOERROR) {
        std::cerr << "PassThruOpen failed with 0x" << std::hex << status << std::dec << ": " << lastJ2534Error(api)
                  << "\n";
        waitBeforeExit();
        return 1;
    }

    unsigned long channelId = 0;
    status = api.PassThruConnect(deviceId, CAN, 0, kCanBaudrate, &channelId);
    if (status != STATUS_NOERROR) {
        std::cerr << "PassThruConnect(CAN) failed with 0x" << std::hex << status << std::dec << ": "
                  << lastJ2534Error(api) << "\n";
        api.PassThruClose(deviceId);
        waitBeforeExit();
        return 1;
    }

    std::cout << "DTS raw-CAN monitor using " << dllPath << "\n";
    std::cout << "req=0x" << std::hex << requestId << " resp=0x" << responseId << std::dec
              << " bitrate=" << kCanBaudrate << "\n";
    (void)api.PassThruIoctl(channelId, CLEAR_RX_BUFFER, nullptr, nullptr);

    DWORD lastRxMs = GetTickCount();
    unsigned long totalRx = 0;

    writeCan(api, channelId, requestId, {0x02, 0x10, 0x03, 0x55, 0x55, 0x55, 0x55, 0x55});
    readFor(api, channelId, responseId, kStepTimeoutMs, lastRxMs, totalRx);

    writeCan(api, channelId, requestId, {0x03, 0x22, 0xF1, 0xA0, 0x55, 0x55, 0x55, 0x55});
    readFor(api, channelId, responseId, kStepTimeoutMs, lastRxMs, totalRx);

    writeCan(api, channelId, requestId, {0x03, 0x22, 0xF1, 0xA0, 0x55, 0x55, 0x55, 0x55});
    readFor(api, channelId, responseId, kStepTimeoutMs, lastRxMs, totalRx);

    std::cout << "Requesting VIN with 22 F1 90...\n";
    writeCan(api, channelId, requestId, {0x03, 0x22, 0xF1, 0x90, 0x55, 0x55, 0x55, 0x55});
    const std::string vin = readFor(api, channelId, responseId, kStepTimeoutMs, lastRxMs, totalRx);
    if (!vin.empty()) {
        std::cout << "VIN: " << vin << "\n";
    } else {
        std::cout << "VIN not decoded during the request window.\n";
    }

    std::cout << "Observing CAN stream for " << (kObserveMs / 1000) << " seconds...\n";
    readFor(api, channelId, responseId, kObserveMs, lastRxMs, totalRx);
    std::cout << "Done. total_rx=" << totalRx << ", last_rx_age_ms=" << (GetTickCount() - lastRxMs) << "\n";

    api.PassThruDisconnect(channelId);
    api.PassThruClose(deviceId);
    if (api.module) {
        FreeLibrary(api.module);
    }
    waitBeforeExit();
    return 0;
}
