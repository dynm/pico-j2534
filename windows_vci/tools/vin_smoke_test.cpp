#include "j2534.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr unsigned long kObdRequestId = 0x7DF;
constexpr unsigned long kObdResponseBase = 0x7E8;
constexpr unsigned long kObdPhysicalRequestBase = 0x7E0;
constexpr unsigned long kObdExactMask = 0x7FF;
constexpr unsigned long kCanBaudrate = 500000;
constexpr unsigned long kReadTimeoutMs = 250;
constexpr unsigned long kFunctionalReadTimeMs = 5000;
constexpr unsigned long kPhysicalReadTimeMs = 1500;

using PassThruOpenFn = long(WINAPI*)(void*, unsigned long*);
using PassThruCloseFn = long(WINAPI*)(unsigned long);
using PassThruConnectFn = long(WINAPI*)(unsigned long, unsigned long, unsigned long, unsigned long, unsigned long*);
using PassThruDisconnectFn = long(WINAPI*)(unsigned long);
using PassThruReadMsgsFn = long(WINAPI*)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
using PassThruWriteMsgsFn = long(WINAPI*)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
using PassThruStartMsgFilterFn = long(WINAPI*)(unsigned long, unsigned long, PASSTHRU_MSG*, PASSTHRU_MSG*, PASSTHRU_MSG*, unsigned long*);
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
    PassThruStartMsgFilterFn PassThruStartMsgFilter = nullptr;
    PassThruIoctlFn PassThruIoctl = nullptr;
    PassThruGetLastErrorFn PassThruGetLastError = nullptr;
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
    const DWORD size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
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
    ok &= loadProc(api.module, "PassThruStartMsgFilter", api.PassThruStartMsgFilter);
    ok &= loadProc(api.module, "PassThruIoctl", api.PassThruIoctl);
    ok &= loadProc(api.module, "PassThruGetLastError", api.PassThruGetLastError);
    return ok;
}

std::string lastJ2534Error(const J2534Api& api) {
    if (!api.PassThruGetLastError) {
        return "PassThruGetLastError unavailable";
    }

    std::array<char, 80> buffer{};
    if (api.PassThruGetLastError(buffer.data()) != STATUS_NOERROR) {
        return "PassThruGetLastError failed";
    }
    return buffer.data();
}

void setPayload(PASSTHRU_MSG& msg, unsigned long canId, const std::vector<unsigned char>& payload) {
    std::memset(&msg, 0, sizeof(msg));
    msg.ProtocolID = ISO15765;
    msg.TxFlags = ISO15765_FRAME_PAD;
    msg.DataSize = static_cast<unsigned long>(4 + payload.size());
    msg.ExtraDataIndex = msg.DataSize;
    writeCanId(msg, canId);
    std::copy(payload.begin(), payload.end(), msg.Data + 4);
}

bool installVinFilter(const J2534Api& api, unsigned long channelId) {
    bool installed = false;
    for (unsigned long offset = 0; offset < 8; ++offset) {
        PASSTHRU_MSG mask{};
        PASSTHRU_MSG pattern{};
        PASSTHRU_MSG flow{};
        setPayload(mask, kObdExactMask, {});
        setPayload(pattern, kObdResponseBase + offset, {});
        setPayload(flow, kObdPhysicalRequestBase + offset, {});

        unsigned long filterId = 0;
        const long status = api.PassThruStartMsgFilter(channelId, FLOW_CONTROL_FILTER, &mask, &pattern, &flow, &filterId);
        if (status != STATUS_NOERROR) {
            std::cerr << "Warning: PassThruStartMsgFilter failed for 0x" << std::hex << (kObdResponseBase + offset)
                      << " -> 0x" << (kObdPhysicalRequestBase + offset) << " with 0x" << status << std::dec
                      << ": " << lastJ2534Error(api) << "\n";
            continue;
        }
        installed = true;
    }
    return installed;
}

std::string parseVinPayload(const PASSTHRU_MSG& msg) {
    const unsigned long canId = readCanId(msg);
    if (canId < kObdResponseBase || canId > 0x7EF || msg.DataSize <= 6) {
        return {};
    }

    const unsigned char* payload = msg.Data + 4;
    const size_t payloadSize = msg.DataSize - 4;
    if (payloadSize < 3 || payload[0] != 0x49 || payload[1] != 0x02) {
        return {};
    }

    size_t vinOffset = 2;
    if (payloadSize >= 4 && payload[2] == 0x01) {
        vinOffset = 3;
    }

    std::string vin;
    for (size_t i = vinOffset; i < payloadSize && vin.size() < 17; ++i) {
        const unsigned char ch = payload[i];
        if (std::isalnum(ch)) {
            vin.push_back(static_cast<char>(ch));
        }
    }
    return vin.size() == 17 ? vin : std::string{};
}

void clearRxBuffer(const J2534Api& api, unsigned long channelId) {
    const long status = api.PassThruIoctl(channelId, CLEAR_RX_BUFFER, nullptr, nullptr);
    if (status != STATUS_NOERROR) {
        std::cerr << "Warning: CLEAR_RX_BUFFER failed with 0x" << std::hex << status << std::dec
                  << ": " << lastJ2534Error(api) << "\n";
    }
}

std::string requestVin(const J2534Api& api, unsigned long channelId, unsigned long requestId, unsigned long totalReadTimeMs) {
    clearRxBuffer(api, channelId);

    PASSTHRU_MSG request{};
    setPayload(request, requestId, {0x09, 0x02});
    unsigned long requestCount = 1;
    long status = api.PassThruWriteMsgs(channelId, &request, &requestCount, 1000);
    if (status != STATUS_NOERROR || requestCount != 1) {
        std::cerr << "PassThruWriteMsgs failed for request ID 0x" << std::hex << requestId << " with 0x" << status << std::dec
                  << ": " << lastJ2534Error(api) << "\n";
        return {};
    }

    const DWORD start = GetTickCount();
    while (GetTickCount() - start < totalReadTimeMs) {
        PASSTHRU_MSG response{};
        unsigned long responseCount = 1;
        status = api.PassThruReadMsgs(channelId, &response, &responseCount, kReadTimeoutMs);
        if (status == STATUS_NOERROR && responseCount > 0) {
            const std::string vin = parseVinPayload(response);
            if (!vin.empty()) {
                return vin;
            }
        } else if (status != ERR_BUFFER_EMPTY) {
            std::cerr << "PassThruReadMsgs failed with 0x" << std::hex << status << std::dec << ": " << lastJ2534Error(api) << "\n";
            return {};
        }
    }

    return {};
}

bool readVin(const J2534Api& api) {
    unsigned long deviceId = 0;
    long status = api.PassThruOpen(nullptr, &deviceId);
    if (status != STATUS_NOERROR) {
        std::cerr << "PassThruOpen failed with 0x" << std::hex << status << std::dec << ": " << lastJ2534Error(api) << "\n";
        return false;
    }

    unsigned long channelId = 0;
    status = api.PassThruConnect(deviceId, ISO15765, 0, kCanBaudrate, &channelId);
    if (status != STATUS_NOERROR) {
        std::cerr << "PassThruConnect failed with 0x" << std::hex << status << std::dec << ": " << lastJ2534Error(api) << "\n";
        api.PassThruClose(deviceId);
        return false;
    }

    installVinFilter(api, channelId);

    std::string vin = requestVin(api, channelId, kObdRequestId, kFunctionalReadTimeMs);
    if (vin.empty()) {
        for (unsigned long offset = 0; offset < 8 && vin.empty(); ++offset) {
            vin = requestVin(api, channelId, kObdPhysicalRequestBase + offset, kPhysicalReadTimeMs);
        }
    }
    if (!vin.empty()) {
        std::cout << "VIN: " << vin << "\n";
        api.PassThruDisconnect(channelId);
        api.PassThruClose(deviceId);
        return true;
    }

    std::cerr << "Timed out waiting for a VIN response\n";
    api.PassThruDisconnect(channelId);
    api.PassThruClose(deviceId);
    return false;
}

void printUsage(const char* exeName) {
    std::cerr
        << "Usage: " << exeName << " [--check-exports] [pico_j2534.dll]\n"
        << "\n"
        << "Without --check-exports, opens the DLL through J2534 and sends OBD-II service 09 PID 02\n"
        << "over ISO15765 at 500 kbit/s to read and print the vehicle VIN.\n";
}

} // namespace

int main(int argc, char** argv) {
    bool checkExportsOnly = false;
    const char* dllPath = "pico_j2534.dll";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--check-exports") {
            checkExportsOnly = true;
            continue;
        }
        dllPath = argv[i];
    }

    J2534Api api{};
    if (!loadApi(dllPath, api)) {
        if (api.module) {
            FreeLibrary(api.module);
        }
        return 2;
    }

    if (checkExportsOnly) {
        std::cout << "Loaded J2534 DLL and resolved required exports: " << dllPath << "\n";
        FreeLibrary(api.module);
        return 0;
    }

    const bool ok = readVin(api);
    FreeLibrary(api.module);
    return ok ? 0 : 1;
}
