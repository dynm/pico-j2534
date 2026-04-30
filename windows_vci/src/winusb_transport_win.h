#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "pico_j2534_protocol.h"

class WinUsbTransport {
public:
    WinUsbTransport();
    ~WinUsbTransport();

    bool open();
    void close();
    bool isOpen() const;

    bool transact(uint8_t cmd, const void* outData, uint8_t outLen, picoj_packet_t& response, unsigned timeoutMs);
    bool readPacket(picoj_packet_t& packet, unsigned timeoutMs);
    const std::string& lastError() const { return lastError_; }

private:
    void closeUnlocked();
    bool writePacketUnlocked(const picoj_packet_t& packet, unsigned timeoutMs);
    bool readPacketUnlocked(picoj_packet_t& packet, unsigned timeoutMs);
    void setError(const std::string& error);

    void* deviceHandle_;
    void* winusbHandle_;
    uint8_t bulkIn_;
    uint8_t bulkOut_;
    uint8_t seq_;
    mutable std::mutex ioMutex_;
    std::string lastError_;
};
