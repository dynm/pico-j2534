#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

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
    void clearPending();
    void restorePending(const std::vector<picoj_packet_t>& packets);
    const std::string& lastError() const { return lastError_; }
    bool lastErrorWasTimeout() const { return lastErrorWasTimeout_; }

private:
    void closeUnlocked();
    void flushInputUnlocked();
    bool writePacketUnlocked(const picoj_packet_t& packet, unsigned timeoutMs);
    bool readPacketUnlocked(picoj_packet_t& packet, unsigned timeoutMs);
    void setError(const std::string& error);

    void* deviceHandle_;
    void* winusbHandle_;
    uint8_t bulkIn_;
    uint8_t bulkOut_;
    uint8_t seq_;
    mutable std::mutex ioMutex_;
    std::deque<picoj_packet_t> pendingPackets_;
    std::string lastError_;
    bool lastErrorWasTimeout_;
};
