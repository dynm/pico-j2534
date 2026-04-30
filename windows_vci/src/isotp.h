#pragma once

#include <cstdint>
#include <vector>

#include "pico_j2534_protocol.h"

struct IsoTpFrame {
    uint32_t canId = 0;
    bool extended = false;
    std::vector<uint8_t> payload;
};

enum class IsoTpFlowStatus {
    Continue,
    Wait,
    Overflow,
};

struct IsoTpFlowControl {
    IsoTpFlowStatus status = IsoTpFlowStatus::Continue;
    uint8_t blockSize = 0;
    uint8_t stMin = 0;
};

class IsoTpReassembler {
public:
    bool accept(const picoj_can_frame_t& frame, IsoTpFrame& out);
    void reset();

private:
    uint32_t canId_ = 0;
    bool extended_ = false;
    uint16_t expected_ = 0;
    uint8_t nextSeq_ = 1;
    std::vector<uint8_t> buffer_;
};

bool isotpParseFlowControl(const picoj_can_frame_t& frame, IsoTpFlowControl& out);
unsigned isotpStMinDelayMs(uint8_t stMin);
std::vector<picoj_can_frame_t> isotpSegment(uint32_t canId, bool extended, const uint8_t* data, size_t size);
