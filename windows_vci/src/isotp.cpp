#include "isotp.h"

#include <algorithm>
#include <cstring>

bool IsoTpReassembler::accept(const picoj_can_frame_t& frame, IsoTpFrame& out) {
    if (frame.dlc == 0) {
        return false;
    }

    const uint8_t pci = frame.data[0] >> 4;
    if (pci == 0x0) {
        const uint8_t len = frame.data[0] & 0x0F;
        if (len > 7 || frame.dlc < static_cast<uint8_t>(len + 1)) {
            return false;
        }
        out.canId = frame.can_id;
        out.extended = (frame.flags & PICOJ_CAN_EXTENDED) != 0;
        out.payload.assign(frame.data + 1, frame.data + 1 + len);
        return true;
    }

    if (pci == 0x1) {
        if (frame.dlc < 2) {
            reset();
            return false;
        }
        expected_ = static_cast<uint16_t>(((frame.data[0] & 0x0F) << 8) | frame.data[1]);
        canId_ = frame.can_id;
        extended_ = (frame.flags & PICOJ_CAN_EXTENDED) != 0;
        nextSeq_ = 1;
        buffer_.clear();

        const size_t copied = std::min<size_t>(6, expected_);
        buffer_.insert(buffer_.end(), frame.data + 2, frame.data + 2 + copied);
        return false;
    }

    if (pci == 0x2) {
        const uint8_t seq = frame.data[0] & 0x0F;
        if (buffer_.empty() || frame.can_id != canId_ || seq != nextSeq_) {
            reset();
            return false;
        }

        nextSeq_ = static_cast<uint8_t>((nextSeq_ + 1) & 0x0F);
        const size_t remaining = expected_ > buffer_.size() ? expected_ - buffer_.size() : 0;
        const size_t copied = std::min<size_t>(7, remaining);
        buffer_.insert(buffer_.end(), frame.data + 1, frame.data + 1 + copied);

        if (buffer_.size() >= expected_) {
            out.canId = canId_;
            out.extended = extended_;
            out.payload = buffer_;
            reset();
            return true;
        }
    }

    return false;
}

void IsoTpReassembler::reset() {
    canId_ = 0;
    extended_ = false;
    expected_ = 0;
    nextSeq_ = 1;
    buffer_.clear();
}

bool isotpParseFlowControl(const picoj_can_frame_t& frame, IsoTpFlowControl& out) {
    if (frame.dlc < 3 || (frame.data[0] >> 4) != 0x3) {
        return false;
    }

    const uint8_t flowStatus = frame.data[0] & 0x0F;
    if (flowStatus == 0x0) {
        out.status = IsoTpFlowStatus::Continue;
    } else if (flowStatus == 0x1) {
        out.status = IsoTpFlowStatus::Wait;
    } else if (flowStatus == 0x2) {
        out.status = IsoTpFlowStatus::Overflow;
    } else {
        return false;
    }
    out.blockSize = frame.data[1];
    out.stMin = frame.data[2];
    return true;
}

unsigned isotpStMinDelayMs(uint8_t stMin) {
    if (stMin <= 0x7F) {
        return stMin;
    }
    if (stMin >= 0xF1 && stMin <= 0xF9) {
        return 1;
    }
    return 0;
}

std::vector<picoj_can_frame_t> isotpSegment(uint32_t canId, bool extended, const uint8_t* data, size_t size) {
    std::vector<picoj_can_frame_t> frames;
    const uint8_t flags = extended ? PICOJ_CAN_EXTENDED : 0;

    if (size <= 7) {
        picoj_can_frame_t frame{};
        frame.can_id = canId;
        frame.flags = flags;
        frame.dlc = 8;
        frame.data[0] = static_cast<uint8_t>(size);
        if (size) {
            std::memcpy(frame.data + 1, data, size);
        }
        frames.push_back(frame);
        return frames;
    }

    picoj_can_frame_t first{};
    first.can_id = canId;
    first.flags = flags;
    first.dlc = 8;
    first.data[0] = static_cast<uint8_t>(0x10 | ((size >> 8) & 0x0F));
    first.data[1] = static_cast<uint8_t>(size & 0xFF);
    std::memcpy(first.data + 2, data, 6);
    frames.push_back(first);

    size_t offset = 6;
    uint8_t seq = 1;
    while (offset < size) {
        picoj_can_frame_t cf{};
        cf.can_id = canId;
        cf.flags = flags;
        cf.dlc = 8;
        cf.data[0] = static_cast<uint8_t>(0x20 | (seq & 0x0F));
        const size_t chunk = std::min<size_t>(7, size - offset);
        std::memcpy(cf.data + 1, data + offset, chunk);
        frames.push_back(cf);
        offset += chunk;
        seq = static_cast<uint8_t>((seq + 1) & 0x0F);
    }

    return frames;
}
