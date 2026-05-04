#include "pico_j2534_protocol.h"

#include <libusb.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kVendorInterface = 2;
constexpr uint8_t kBulkOut = 0x03;
constexpr uint8_t kBulkIn = 0x83;
constexpr unsigned kDefaultTimeoutMs = 1000;
constexpr unsigned kIsoTpResponseTimeoutMs = 3000;

class PicoUsb {
public:
    PicoUsb() {
        if (libusb_init(&context_) != 0) {
            throw std::runtime_error("libusb_init failed");
        }
    }

    ~PicoUsb() {
        close();
        if (context_) {
            libusb_exit(context_);
        }
    }

    PicoUsb(const PicoUsb&) = delete;
    PicoUsb& operator=(const PicoUsb&) = delete;

    void open() {
        handle_ = libusb_open_device_with_vid_pid(context_, PICOJ_USB_VID, PICOJ_USB_PID);
        if (!handle_) {
            throw std::runtime_error("Pico J2534 USB device not found");
        }

        const int claim = libusb_claim_interface(handle_, kVendorInterface);
        if (claim != 0) {
            close();
            throw std::runtime_error("libusb_claim_interface failed: " + std::string(libusb_error_name(claim)));
        }
        claimed_ = true;
    }

    void close() {
        if (handle_) {
            if (claimed_) {
                libusb_release_interface(handle_, kVendorInterface);
                claimed_ = false;
            }
            libusb_close(handle_);
            handle_ = nullptr;
        }
    }

    picoj_packet_t transact(uint8_t cmd, const void* payload, uint8_t len, unsigned timeoutMs = kDefaultTimeoutMs) {
        if (len > PICOJ_PACKET_PAYLOAD_SIZE) {
            throw std::runtime_error("USB payload too large");
        }

        picoj_packet_t request{};
        request.magic = PICOJ_PACKET_MAGIC;
        request.seq = seq_;
        seq_ = static_cast<uint8_t>(seq_ == 0xFF ? 1 : seq_ + 1);
        request.cmd = cmd;
        request.len = len;
        if (payload && len) {
            std::memcpy(request.payload, payload, len);
        }

        writePacket(request, timeoutMs);
        for (;;) {
            picoj_packet_t response = readPacket(timeoutMs);
            if (response.cmd == PICOJ_CMD_CAN_RX || response.cmd == PICOJ_CMD_ISOTP_RX) {
                pending_.push_back(response);
                continue;
            }
            if (response.seq == request.seq) {
                return response;
            }
        }
    }

    picoj_packet_t readAny(unsigned timeoutMs) {
        if (!pending_.empty()) {
            picoj_packet_t packet = pending_.front();
            pending_.erase(pending_.begin());
            return packet;
        }
        return readPacket(timeoutMs);
    }

    void clearPending() {
        pending_.clear();
    }

private:
    void writePacket(const picoj_packet_t& packet, unsigned timeoutMs) {
        int transferred = 0;
        const int status = libusb_bulk_transfer(handle_,
                                                kBulkOut,
                                                const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(&packet)),
                                                sizeof(packet),
                                                &transferred,
                                                timeoutMs);
        if (status != 0 || transferred != static_cast<int>(sizeof(packet))) {
            throw std::runtime_error("USB write failed: " + std::string(libusb_error_name(status)));
        }
    }

    picoj_packet_t readPacket(unsigned timeoutMs) {
        picoj_packet_t packet{};
        auto* out = reinterpret_cast<unsigned char*>(&packet);
        int total = 0;
        int emptyReads = 0;
        while (total < static_cast<int>(sizeof(packet))) {
            int transferred = 0;
            const int status = libusb_bulk_transfer(handle_,
                                                    kBulkIn,
                                                    out + total,
                                                    static_cast<int>(sizeof(packet)) - total,
                                                    &transferred,
                                                    timeoutMs);
            if (status != 0) {
                throw std::runtime_error("USB read failed: " + std::string(libusb_error_name(status)));
            }
            if (transferred == 0 && ++emptyReads > 20) {
                throw std::runtime_error("USB read returned no data");
            }
            if (transferred == 0) {
                continue;
            }
            total += transferred;
        }
        if (packet.magic != PICOJ_PACKET_MAGIC || packet.len > PICOJ_PACKET_PAYLOAD_SIZE) {
            throw std::runtime_error("Invalid Pico USB packet");
        }
        return packet;
    }

    libusb_context* context_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    bool claimed_ = false;
    uint8_t seq_ = 1;
    std::vector<picoj_packet_t> pending_;
};

uint32_t parseCanId(const std::string& text) {
    uint32_t value = 0;
    std::stringstream stream(text);
    if (text.rfind("0x", 0) == 0 || text.rfind("0X", 0) == 0) {
        stream >> std::hex >> value;
    } else {
        stream >> std::hex >> value;
    }
    if (!stream || value > 0x1FFFFFFF) {
        throw std::runtime_error("Invalid CAN ID: " + text);
    }
    return value;
}

void requireStatusOk(const picoj_packet_t& packet, const char* what) {
    if (packet.cmd != PICOJ_CMD_STATUS || packet.len < sizeof(picoj_status_t)) {
        throw std::runtime_error(std::string(what) + " returned unexpected USB response");
    }
    picoj_status_t status{};
    std::memcpy(&status, packet.payload, sizeof(status));
    if (status.code != 0) {
        std::ostringstream out;
        out << what << " failed, firmware status=" << status.code << " detail=0x" << std::hex << status.detail;
        throw std::runtime_error(out.str());
    }
}

void printCanFrame(const char* prefix, const picoj_can_frame_t& frame) {
    std::cout << prefix << " CAN 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << " ["
              << std::dec << static_cast<unsigned>(frame.dlc) << "]";
    for (uint8_t i = 0; i < frame.dlc; ++i) {
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(frame.data[i]);
    }
    std::cout << std::dec << "\n";
}

void sendRawCan(PicoUsb& usb, uint32_t canId, const std::array<uint8_t, 8>& data, const char* label) {
    picoj_can_frame_t frame{};
    frame.can_id = canId;
    frame.dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    printCanFrame(label, frame);
    requireStatusOk(usb.transact(PICOJ_CMD_CAN_TX, &frame, sizeof(frame)), "CAN_TX");
}

std::vector<uint8_t> readRawCanPayload(PicoUsb& usb, uint32_t requestId, uint32_t responseId, bool hostFlowControl) {
    std::vector<uint8_t> payload;
    uint16_t expectedLen = 0;
    uint8_t nextSn = 1;

    while (true) {
        picoj_packet_t packet = usb.readAny(kIsoTpResponseTimeoutMs);
        if (packet.cmd == PICOJ_CMD_ISOTP_RX && packet.len >= sizeof(picoj_isotp_chunk_t)) {
            picoj_isotp_chunk_t chunk{};
            std::memcpy(&chunk, packet.payload, sizeof(chunk));
            std::cout << "RX ISOTP_CHUNK can=0x" << std::hex << chunk.can_id << " total=" << std::dec << chunk.total_len
                      << " offset=" << chunk.offset << " chunk=" << static_cast<unsigned>(chunk.chunk_len) << "\n";
            continue;
        }
        if (packet.cmd != PICOJ_CMD_CAN_RX || packet.len < sizeof(picoj_can_frame_t)) {
            std::cout << "RX ignored cmd=0x" << std::hex << static_cast<unsigned>(packet.cmd) << std::dec << "\n";
            continue;
        }

        picoj_can_frame_t frame{};
        std::memcpy(&frame, packet.payload, sizeof(frame));
        if (frame.can_id == requestId) {
            printCanFrame("RX", frame);
            continue;
        }
        if (frame.can_id != responseId || frame.dlc < 2) {
            continue;
        }
        printCanFrame("RX", frame);

        const uint8_t pciType = frame.data[0] >> 4;
        if (pciType == 0) {
            const uint8_t len = frame.data[0] & 0x0F;
            if (len == 0 || len > 7 || frame.dlc < static_cast<uint8_t>(len + 1)) {
                throw std::runtime_error("Invalid raw ISO-TP single frame");
            }
            return std::vector<uint8_t>(frame.data + 1, frame.data + 1 + len);
        }
        if (pciType == 1) {
            expectedLen = static_cast<uint16_t>(((frame.data[0] & 0x0F) << 8) | frame.data[1]);
            payload.assign(frame.data + 2, frame.data + 8);
            payload.resize(std::min<size_t>(payload.size(), expectedLen));
            nextSn = 1;
            if (hostFlowControl) {
                sendRawCan(usb, requestId, {0x30, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "TX FC");
            }
            continue;
        }
        if (pciType == 2 && expectedLen != 0) {
            const uint8_t sn = frame.data[0] & 0x0F;
            if (sn != nextSn) {
                throw std::runtime_error("Raw ISO-TP consecutive frame sequence mismatch");
            }
            nextSn = static_cast<uint8_t>((nextSn + 1u) & 0x0Fu);
            const size_t remaining = expectedLen - payload.size();
            const size_t copyLen = std::min<size_t>(remaining, frame.dlc - 1);
            payload.insert(payload.end(), frame.data + 1, frame.data + 1 + copyLen);
            if (payload.size() >= expectedLen) {
                return payload;
            }
        }
    }
}

std::vector<uint8_t> readRawCanMatching(PicoUsb& usb,
                                        uint32_t requestId,
                                        uint32_t responseId,
                                        uint8_t firstByte,
                                        bool hostFlowControl) {
    for (;;) {
        std::vector<uint8_t> payload = readRawCanPayload(usb, requestId, responseId, hostFlowControl);
        if (!payload.empty() && payload[0] == firstByte) {
            return payload;
        }
        std::cout << "RX payload ignored:";
        for (uint8_t byte : payload) {
            std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
        }
        std::cout << std::dec << "\n";
    }
}

std::vector<uint8_t> readIsoTpPayload(PicoUsb& usb, uint32_t expectedCanId) {
    std::vector<uint8_t> payload;
    uint16_t expectedLen = 0;
    uint16_t received = 0;

    while (true) {
        picoj_packet_t packet = usb.readAny(kIsoTpResponseTimeoutMs);
        if (packet.cmd == PICOJ_CMD_CAN_RX) {
            continue;
        }
        if (packet.cmd != PICOJ_CMD_ISOTP_RX || packet.len < sizeof(picoj_isotp_chunk_t)) {
            continue;
        }

        picoj_isotp_chunk_t chunk{};
        std::memcpy(&chunk, packet.payload, sizeof(chunk));
        if (chunk.can_id != expectedCanId) {
            continue;
        }
        if (chunk.total_len > PICOJ_ISOTP_MAX_PAYLOAD || chunk.chunk_len > PICOJ_ISOTP_CHUNK_DATA_SIZE ||
            static_cast<uint16_t>(chunk.offset + chunk.chunk_len) > chunk.total_len) {
            throw std::runtime_error("Invalid ISO-TP chunk from firmware");
        }

        if (chunk.offset == 0) {
            expectedLen = chunk.total_len;
            received = 0;
            payload.assign(expectedLen, 0);
        }
        if (chunk.total_len != expectedLen || chunk.offset != received) {
            throw std::runtime_error("Out-of-order ISO-TP chunk from firmware");
        }
        std::copy(chunk.data, chunk.data + chunk.chunk_len, payload.begin() + chunk.offset);
        received = static_cast<uint16_t>(received + chunk.chunk_len);
        if (received >= expectedLen) {
            return payload;
        }
    }
}

std::string parseVin(const std::vector<uint8_t>& payload) {
    if (payload.size() >= 3 && payload[0] == 0x7F) {
        std::ostringstream out;
        out << "UDS negative response: service=0x" << std::hex << static_cast<unsigned>(payload[1])
            << " nrc=0x" << static_cast<unsigned>(payload[2]);
        throw std::runtime_error(out.str());
    }
    if (payload.size() < 20 || payload[0] != 0x62 || payload[1] != 0xF1 || payload[2] != 0x90) {
        std::ostringstream out;
        out << "Unexpected UDS response:";
        for (uint8_t byte : payload) {
            out << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
        }
        throw std::runtime_error(out.str());
    }

    std::string vin;
    for (size_t i = 3; i < payload.size() && vin.size() < 17; ++i) {
        if (std::isalnum(payload[i])) {
            vin.push_back(static_cast<char>(payload[i]));
        }
    }
    if (vin.size() != 17) {
        throw std::runtime_error("VIN response did not contain 17 alphanumeric characters");
    }
    return vin;
}

void printUsage(const char* exe) {
    std::cerr << "Usage: " << exe << " [--req 607] [--resp 60f] [--bitrate 500000] [--bootloader] [--raw-can]\n";
}

} // namespace

int main(int argc, char** argv) {
    uint32_t requestId = 0x607;
    uint32_t responseId = 0x60F;
    uint32_t bitrate = 500000;
    bool rebootToBootloader = false;
    bool rawCan = false;
    bool hostFlowControl = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if ((arg == "--help" || arg == "-h")) {
                printUsage(argv[0]);
                return 0;
            }
            if (arg == "--bootloader") {
                rebootToBootloader = true;
                continue;
            }
            if (arg == "--raw-can") {
                rawCan = true;
                continue;
            }
            if (arg == "--host-fc") {
                hostFlowControl = true;
                continue;
            }
            if (arg == "--req" && i + 1 < argc) {
                requestId = parseCanId(argv[++i]);
                continue;
            }
            if (arg == "--resp" && i + 1 < argc) {
                responseId = parseCanId(argv[++i]);
                continue;
            }
            if (arg == "--bitrate" && i + 1 < argc) {
                bitrate = static_cast<uint32_t>(std::stoul(argv[++i]));
                continue;
            }
            throw std::runtime_error("Unknown or incomplete argument: " + arg);
        }

        PicoUsb usb;
        usb.open();

        picoj_packet_t hello = usb.transact(PICOJ_CMD_HELLO, nullptr, 0);
        if (hello.cmd != PICOJ_CMD_HELLO || hello.len < sizeof(picoj_hello_t)) {
            throw std::runtime_error("HELLO failed");
        }

        if (rebootToBootloader) {
            requireStatusOk(usb.transact(PICOJ_CMD_BOOTLOADER, nullptr, 0), "BOOTLOADER");
            std::cout << "Pico rebooting to USB bootloader\n";
            return 0;
        }

        picoj_bitrate_t bitrateRequest{bitrate};
        requireStatusOk(usb.transact(PICOJ_CMD_SET_BITRATE, &bitrateRequest, sizeof(bitrateRequest)), "SET_BITRATE");
        requireStatusOk(usb.transact(PICOJ_CMD_CLEAR_RX, nullptr, 0), "CLEAR_RX");

        picoj_isotp_config_t config{};
        config.tx_can_id = requestId;
        config.rx_can_id = responseId;
        config.flow_control_can_id = requestId;
        requireStatusOk(usb.transact(PICOJ_CMD_ISOTP_CONFIG, &config, sizeof(config)), "ISOTP_CONFIG");

        if (rawCan) {
            sendRawCan(usb, requestId, {0x02, 0x10, 0x03, 0x55, 0x55, 0x55, 0x55, 0x55}, "TX");
            const std::vector<uint8_t> session = readRawCanMatching(usb, requestId, responseId, 0x50, hostFlowControl);
            std::cout << "SESSION:";
            for (uint8_t byte : session) {
                std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
            }
            std::cout << std::dec << "\n";

            usb.clearPending();
            requireStatusOk(usb.transact(PICOJ_CMD_CLEAR_RX, nullptr, 0), "CLEAR_RX");
            usb.clearPending();
            sendRawCan(usb, requestId, {0x03, 0x22, 0xF1, 0x90, 0x55, 0x55, 0x55, 0x55}, "TX");
            const std::vector<uint8_t> response = readRawCanMatching(usb, requestId, responseId, 0x62, hostFlowControl);
            const std::string vin = parseVin(response);
            std::cout << "VIN: " << vin << "\n";
            return 0;
        }

        const std::array<uint8_t, 3> requestPayload{0x22, 0xF1, 0x90};
        picoj_isotp_chunk_t chunk{};
        chunk.can_id = requestId;
        chunk.total_len = requestPayload.size();
        chunk.offset = 0;
        chunk.chunk_len = requestPayload.size();
        std::copy(requestPayload.begin(), requestPayload.end(), chunk.data);
        requireStatusOk(usb.transact(PICOJ_CMD_ISOTP_TX, &chunk, sizeof(chunk)), "ISOTP_TX");

        const std::vector<uint8_t> response = readIsoTpPayload(usb, responseId);
        const std::string vin = parseVin(response);
        std::cout << "VIN: " << vin << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "uds_vin_test: " << error.what() << "\n";
        return 1;
    }
}
