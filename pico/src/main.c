#include <string.h>

#include "board_config.h"
#include "bsp/board.h"
#include "hardware/gpio.h"
#include "isotp.h"
#include "pico/bootrom.h"
#include "mcp2515.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico_j2534_protocol.h"
#include "tusb.h"

static bool can_ready = false;
static bool can_error = false;
static uint32_t led_activity_until_ms = 0;

#define PICOJ_USB_REPLY_TIMEOUT_MS 1000u
#define PICOJ_ISOTP_CAN_TX_ACCEPT_TIMEOUT_MS 20u
#define CAN_RX_QUEUE_CAPACITY 64u
#define CAN_RX_DRAIN_LIMIT 16u
#define ISOTP_DRAIN_LIMIT 16u

static picoj_can_frame_t can_rx_queue[CAN_RX_QUEUE_CAPACITY];
static uint8_t can_rx_head = 0;
static uint8_t can_rx_tail = 0;
static uint8_t can_rx_count = 0;
static uint32_t can_rx_overflows = 0;

static IsoTpLink isotp_link;
static uint8_t isotp_send_buffer[PICOJ_ISOTP_MAX_PAYLOAD];
static uint8_t isotp_recv_buffer[PICOJ_ISOTP_MAX_PAYLOAD];
static uint8_t isotp_rx_out_buffer[PICOJ_ISOTP_MAX_PAYLOAD];
static uint8_t isotp_tx_staging_buffer[PICOJ_ISOTP_MAX_PAYLOAD];
static bool isotp_configured = false;
static uint32_t isotp_tx_can_id = 0;
static uint32_t isotp_rx_can_id = 0;
static uint32_t isotp_flow_control_can_id = 0;
static uint8_t isotp_tx_flags = 0;
static uint8_t isotp_rx_flags = 0;
static uint8_t isotp_flow_control_flags = 0;
static uint16_t isotp_rx_out_len = 0;
static uint16_t isotp_rx_out_offset = 0;
static uint32_t isotp_rx_out_can_id = 0;
static uint8_t isotp_rx_out_flags = 0;
static uint16_t isotp_tx_expected_len = 0;
static uint16_t isotp_tx_received_len = 0;
static bool is_draining_can_rx = false;
static bool pending_isotp_flow_control = false;
static picoj_can_frame_t pending_isotp_flow_control_frame;

static void led_write(bool on) {
    gpio_put(PICOJ_STATUS_LED_PIN, PICOJ_STATUS_LED_ACTIVE_HIGH ? on : !on);
}

static void led_init(void) {
    gpio_init(PICOJ_STATUS_LED_PIN);
    gpio_set_dir(PICOJ_STATUS_LED_PIN, GPIO_OUT);
    led_write(false);
}

static uint32_t millis(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void led_mark_activity(void) {
    led_activity_until_ms = millis() + 120;
}

static void led_update(void) {
    uint32_t now = millis();
    bool on = true;

    if (!tud_mounted()) {
        on = ((now / 500) & 1u) != 0; // Waiting for USB host.
    } else if (can_error) {
        on = ((now / 100) & 1u) != 0; // CAN controller/configuration error.
    } else if (now < led_activity_until_ms) {
        on = ((now / 40) & 1u) != 0; // Short traffic flicker while ready.
    }

    led_write(on);
}

static bool send_packet(uint8_t seq, uint8_t cmd, const void* payload, uint8_t len, uint32_t timeout_ms) {
    if (!tud_vendor_mounted()) {
        return false;
    }

    const uint32_t start = millis();
    while (tud_vendor_write_available() < sizeof(picoj_packet_t)) {
        if (timeout_ms == 0 || millis() - start >= timeout_ms) {
            return false;
        }
        if (can_ready && !mcp2515_poll()) {
            can_ready = false;
            can_error = true;
            return false;
        }
        tud_task();
    }

    picoj_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.magic = PICOJ_PACKET_MAGIC;
    packet.seq = seq;
    packet.cmd = cmd;
    packet.len = len;
    if (payload && len) {
        memcpy(packet.payload, payload, len);
    }
    tud_vendor_write(&packet, sizeof(packet));
    tud_vendor_write_flush();
    led_mark_activity();
    return true;
}

static void send_status(uint8_t seq, int32_t code, uint32_t detail) {
    picoj_status_t status = {
        .code = code,
        .detail = detail,
    };
    send_packet(seq, PICOJ_CMD_STATUS, &status, sizeof(status), PICOJ_USB_REPLY_TIMEOUT_MS);
}

void isotp_user_debug(const char* message, ...) {
    (void)message;
}

uint32_t isotp_user_get_ms(void) {
    return millis();
}

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size) {
    if (!data || size == 0 || size > 8 || !can_ready) {
        return ISOTP_RET_ERROR;
    }

    picoj_can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = arbitration_id;
    frame.flags = isotp_tx_flags;
    frame.dlc = size;
    memcpy(frame.data, data, size);

    if (is_draining_can_rx && ((data[0] >> 4) == 3)) {
        pending_isotp_flow_control_frame = frame;
        pending_isotp_flow_control = true;
        return ISOTP_RET_OK;
    }

    const uint32_t start = millis();
    while (millis() - start < PICOJ_ISOTP_CAN_TX_ACCEPT_TIMEOUT_MS) {
        mcp2515_tx_result_t tx = mcp2515_send(&frame);
        if (tx == MCP2515_TX_OK) {
            led_mark_activity();
            return ISOTP_RET_OK;
        }
        if (tx == MCP2515_TX_ERROR) {
            return ISOTP_RET_ERROR;
        }
        if (!mcp2515_poll()) {
            can_ready = false;
            can_error = true;
            return ISOTP_RET_ERROR;
        }
        tud_task();
        sleep_us(100);
    }
    return ISOTP_RET_INPROGRESS;
}

static void send_pending_isotp_flow_control(void) {
    if (!pending_isotp_flow_control || !can_ready) {
        return;
    }

    const uint32_t start = millis();
    while (millis() - start < PICOJ_ISOTP_CAN_TX_ACCEPT_TIMEOUT_MS) {
        mcp2515_tx_result_t tx = mcp2515_send(&pending_isotp_flow_control_frame);
        if (tx == MCP2515_TX_OK) {
            pending_isotp_flow_control = false;
            led_mark_activity();
            return;
        }
        if (tx == MCP2515_TX_ERROR || !mcp2515_poll()) {
            can_ready = false;
            can_error = true;
            return;
        }
        tud_task();
        sleep_us(100);
    }
}

static void queue_isotp_flow_control_frame(uint8_t flow_status) {
    memset(&pending_isotp_flow_control_frame, 0, sizeof(pending_isotp_flow_control_frame));
    pending_isotp_flow_control_frame.can_id = isotp_flow_control_can_id ? isotp_flow_control_can_id : isotp_tx_can_id;
    pending_isotp_flow_control_frame.flags = isotp_flow_control_flags;
    pending_isotp_flow_control_frame.dlc = 8;
    pending_isotp_flow_control_frame.data[0] = (uint8_t)(0x30u | (flow_status & 0x0Fu));
    pending_isotp_flow_control_frame.data[1] = 8;
    pending_isotp_flow_control_frame.data[2] = 0;
    pending_isotp_flow_control = true;
}

static void can_rx_queue_clear(void) {
    can_rx_head = 0;
    can_rx_tail = 0;
    can_rx_count = 0;
}

static void isotp_reset_link(uint32_t send_can_id) {
    isotp_init_link(&isotp_link,
                    send_can_id,
                    isotp_send_buffer,
                    sizeof(isotp_send_buffer),
                    isotp_recv_buffer,
                    sizeof(isotp_recv_buffer));
    isotp_tx_expected_len = 0;
    isotp_tx_received_len = 0;
    isotp_rx_out_len = 0;
    isotp_rx_out_offset = 0;
}

static bool isotp_frame_matches_rx(const picoj_can_frame_t* frame) {
    if (!isotp_configured || !frame) {
        return false;
    }
    if (frame->can_id != isotp_rx_can_id) {
        return false;
    }
    return ((frame->flags ^ isotp_rx_flags) & PICOJ_CAN_EXTENDED) == 0;
}

static bool isotp_output_busy(void) {
    return isotp_rx_out_offset < isotp_rx_out_len;
}

static void isotp_publish_rx_payload(uint16_t len) {
    isotp_rx_out_can_id = isotp_rx_can_id;
    isotp_rx_out_flags = isotp_rx_flags;
    isotp_rx_out_len = len;
    isotp_rx_out_offset = 0;
}

static void isotp_queue_received_payload(void) {
    if (isotp_output_busy()) {
        return;
    }

    uint16_t out_len = 0;
    if (isotp_receive(&isotp_link, isotp_rx_out_buffer, sizeof(isotp_rx_out_buffer), &out_len) != ISOTP_RET_OK) {
        return;
    }

    isotp_publish_rx_payload(out_len);
}

static void isotp_handle_can_rx(const picoj_can_frame_t* frame) {
    if (!isotp_frame_matches_rx(frame)) {
        return;
    }

    const uint8_t pci_type = frame->dlc > 0 ? (uint8_t)(frame->data[0] >> 4) : 0xFFu;
    if (isotp_output_busy() && (pci_type == 0 || pci_type == 1)) {
        if (pci_type == 1) {
            queue_isotp_flow_control_frame(2);
        }
        return;
    }

    isotp_on_can_message(&isotp_link, (uint8_t*)frame->data, frame->dlc);
    isotp_queue_received_payload();
}

static void flush_isotp_rx(void) {
    while (isotp_rx_out_offset < isotp_rx_out_len && tud_vendor_mounted() &&
           tud_vendor_write_available() >= sizeof(picoj_packet_t)) {
        picoj_isotp_chunk_t chunk;
        memset(&chunk, 0, sizeof(chunk));
        chunk.can_id = isotp_rx_out_can_id;
        chunk.total_len = isotp_rx_out_len;
        chunk.offset = isotp_rx_out_offset;
        chunk.flags = isotp_rx_out_flags;
        uint16_t remaining = (uint16_t)(isotp_rx_out_len - isotp_rx_out_offset);
        chunk.chunk_len = remaining > PICOJ_ISOTP_CHUNK_DATA_SIZE ? PICOJ_ISOTP_CHUNK_DATA_SIZE : (uint8_t)remaining;
        memcpy(chunk.data, isotp_rx_out_buffer + isotp_rx_out_offset, chunk.chunk_len);
        if (!send_packet(0, PICOJ_CMD_ISOTP_RX, &chunk, sizeof(chunk), 0)) {
            break;
        }
        isotp_rx_out_offset = (uint16_t)(isotp_rx_out_offset + chunk.chunk_len);
        tud_task();
    }
    if (isotp_rx_out_len != 0 && isotp_rx_out_offset >= isotp_rx_out_len) {
        isotp_rx_out_len = 0;
        isotp_rx_out_offset = 0;
    }
}

static void can_rx_queue_push(const picoj_can_frame_t* frame) {
    if (!frame) {
        return;
    }
    if (can_rx_count == CAN_RX_QUEUE_CAPACITY) {
        can_rx_tail = (uint8_t)((can_rx_tail + 1u) % CAN_RX_QUEUE_CAPACITY);
        --can_rx_count;
        ++can_rx_overflows;
    }
    can_rx_queue[can_rx_head] = *frame;
    can_rx_head = (uint8_t)((can_rx_head + 1u) % CAN_RX_QUEUE_CAPACITY);
    ++can_rx_count;
}

static bool can_rx_queue_peek(picoj_can_frame_t* frame) {
    if (!frame || can_rx_count == 0) {
        return false;
    }
    *frame = can_rx_queue[can_rx_tail];
    return true;
}

static void can_rx_queue_drop(void) {
    if (can_rx_count == 0) {
        return;
    }
    can_rx_tail = (uint8_t)((can_rx_tail + 1u) % CAN_RX_QUEUE_CAPACITY);
    --can_rx_count;
}

static void drain_can_rx(void) {
    is_draining_can_rx = true;
    for (uint8_t i = 0; i < CAN_RX_DRAIN_LIMIT; ++i) {
        picoj_can_frame_t frame;
        if (!mcp2515_read(&frame)) {
            break;
        }
        can_rx_queue_push(&frame);
        if (isotp_frame_matches_rx(&frame)) {
            isotp_handle_can_rx(&frame);
        }
        led_mark_activity();
    }
    is_draining_can_rx = false;
    send_pending_isotp_flow_control();
}

static void poll_isotp(void) {
    if (!isotp_configured) {
        return;
    }
    for (uint8_t i = 0; i < ISOTP_DRAIN_LIMIT; ++i) {
        isotp_poll(&isotp_link);
        if (isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
            break;
        }
    }
}

static void flush_can_rx(void) {
    while (can_rx_count > 0 && tud_vendor_mounted() && tud_vendor_write_available() >= sizeof(picoj_packet_t)) {
        picoj_can_frame_t frame;
        if (!can_rx_queue_peek(&frame)) {
            break;
        }
        if (!send_packet(0, PICOJ_CMD_CAN_RX, &frame, sizeof(frame), 0)) {
            break;
        }
        can_rx_queue_drop();
        tud_task();
    }
}

static void process_host_packet(const picoj_packet_t* packet) {
    if (!packet || packet->magic != PICOJ_PACKET_MAGIC || packet->len > PICOJ_PACKET_PAYLOAD_SIZE) {
        return;
    }

    switch (packet->cmd) {
    case PICOJ_CMD_HELLO: {
        picoj_hello_t hello = {
            .version = PICOJ_USB_VERSION,
            .max_channels = 1,
            .reserved = 0,
        };
        send_packet(packet->seq, PICOJ_CMD_HELLO, &hello, sizeof(hello), PICOJ_USB_REPLY_TIMEOUT_MS);
        break;
    }
    case PICOJ_CMD_SET_BITRATE: {
        if (packet->len < sizeof(picoj_bitrate_t)) {
            send_status(packet->seq, -1, 0);
            break;
        }
        picoj_bitrate_t bitrate;
        memcpy(&bitrate, packet->payload, sizeof(bitrate));
        can_ready = mcp2515_init(bitrate.bitrate);
        can_error = !can_ready;
        can_rx_queue_clear();
        isotp_configured = false;
        send_status(packet->seq, can_ready ? 0 : -2, bitrate.bitrate);
        break;
    }
    case PICOJ_CMD_CAN_TX: {
        if (packet->len < sizeof(picoj_can_frame_t) || !can_ready) {
            send_status(packet->seq, -1, 0);
            break;
        }
        picoj_can_frame_t frame;
        memcpy(&frame, packet->payload, sizeof(frame));
        mcp2515_tx_result_t tx = mcp2515_send(&frame);
        if (tx == MCP2515_TX_OK) {
            led_mark_activity();
        }
        send_status(packet->seq, tx == MCP2515_TX_OK ? 0 : (tx == MCP2515_TX_BUSY ? -3 : -4), frame.can_id);
        break;
    }
    case PICOJ_CMD_CLEAR_RX:
        can_rx_queue_clear();
        isotp_rx_out_len = 0;
        isotp_rx_out_offset = 0;
        send_status(packet->seq, 0, can_rx_overflows);
        break;
    case PICOJ_CMD_BOOTLOADER:
        send_status(packet->seq, 0, PICOJ_USB_VERSION);
        tud_task();
        sleep_ms(100);
        reset_usb_boot(0, 0);
        break;
    case PICOJ_CMD_ISOTP_CONFIG: {
        if (packet->len < sizeof(picoj_isotp_config_t)) {
            send_status(packet->seq, -1, 0);
            break;
        }
        picoj_isotp_config_t config;
        memcpy(&config, packet->payload, sizeof(config));
        if (config.tx_can_id == 0 || config.rx_can_id == 0) {
            isotp_configured = false;
            isotp_tx_can_id = 0;
            isotp_rx_can_id = 0;
            isotp_flow_control_can_id = 0;
            isotp_tx_flags = 0;
            isotp_rx_flags = 0;
            isotp_flow_control_flags = 0;
            isotp_reset_link(0);
            send_status(packet->seq, 0, 0);
            break;
        }
        isotp_tx_can_id = config.tx_can_id;
        isotp_rx_can_id = config.rx_can_id;
        isotp_flow_control_can_id = config.flow_control_can_id ? config.flow_control_can_id : config.tx_can_id;
        isotp_tx_flags = config.tx_flags & PICOJ_CAN_EXTENDED;
        isotp_rx_flags = config.rx_flags & PICOJ_CAN_EXTENDED;
        isotp_flow_control_flags = config.tx_flags & PICOJ_CAN_EXTENDED;
        isotp_configured = true;
        isotp_reset_link(config.flow_control_can_id ? config.flow_control_can_id : config.tx_can_id);
        send_status(packet->seq, 0, isotp_rx_can_id);
        break;
    }
    case PICOJ_CMD_ISOTP_TX: {
        if (packet->len < sizeof(picoj_isotp_chunk_t) || !can_ready || !isotp_configured) {
            send_status(packet->seq, -1, 0);
            break;
        }
        picoj_isotp_chunk_t chunk;
        memcpy(&chunk, packet->payload, sizeof(chunk));
        if (chunk.total_len > PICOJ_ISOTP_MAX_PAYLOAD || chunk.chunk_len > PICOJ_ISOTP_CHUNK_DATA_SIZE ||
            chunk.offset > chunk.total_len || (uint16_t)(chunk.offset + chunk.chunk_len) > chunk.total_len) {
            send_status(packet->seq, -2, chunk.total_len);
            break;
        }
        if (chunk.offset == 0) {
            if (isotp_link.send_status == ISOTP_SEND_STATUS_INPROGRESS) {
                send_status(packet->seq, -3, chunk.can_id);
                break;
            }
            isotp_tx_expected_len = chunk.total_len;
            isotp_tx_received_len = 0;
            isotp_tx_can_id = chunk.can_id;
            isotp_tx_flags = chunk.flags & PICOJ_CAN_EXTENDED;
            isotp_link.send_arbitration_id = chunk.can_id;
        }
        if (chunk.total_len != isotp_tx_expected_len || chunk.offset != isotp_tx_received_len) {
            send_status(packet->seq, -2, chunk.offset);
            break;
        }
        memcpy(isotp_tx_staging_buffer + chunk.offset, chunk.data, chunk.chunk_len);
        isotp_tx_received_len = (uint16_t)(isotp_tx_received_len + chunk.chunk_len);
        if (isotp_tx_received_len == isotp_tx_expected_len) {
            int ret = isotp_send_with_id(&isotp_link, chunk.can_id, isotp_tx_staging_buffer, isotp_tx_expected_len);
            send_status(packet->seq, ret == ISOTP_RET_OK ? 0 : ret, chunk.can_id);
        } else {
            send_status(packet->seq, 0, isotp_tx_received_len);
        }
        break;
    }
    default:
        send_status(packet->seq, -127, packet->cmd);
        break;
    }
}

int main(void) {
    board_init();
    led_init();
    mcp2515_hw_init();
    can_ready = mcp2515_init(PICOJ_DEFAULT_CAN_BITRATE);
    can_error = !can_ready;
    can_rx_queue_clear();
    tusb_init();

    while (true) {
        tud_task();
        led_update();

        if (tud_vendor_available() >= sizeof(picoj_packet_t)) {
            picoj_packet_t packet;
            tud_vendor_read(&packet, sizeof(packet));
            led_mark_activity();
            process_host_packet(&packet);
        }

        if (can_ready) {
            can_ready = mcp2515_poll();
            can_error = !can_ready;
        }
        if (can_ready) {
            drain_can_rx();
            poll_isotp();
            flush_isotp_rx();
            flush_can_rx();
        }
    }
}
