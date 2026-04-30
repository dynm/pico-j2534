#include <string.h>

#include "board_config.h"
#include "bsp/board.h"
#include "hardware/gpio.h"
#include "mcp2515.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico_j2534_protocol.h"
#include "tusb.h"

static bool can_ready = false;
static bool can_error = false;
static uint32_t led_activity_until_ms = 0;

#define PICOJ_USB_REPLY_TIMEOUT_MS 1000u
#define CAN_RX_QUEUE_CAPACITY 64u
#define CAN_RX_DRAIN_LIMIT 16u

static picoj_can_frame_t can_rx_queue[CAN_RX_QUEUE_CAPACITY];
static uint8_t can_rx_head = 0;
static uint8_t can_rx_tail = 0;
static uint8_t can_rx_count = 0;
static uint32_t can_rx_overflows = 0;

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

static void can_rx_queue_clear(void) {
    can_rx_head = 0;
    can_rx_tail = 0;
    can_rx_count = 0;
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
    for (uint8_t i = 0; i < CAN_RX_DRAIN_LIMIT; ++i) {
        picoj_can_frame_t frame;
        if (!mcp2515_read(&frame)) {
            break;
        }
        can_rx_queue_push(&frame);
        led_mark_activity();
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
        bool sent = mcp2515_send(&frame);
        if (sent) {
            led_mark_activity();
        }
        send_status(packet->seq, sent ? 0 : -3, frame.can_id);
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
            drain_can_rx();
            flush_can_rx();
        }
    }
}
