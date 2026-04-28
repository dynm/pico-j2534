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

static void send_packet(uint8_t seq, uint8_t cmd, const void* payload, uint8_t len) {
    if (!tud_vendor_mounted() || tud_vendor_write_available() < sizeof(picoj_packet_t)) {
        return;
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
}

static void send_status(uint8_t seq, int32_t code, uint32_t detail) {
    picoj_status_t status = {
        .code = code,
        .detail = detail,
    };
    send_packet(seq, PICOJ_CMD_STATUS, &status, sizeof(status));
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
        send_packet(packet->seq, PICOJ_CMD_HELLO, &hello, sizeof(hello));
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
        can_error = !sent;
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
    tusb_init();

    while (true) {
        tud_task();
        led_update();

        picoj_can_frame_t frame;
        if (tud_vendor_available() >= sizeof(picoj_packet_t)) {
            picoj_packet_t packet;
            tud_vendor_read(&packet, sizeof(packet));
            led_mark_activity();
            process_host_packet(&packet);
        }

        if (can_ready && tud_vendor_mounted() && mcp2515_read(&frame)) {
            led_mark_activity();
            send_packet(0, PICOJ_CMD_CAN_RX, &frame, sizeof(frame));
        }
    }
}
