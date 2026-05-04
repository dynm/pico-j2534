#pragma once

#include <stdint.h>

#define PICOJ_USB_VID 0x1209
#define PICOJ_USB_PID 0x2534
#define PICOJ_USB_VERSION 0x0106

#define PICOJ_USB_PACKET_SIZE 64
#define PICOJ_PACKET_MAGIC 0x4A50u
#define PICOJ_PACKET_HEADER_SIZE 5
#define PICOJ_PACKET_PAYLOAD_SIZE (PICOJ_USB_PACKET_SIZE - PICOJ_PACKET_HEADER_SIZE)

typedef enum picoj_cmd {
    PICOJ_CMD_HELLO = 0x01,
    PICOJ_CMD_SET_BITRATE = 0x02,
    PICOJ_CMD_CAN_TX = 0x03,
    PICOJ_CMD_CAN_RX = 0x04,
    PICOJ_CMD_STATUS = 0x05,
    PICOJ_CMD_CLEAR_RX = 0x06,
    PICOJ_CMD_ISOTP_CONFIG = 0x07,
    PICOJ_CMD_ISOTP_RX = 0x08,
    PICOJ_CMD_ISOTP_TX = 0x09,
    PICOJ_CMD_BOOTLOADER = 0x0A,
} picoj_cmd_t;

#define PICOJ_ISOTP_MAX_PAYLOAD 4095u
#define PICOJ_ISOTP_CHUNK_DATA_SIZE 49u

typedef enum picoj_can_flags {
    PICOJ_CAN_EXTENDED = 0x01,
    PICOJ_CAN_RTR = 0x02,
} picoj_can_flags_t;

#pragma pack(push, 1)
typedef struct picoj_packet {
    uint16_t magic;
    uint8_t seq;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PICOJ_PACKET_PAYLOAD_SIZE];
} picoj_packet_t;

typedef struct picoj_hello {
    uint16_t version;
    uint8_t max_channels;
    uint8_t reserved;
} picoj_hello_t;

typedef struct picoj_bitrate {
    uint32_t bitrate;
} picoj_bitrate_t;

typedef struct picoj_can_frame {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[8];
} picoj_can_frame_t;

typedef struct picoj_status {
    int32_t code;
    uint32_t detail;
} picoj_status_t;

typedef struct picoj_isotp_config {
    uint32_t tx_can_id;
    uint32_t rx_can_id;
    uint32_t flow_control_can_id;
    uint8_t tx_flags;
    uint8_t rx_flags;
    uint8_t reserved[2];
} picoj_isotp_config_t;

typedef struct picoj_isotp_chunk {
    uint32_t can_id;
    uint16_t total_len;
    uint16_t offset;
    uint8_t chunk_len;
    uint8_t flags;
    uint8_t data[PICOJ_ISOTP_CHUNK_DATA_SIZE];
} picoj_isotp_chunk_t;
#pragma pack(pop)
