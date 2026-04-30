#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pico_j2534_protocol.h"

void mcp2515_hw_init(void);
bool mcp2515_init(uint32_t bitrate);
bool mcp2515_set_bitrate(uint32_t bitrate);
typedef enum {
    MCP2515_TX_OK = 0,
    MCP2515_TX_BUSY,
    MCP2515_TX_ERROR,
} mcp2515_tx_result_t;

mcp2515_tx_result_t mcp2515_send(const picoj_can_frame_t* frame);
bool mcp2515_read(picoj_can_frame_t* frame);
