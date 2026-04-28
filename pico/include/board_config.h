#pragma once

#include "hardware/spi.h"

// MCP2515-compatible CAN controller wiring on SPI1.
#define PICOJ_CAN_SPI spi1
#define PICOJ_CAN_SPI_BAUD 10000000u
#define PICOJ_CAN_PIN_INT 8
#define PICOJ_CAN_PIN_CS 9
#define PICOJ_CAN_PIN_SCK 10
#define PICOJ_CAN_PIN_MOSI 11
#define PICOJ_CAN_PIN_MISO 12

#define PICOJ_STATUS_LED_PIN 25
#define PICOJ_STATUS_LED_ACTIVE_HIGH 1

// Most MCP2515 CAN boards use either 8 MHz or 16 MHz crystals.
#define PICOJ_MCP2515_OSC_HZ 16000000u
#define PICOJ_DEFAULT_CAN_BITRATE 500000u
