#include "mcp2515.h"

#include <string.h>

#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

#define MCP_RESET 0xC0
#define MCP_READ 0x03
#define MCP_WRITE 0x02
#define MCP_BITMOD 0x05
#define MCP_RTS_TXB0 0x81
#define MCP_READ_STATUS 0xA0

#define CANCTRL 0x0F
#define CANSTAT 0x0E
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B
#define CANINTF 0x2C
#define EFLG 0x2D
#define RXB0CTRL 0x60
#define RXB1CTRL 0x70
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB1CTRL 0x40
#define TXB1SIDH 0x41
#define TXB2CTRL 0x50
#define TXB2SIDH 0x51
#define RXB0SIDH 0x61
#define RXB1SIDH 0x71
#define MCP_TX_TIMEOUT_US 10000u

#define CANCTRL_ABORT_ALL 0x10
#define TXB_TXREQ 0x08
#define TXB_ABTF 0x40
#define TXB_MLOA 0x20
#define TXB_TXERR 0x10
#define TXB_ERROR_FLAGS (TXB_ABTF | TXB_MLOA | TXB_TXERR)
#define TXB_FATAL_FLAGS (TXB_ABTF | TXB_TXERR)

static void cs_select(void) {
    gpio_put(PICOJ_CAN_PIN_CS, 0);
}

static void cs_deselect(void) {
    gpio_put(PICOJ_CAN_PIN_CS, 1);
}

static uint8_t spi_xfer(uint8_t value) {
    uint8_t rx = 0;
    spi_write_read_blocking(PICOJ_CAN_SPI, &value, &rx, 1);
    return rx;
}

static void write_reg(uint8_t reg, uint8_t value) {
    cs_select();
    spi_xfer(MCP_WRITE);
    spi_xfer(reg);
    spi_xfer(value);
    cs_deselect();
}

static uint8_t read_reg(uint8_t reg) {
    cs_select();
    spi_xfer(MCP_READ);
    spi_xfer(reg);
    uint8_t value = spi_xfer(0);
    cs_deselect();
    return value;
}

static void bit_modify(uint8_t reg, uint8_t mask, uint8_t value) {
    cs_select();
    spi_xfer(MCP_BITMOD);
    spi_xfer(reg);
    spi_xfer(mask);
    spi_xfer(value);
    cs_deselect();
}

static void reset_chip(void) {
    cs_select();
    spi_xfer(MCP_RESET);
    cs_deselect();
    sleep_ms(10);
}

static bool set_mode(uint8_t mode) {
    bit_modify(CANCTRL, 0xE0, mode);
    for (int i = 0; i < 20; ++i) {
        if ((read_reg(CANSTAT) & 0xE0) == mode) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

static bool set_cnf(uint32_t bitrate) {
#if PICOJ_MCP2515_OSC_HZ == 16000000u
    if (bitrate == 500000u) {
        write_reg(CNF1, 0x00);
        write_reg(CNF2, 0xAC);
        write_reg(CNF3, 0x03);
        return true;
    }
    if (bitrate == 250000u) {
        write_reg(CNF1, 0x01);
        write_reg(CNF2, 0xAC);
        write_reg(CNF3, 0x03);
        return true;
    }
    if (bitrate == 125000u) {
        write_reg(CNF1, 0x03);
        write_reg(CNF2, 0xAC);
        write_reg(CNF3, 0x03);
        return true;
    }
#elif PICOJ_MCP2515_OSC_HZ == 8000000u
    if (bitrate == 500000u) {
        write_reg(CNF1, 0x00);
        write_reg(CNF2, 0x91);
        write_reg(CNF3, 0x01);
        return true;
    }
    if (bitrate == 250000u) {
        write_reg(CNF1, 0x00);
        write_reg(CNF2, 0xAC);
        write_reg(CNF3, 0x03);
        return true;
    }
    if (bitrate == 125000u) {
        write_reg(CNF1, 0x01);
        write_reg(CNF2, 0xAC);
        write_reg(CNF3, 0x03);
        return true;
    }
#endif
    return false;
}

static void write_id(uint8_t base, const picoj_can_frame_t* frame) {
    uint32_t id = frame->can_id;
    if (frame->flags & PICOJ_CAN_EXTENDED) {
        write_reg(base + 0, (uint8_t)(id >> 21));
        write_reg(base + 1, (uint8_t)(((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03)));
        write_reg(base + 2, (uint8_t)(id >> 8));
        write_reg(base + 3, (uint8_t)id);
    } else {
        id &= 0x7FF;
        write_reg(base + 0, (uint8_t)(id >> 3));
        write_reg(base + 1, (uint8_t)(id << 5));
        write_reg(base + 2, 0);
        write_reg(base + 3, 0);
    }
}

static bool tx_buffer_available(uint8_t ctrl_reg) {
    uint8_t ctrl = read_reg(ctrl_reg);
    if (ctrl & TXB_ERROR_FLAGS) {
        bit_modify(ctrl_reg, TXB_TXREQ | TXB_ERROR_FLAGS, 0);
        ctrl = read_reg(ctrl_reg);
    }
    return (ctrl & TXB_TXREQ) == 0;
}

static void read_id(uint8_t base, picoj_can_frame_t* frame) {
    uint8_t sidh = read_reg(base + 0);
    uint8_t sidl = read_reg(base + 1);
    uint8_t eid8 = read_reg(base + 2);
    uint8_t eid0 = read_reg(base + 3);
    if (sidl & 0x08) {
        frame->can_id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl & 0xE0) << 13) |
                        ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
        frame->flags = PICOJ_CAN_EXTENDED;
    } else {
        frame->can_id = ((uint32_t)sidh << 3) | (sidl >> 5);
        frame->flags = 0;
    }
}

void mcp2515_hw_init(void) {
    spi_init(PICOJ_CAN_SPI, PICOJ_CAN_SPI_BAUD);
    gpio_set_function(PICOJ_CAN_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PICOJ_CAN_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PICOJ_CAN_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PICOJ_CAN_PIN_CS);
    gpio_set_dir(PICOJ_CAN_PIN_CS, GPIO_OUT);
    cs_deselect();
    gpio_init(PICOJ_CAN_PIN_INT);
    gpio_set_dir(PICOJ_CAN_PIN_INT, GPIO_IN);
    gpio_pull_up(PICOJ_CAN_PIN_INT);
}

bool mcp2515_init(uint32_t bitrate) {
    reset_chip();
    if (!mcp2515_set_bitrate(bitrate)) {
        return false;
    }
    write_reg(CANINTF, 0x00);
    write_reg(EFLG, 0x00);
    write_reg(TXB0CTRL, 0x00);
    write_reg(TXB1CTRL, 0x00);
    write_reg(TXB2CTRL, 0x00);
    write_reg(RXB0CTRL, 0x64); // Accept all frames and roll RXB0 over into RXB1.
    write_reg(RXB1CTRL, 0x60);
    write_reg(CANINTE, 0x03);
    return set_mode(0x00);
}

bool mcp2515_set_bitrate(uint32_t bitrate) {
    if (!set_mode(0x80)) {
        return false;
    }
    if (!set_cnf(bitrate)) {
        return false;
    }
    return true;
}

mcp2515_tx_result_t mcp2515_send(const picoj_can_frame_t* frame) {
    if (!frame || frame->dlc > 8) {
        return MCP2515_TX_ERROR;
    }

    uint8_t ctrl_reg = 0;
    uint8_t id_reg = 0;
    uint8_t rts = 0;
    if (tx_buffer_available(TXB0CTRL)) {
        ctrl_reg = TXB0CTRL;
        id_reg = TXB0SIDH;
        rts = MCP_RTS_TXB0;
    } else if (tx_buffer_available(TXB1CTRL)) {
        ctrl_reg = TXB1CTRL;
        id_reg = TXB1SIDH;
        rts = 0x82;
    } else if (tx_buffer_available(TXB2CTRL)) {
        ctrl_reg = TXB2CTRL;
        id_reg = TXB2SIDH;
        rts = 0x84;
    } else {
        return MCP2515_TX_BUSY;
    }

    bit_modify(ctrl_reg, TXB_ERROR_FLAGS, 0);
    write_id(id_reg, frame);
    uint8_t dlc = frame->dlc & 0x0F;
    if (frame->flags & PICOJ_CAN_RTR) {
        dlc |= 0x40;
    }
    write_reg(id_reg + 4, dlc);
    for (uint8_t i = 0; i < frame->dlc; ++i) {
        write_reg(id_reg + 5 + i, frame->data[i]);
    }

    cs_select();
    spi_xfer(rts);
    cs_deselect();

    const absolute_time_t deadline = make_timeout_time_us(MCP_TX_TIMEOUT_US);
    while (!time_reached(deadline)) {
        uint8_t ctrl = read_reg(ctrl_reg);
        if (ctrl & TXB_FATAL_FLAGS) {
            bit_modify(ctrl_reg, TXB_TXREQ | TXB_ERROR_FLAGS, 0);
            return MCP2515_TX_ERROR;
        }
        if ((ctrl & TXB_TXREQ) == 0) {
            return MCP2515_TX_OK;
        }
        sleep_us(100);
    }

    bit_modify(ctrl_reg, TXB_TXREQ | TXB_ERROR_FLAGS, 0);
    return MCP2515_TX_ERROR;
}

bool mcp2515_read(picoj_can_frame_t* frame) {
    uint8_t intf = read_reg(CANINTF);
    uint8_t base = 0;
    uint8_t clear_mask = 0;
    if (intf & 0x01) {
        base = RXB0SIDH;
        clear_mask = 0x01;
    } else if (intf & 0x02) {
        base = RXB1SIDH;
        clear_mask = 0x02;
    } else {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    read_id(base, frame);
    uint8_t dlc = read_reg(base + 4);
    frame->dlc = dlc & 0x0F;
    if (dlc & 0x40) {
        frame->flags |= PICOJ_CAN_RTR;
    }
    if (frame->dlc > 8) {
        frame->dlc = 8;
    }
    for (uint8_t i = 0; i < frame->dlc; ++i) {
        frame->data[i] = read_reg(base + 5 + i);
    }
    bit_modify(CANINTF, clear_mask, 0);
    bit_modify(EFLG, 0xC0, 0);
    return true;
}
