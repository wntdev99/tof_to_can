#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

/* RP2040 → MCP25625 SPI 핀 (PCB 고정) */
#define MCP_SPI_PORT    spi1
#define MCP_PIN_MISO    8
#define MCP_PIN_SCK     14
#define MCP_PIN_MOSI    15
#define MCP_PIN_CS      19
#define MCP_PIN_INT     22
#define MCP_PIN_RESET   18
#define MCP_PIN_STBY    16
#define MCP_SPI_SPEED   10000000U   /* 10 MHz */

/* MCP2515 SPI 명령어 */
#define MCP_CMD_RESET       0xC0
#define MCP_CMD_READ        0x03
#define MCP_CMD_WRITE       0x02
#define MCP_CMD_RTS_TX0     0x81
#define MCP_CMD_READ_STATUS 0xA0
#define MCP_CMD_BIT_MODIFY  0x05
#define MCP_CMD_LOAD_TX0    0x40
#define MCP_CMD_READ_RX0    0x90

/* 주요 레지스터 */
#define MCP_REG_CANCTRL     0x0F
#define MCP_REG_CANSTAT     0x0E
#define MCP_REG_CNF1        0x2A
#define MCP_REG_CNF2        0x29
#define MCP_REG_CNF3        0x28
#define MCP_REG_CANINTE     0x2B
#define MCP_REG_CANINTF     0x2C
#define MCP_REG_TXB0CTRL    0x30
#define MCP_REG_TXB0SIDH    0x31
#define MCP_REG_TXB0SIDL    0x32
#define MCP_REG_TXB0DLC     0x35
#define MCP_REG_TXB0D0      0x36

/* CANCTRL 모드 */
#define MCP_MODE_NORMAL     0x00
#define MCP_MODE_LOOPBACK   0x40
#define MCP_MODE_CONFIG     0x80
#define MCP_MODE_MASK       0xE0

/* 비트타이밍 (12MHz 크리스탈, 500kbps) */
#define MCP_CNF1_500K_12MHZ 0x00
#define MCP_CNF2_500K_12MHZ 0x9A
#define MCP_CNF3_500K_12MHZ 0x02

bool mcp25625_init(void);
bool mcp25625_set_normal_mode(void);
bool mcp25625_set_loopback_mode(void);
bool mcp25625_send_frame(uint16_t id, const uint8_t *data, uint8_t dlc);
bool mcp25625_loopback_test(void);

uint8_t mcp25625_read_reg(uint8_t addr);
void    mcp25625_write_reg(uint8_t addr, uint8_t val);
void    mcp25625_bit_modify(uint8_t addr, uint8_t mask, uint8_t val);
