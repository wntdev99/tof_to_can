#include "mcp25625.h"
#include "pico/stdlib.h"
#include <string.h>

static inline void cs_low(void)  { gpio_put(MCP_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(MCP_PIN_CS, 1); }

static void spi_write(const uint8_t *buf, size_t len) {
    spi_write_blocking(MCP_SPI_PORT, buf, len);
}

static void spi_read(uint8_t *buf, size_t len) {
    spi_read_blocking(MCP_SPI_PORT, 0x00, buf, len);
}

uint8_t mcp25625_read_reg(uint8_t addr) {
    uint8_t cmd[2] = { MCP_CMD_READ, addr };
    uint8_t val = 0;
    cs_low();
    spi_write(cmd, 2);
    spi_read(&val, 1);
    cs_high();
    return val;
}

void mcp25625_write_reg(uint8_t addr, uint8_t val) {
    uint8_t cmd[3] = { MCP_CMD_WRITE, addr, val };
    cs_low();
    spi_write(cmd, 3);
    cs_high();
}

void mcp25625_bit_modify(uint8_t addr, uint8_t mask, uint8_t val) {
    uint8_t cmd[4] = { MCP_CMD_BIT_MODIFY, addr, mask, val };
    cs_low();
    spi_write(cmd, 4);
    cs_high();
}

static void hw_reset(void) {
    gpio_put(MCP_PIN_RESET, 0);
    sleep_ms(1);
    gpio_put(MCP_PIN_RESET, 1);
    sleep_ms(5);
}

static void soft_reset(void) {
    uint8_t cmd = MCP_CMD_RESET;
    cs_low();
    spi_write(&cmd, 1);
    cs_high();
    sleep_ms(5);
}

static bool wait_mode(uint8_t mode, uint32_t timeout_ms) {
    uint32_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        if ((mcp25625_read_reg(MCP_REG_CANSTAT) & MCP_MODE_MASK) == mode)
            return true;
        sleep_ms(1);
    }
    return false;
}

bool mcp25625_init(void) {
    /* SPI 초기화 */
    spi_init(MCP_SPI_PORT, MCP_SPI_SPEED);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(MCP_PIN_CS);    gpio_set_dir(MCP_PIN_CS,    GPIO_OUT); gpio_put(MCP_PIN_CS,    1);
    gpio_init(MCP_PIN_RESET); gpio_set_dir(MCP_PIN_RESET, GPIO_OUT); gpio_put(MCP_PIN_RESET, 1);
    gpio_init(MCP_PIN_STBY);  gpio_set_dir(MCP_PIN_STBY,  GPIO_OUT); gpio_put(MCP_PIN_STBY,  0);
    gpio_init(MCP_PIN_INT);   gpio_set_dir(MCP_PIN_INT,   GPIO_IN);

    hw_reset();
    soft_reset();

    /* CONFIG 모드 진입 확인 */
    if (!wait_mode(MCP_MODE_CONFIG, 10))
        return false;

    /* 비트타이밍: 12MHz XTAL, 500kbps */
    mcp25625_write_reg(MCP_REG_CNF1, MCP_CNF1_500K_16MHZ);
    mcp25625_write_reg(MCP_REG_CNF2, MCP_CNF2_500K_16MHZ);
    mcp25625_write_reg(MCP_REG_CNF3, MCP_CNF3_500K_16MHZ);

    /* TX 인터럽트 비활성화 (폴링 모드) */
    mcp25625_write_reg(MCP_REG_CANINTE, 0x00);
    mcp25625_write_reg(MCP_REG_CANINTF, 0x00);

    return true;
}

bool mcp25625_set_normal_mode(void) {
    mcp25625_bit_modify(MCP_REG_CANCTRL, MCP_MODE_MASK, MCP_MODE_NORMAL);
    return wait_mode(MCP_MODE_NORMAL, 10);
}

bool mcp25625_set_loopback_mode(void) {
    mcp25625_bit_modify(MCP_REG_CANCTRL, MCP_MODE_MASK, MCP_MODE_LOOPBACK);
    return wait_mode(MCP_MODE_LOOPBACK, 10);
}

bool mcp25625_send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
    /* TXB0 사용, 11비트 표준 ID */
    uint8_t sidh = (id >> 3) & 0xFF;
    uint8_t sidl = (id & 0x07) << 5;

    mcp25625_write_reg(MCP_REG_TXB0SIDH, sidh);
    mcp25625_write_reg(MCP_REG_TXB0SIDL, sidl);
    mcp25625_write_reg(MCP_REG_TXB0DLC,  dlc & 0x0F);

    for (uint8_t i = 0; i < dlc; i++)
        mcp25625_write_reg(MCP_REG_TXB0D0 + i, data[i]);

    /* 전송 요청 */
    uint8_t rts = MCP_CMD_RTS_TX0;
    cs_low();
    spi_write(&rts, 1);
    cs_high();

    /* 전송 완료 대기 (최대 5ms) */
    uint32_t deadline = to_ms_since_boot(get_absolute_time()) + 5;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        if (!(mcp25625_read_reg(MCP_REG_TXB0CTRL) & 0x08))
            return true;
    }
    return false;
}

bool mcp25625_loopback_test(void) {
    if (!mcp25625_set_loopback_mode())
        return false;

    uint8_t tx[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
    if (!mcp25625_send_frame(0x7FF, tx, 8))
        return false;

    sleep_ms(2);

    /* RXB0에서 수신 확인 */
    uint8_t intf = mcp25625_read_reg(MCP_REG_CANINTF);
    if (!(intf & 0x01))
        return false;

    uint8_t rx[8];
    uint8_t cmd = MCP_CMD_READ_RX0;
    cs_low();
    spi_write(&cmd, 1);
    uint8_t header[5];
    spi_read(header, 5);
    spi_read(rx, 8);
    cs_high();

    mcp25625_bit_modify(MCP_REG_CANINTF, 0x01, 0x00);

    return (memcmp(tx, rx, 8) == 0);
}
