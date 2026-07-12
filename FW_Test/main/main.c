/**
 * @file main.c
 * @brief RetroMeshDevice — Hardware Test Suite v2
 *
 * Hardware:
 *  - OLED SH1106 128x64 @ I2C 0x3C   (same bus as the MCP23017)
 *  - MCP23017 @ I2C 0x20             (A0/A1/A2 tied to GND)
 *  - WIO SX1262 @ SPI2
 *  - XIAO ESP32-S3 Plus
 *
 *
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "main.h"
#include "esp_sleep.h"

static const char *TAG = "RMD";

#define VBAT_SENSE_EN GPIO_NUM_43
#define VBAT_SENSE_CHANNEL ADC_CHANNEL_3

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_calibration = false;
float readBatteryVoltage(void);


/* =========================================================
 * MCP23017 registers (BANK = 0)
 * ========================================================= */
#define MCP_IODIRA   0x00
#define MCP_IODIRB   0x01
#define MCP_GPINTENA 0x04
#define MCP_GPINTENB 0x05
#define MCP_DEFVALA  0x06
#define MCP_DEFVALB  0x07
#define MCP_INTCONA  0x08
#define MCP_INTCONB  0x09
#define MCP_IOCON    0x0A
#define MCP_GPPUA    0x0C
#define MCP_GPPUB    0x0D
#define MCP_INTFA    0x0E
#define MCP_INTFB    0x0F
#define MCP_INTCAPA  0x10
#define MCP_INTCAPB  0x11
#define MCP_GPIOA    0x12
#define MCP_GPIOB    0x13
#define IOCON_MIRROR (1 << 6)

/* =========================================================
 * SX1262
 * ========================================================= */
#define SX1262_CMD_GET_STATUS 0xC0
#define SX1262_NOP            0x00

/* =========================================================
 * MCP23017 pin -> key name map
 *
 * Port A (GPA0-7): K_8 K_11 K_10 K_7 K_4 K_1 K_2 K_5
 * Port B (GPB0-7): R_1 R_2  R_3  K_3 K_6 K_9 K_12 NC
 *
 * R_2 = GPB1 -> used as the "next test / skip" button in self-test mode
 * ========================================================= */
static const char *PORT_A_NAME[8] = {
    "K_8", "K_11", "K_10", "K_7", "K_4", "K_1", "K_2", "K_5"
};
static const char *PORT_B_NAME[8] = {
    "R_1", "R_2", "R_3", "K_3", "K_6", "K_9", "K_12", "NC"
};

/* Ordered list of ALL keys for the keypad-walk test (NC excluded) */
typedef struct { const char *name; uint8_t port; uint8_t bit; } KeyDef;
static const KeyDef ALL_KEYS[] = {
    {"K_1",  0, 5}, {"K_2",  0, 6}, {"K_3",  1, 3},
    {"K_4",  0, 4}, {"K_5",  0, 7}, {"K_6",  1, 4},
    {"K_7",  0, 3}, {"K_8",  0, 0}, {"K_9",  1, 5},
    {"K_10", 0, 2}, {"K_11", 0, 1}, {"K_12", 1, 6},
    {"R_1",  1, 0}, {"R_2",  1, 1}, {"R_3",  1, 2},
};
#define N_KEYS (sizeof(ALL_KEYS) / sizeof(ALL_KEYS[0]))
#define R2_PORT  1
#define R2_BIT   1

/* =========================================================
 * ISR queues
 * ========================================================= */
static QueueHandle_t inta_queue = NULL;
static QueueHandle_t intb_queue = NULL;

static void IRAM_ATTR isr_inta(void *arg) {
    uint32_t v = 0;
    xQueueSendFromISR(inta_queue, &v, NULL);
}
static void IRAM_ATTR isr_intb(void *arg) {
    uint32_t v = 0;
    xQueueSendFromISR(intb_queue, &v, NULL);
}

/* =========================================================
 * SPI handle
 * ========================================================= */
static spi_device_handle_t sx_spi = NULL;

/* =========================================================
 * ╔══════════════════════════════════════╗
 * ║         SH1106 OLED DRIVER           ║
 * ║  128x64, I2C, no external library    ║
 * ╚══════════════════════════════════════╝
 *
 * Framebuffer: 128x64 / 8 = 1024 bytes
 * Font: built-in 5x7, 6px wide including spacing
 * ========================================================= */
#define OLED_ADDR      0x3C
#define OLED_W         128
#define OLED_H         64
#define OLED_PAGES     8    /* 64 / 8 */

static uint8_t oled_fb[OLED_PAGES][OLED_W];  /* framebuffer */

/* 5x7 ASCII font (printable chars 0x20-0x7E) */
static const uint8_t FONT5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x00,0x08,0x14,0x22,0x41}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x41,0x22,0x14,0x08,0x00}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x04,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x08,0x14,0x54,0x54,0x3C}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x40,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x08,0x08,0x2A,0x1C,0x08}, /* '~' */
};

/* Low-level I2C write to the OLED */
static esp_err_t oled_i2c_write(uint8_t ctrl, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ctrl, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void oled_cmd(uint8_t c)  { oled_i2c_write(0x00, &c, 1); }
static void oled_data(const uint8_t *d, size_t n) { oled_i2c_write(0x40, d, n); }

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    static const uint8_t init_seq[] = {
        0xAE,       /* display off */
        0xD5,0x80,  /* clock divider */
        0xA8,0x3F,  /* mux ratio 64 */
        0xD3,0x00,  /* display offset 0 */
        0x40,       /* start line 0 */
        0x8D,0x14,  /* charge pump on */
        0x20,0x00,  /* horizontal addressing */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction reversed */
        0xDA,0x12,  /* COM pins config */
        0x81,0xCF,  /* contrast */
        0xD9,0xF1,  /* pre-charge period */
        0xDB,0x40,  /* VCOM deselect level */
        0xA4,       /* resume display from RAM */
        0xA6,       /* normal (not inverted) */
        0xAF,       /* display ON */
    };
    for (int i = 0; i < (int)sizeof(init_seq); i++) oled_cmd(init_seq[i]);
    memset(oled_fb, 0, sizeof(oled_fb));
    printf("[OLED] init OK\n");
}

/* Push the framebuffer to the display */
static void oled_flush(void)
{
    for (int page = 0; page < OLED_PAGES; page++) {
        /* SH1106 RAM starts at column 2 (display is only 128 of 132 columns) */
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);  /* column low nibble  = 0 */
        oled_cmd(0x10);  /* column high nibble = 0 */
        oled_data(oled_fb[page], OLED_W);
    }
}

static void oled_clear(void) { memset(oled_fb, 0, sizeof(oled_fb)); }

/* Draw one character at pixel position (x, page). Returns new x. */
static int oled_putchar(int x, int page, char c)
{
    if (page >= OLED_PAGES || x >= OLED_W) return x;
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *glyph = FONT5X7[c - 0x20];
    for (int col = 0; col < 5 && (x + col) < OLED_W; col++)
        oled_fb[page][x + col] = glyph[col];
    if (x + 5 < OLED_W) oled_fb[page][x + 5] = 0x00; /* letter spacing */
    return x + 6;
}

/* Draw a string at (x, page). Returns final x. */
static int oled_puts(int x, int page, const char *str)
{
    while (*str) x = oled_putchar(x, page, *str++);
    return x;
}

/* printf-style helper to write one OLED text line (page 0-7) */
static void oled_printf(int page, int x, const char *fmt, ...)
{
    char buf[22]; /* 128 px / 6 px per char ~= 21 chars */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    memset(oled_fb[page] + x, 0, OLED_W - x); /* clear rest of the line first */
    oled_puts(x, page, buf);
}

/* Draw a dotted horizontal separator on a page */
static void oled_hline(int page)
{
    memset(oled_fb[page], 0x08, OLED_W);
}

/* Invert a full page (used to highlight a selection) */
static void oled_invert_page(int page)
{
    for (int x = 0; x < OLED_W; x++)
        oled_fb[page][x] ^= 0xFF;
}

/* =========================================================
 * MCP23017 helpers
 * ========================================================= */
static esp_err_t mcp_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mcp_read(uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Configure the MCP23017: all inputs, pull-ups on, interrupt-on-change */
static void mcp_setup(void)
{
    mcp_write(MCP_IOCON,    0x00);        /* MIRROR=0: INTA=PortA, INTB=PortB */
    mcp_write(MCP_IODIRA,   0xFF);
    mcp_write(MCP_IODIRB,   0xFF);
    mcp_write(MCP_GPPUA,    0xFF);
    mcp_write(MCP_GPPUB,    0xFF);
    mcp_write(MCP_INTCONA,  0x00);        /* interrupt = compare to previous value */
    mcp_write(MCP_INTCONB,  0x00);
    mcp_write(MCP_GPINTENA, 0xFF);
    mcp_write(MCP_GPINTENB, 0xFF);

    uint8_t dummy;
    mcp_read(MCP_INTCAPA, &dummy);  /* clear any pending interrupt */
    mcp_read(MCP_INTCAPB, &dummy);
}

/* Drain ISR queues and clear MCP23017 interrupt flags */
static void mcp_clear_events(void)
{
    uint32_t v;
    while (xQueueReceive(inta_queue, &v, 0) == pdTRUE) {}
    while (xQueueReceive(intb_queue, &v, 0) == pdTRUE) {}
    uint8_t dummy;
    mcp_read(MCP_INTCAPA, &dummy);
    mcp_read(MCP_INTCAPB, &dummy);
}

/* Block until the given MCP23017 pin reads HIGH again (key released) */
static void wait_for_release(uint8_t port, uint8_t bit)
{
    uint8_t reg = (port == 0) ? MCP_GPIOA : MCP_GPIOB;
    uint8_t val;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        mcp_read(reg, &val);
    } while (!(val & (1 << bit)));
}

/* Wait for a specific key to be PRESSED (active-LOW).
 * Returns true once the key is pressed, false on timeout.
 * Pressing R_2 instead aborts early and sets *skipped = true
 * (unless the key being waited for IS R_2 itself). */
static bool wait_for_key(const KeyDef *k, uint32_t timeout_ms, bool *skipped)
{
    if (skipped) *skipped = false;
    mcp_clear_events();
    uint32_t v;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        bool got_event = (xQueueReceive(inta_queue, &v, pdMS_TO_TICKS(20)) == pdTRUE) ||
                          (xQueueReceive(intb_queue, &v, pdMS_TO_TICKS(1))  == pdTRUE);
        if (!got_event) continue;

        vTaskDelay(pdMS_TO_TICKS(20)); /* debounce */
        while (xQueueReceive(inta_queue, &v, 0) == pdTRUE) {}
        while (xQueueReceive(intb_queue, &v, 0) == pdTRUE) {}

        uint8_t gpioa, gpiob, dummy;
        mcp_read(MCP_GPIOA, &gpioa);
        mcp_read(MCP_GPIOB, &gpiob);
        mcp_read(MCP_INTCAPA, &dummy); /* clear INT flags */
        mcp_read(MCP_INTCAPB, &dummy);

        /* R_2 = skip, unless it's the key we're actually waiting for */
        bool target_is_r2 = (k->port == R2_PORT && k->bit == R2_BIT);
        if (!target_is_r2 && !(gpiob & (1 << R2_BIT))) {
            if (skipped) *skipped = true;
            wait_for_release(R2_PORT, R2_BIT);
            mcp_clear_events();
            return false;
        }

        /* Did the target key go LOW (pressed)? */
        uint8_t gpio_val = (k->port == 0) ? gpioa : gpiob;
        if (!(gpio_val & (1 << k->bit))) {
            vTaskDelay(pdMS_TO_TICKS(50));
            wait_for_release(k->port, k->bit);
            mcp_clear_events();
            return true;
        }
    }
    return false; /* timeout */
}

/* =========================================================
 * OLED helpers for the test screens
 * ========================================================= */

/* Header shown at the top of every test screen:
 *   Line 0: "TEST N/M"
 *   Line 1: test name
 *   Line 2: separator
 *   Lines 3-6: status area (cleared, filled in by oled_test_status)
 *   Line 7: "[R2]=Skip" hint */
static void oled_test_header(int n, int total, const char *name)
{
    oled_clear();
    oled_printf(0, 0, "TEST %d/%d", n, total);
    oled_printf(1, 0, "%.21s", name);
    oled_hline(2);
    oled_printf(7, 0, "[R2]=Skip");
    oled_flush();
}

typedef enum { STATUS_RUNNING, STATUS_OK, STATUS_FAIL, STATUS_SKIP } TestStatus;

/* Update the status area (lines 3-5) with the test result */
static void oled_test_status(TestStatus s, const char *detail)
{
    oled_printf(3, 0, "%-21s", "");
    oled_printf(4, 0, "%-21s", "");
    oled_printf(5, 0, "%-21s", "");
    switch (s) {
        case STATUS_RUNNING: oled_printf(3, 0, ">> Running..."); break;
        case STATUS_OK:      oled_printf(3, 0, ">> OK  :)");     break;
        case STATUS_FAIL:    oled_printf(3, 0, ">> FAIL :(");    break;
        case STATUS_SKIP:    oled_printf(3, 0, ">> Skipped");    break;
    }
    if (detail && *detail) oled_printf(4, 0, "%.21s", detail);
    oled_flush();
}

/* =========================================================
 * ╔══════════════════════════════════════╗
 * ║        SELF-TEST FUNCTIONS           ║
 * ╚══════════════════════════════════════╝
 * Each test returns true on pass, false on fail.
 * *skipped is set to true if R_2 was pressed to skip it.
 * ========================================================= */

/* ----------------------------------------------------------
 * T0: Keypad walk — ask the user to press every key in turn
 * ---------------------------------------------------------- */
static bool test_keypad_walk(bool *skipped)
{
    *skipped = false;
    int passed = 0;

    for (int i = 0; i < (int)N_KEYS; i++) {
        const KeyDef *k = &ALL_KEYS[i];

        oled_clear();
        oled_printf(0, 0, "KEYPAD WALK %d/%d", i + 1, (int)N_KEYS);
        oled_hline(1);
        oled_printf(3, 0, "Press:");
        oled_printf(4, 16, ">> %s <<", k->name);
        oled_printf(7, 0, "[R2]=Skip all");
        oled_flush();

        printf("[KEYPAD WALK %d/%d] Press %s\n", i + 1, (int)N_KEYS, k->name);

        bool key_skipped = false;
        bool ok = wait_for_key(k, 30000, &key_skipped);

        if (key_skipped) {
            printf("  -> Skipped\n");
            oled_printf(5, 0, "Skipped");
            oled_flush();
            vTaskDelay(pdMS_TO_TICKS(300));
            *skipped = true;
            break; /* skip the remaining keys too */
        } else if (ok) {
            passed++;
            printf("  -> OK\n");
            oled_printf(5, 0, "OK :)");
            oled_flush();
            vTaskDelay(pdMS_TO_TICKS(400));
        } else {
            printf("  -> TIMEOUT\n");
            oled_printf(5, 0, "TIMEOUT");
            oled_flush();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    if (*skipped) return false;

    char buf[22];
    snprintf(buf, sizeof(buf), "%d/%d passed", passed, (int)N_KEYS);
    oled_clear();
    oled_printf(0, 0, "KEYPAD WALK");
    oled_hline(1);
    oled_printf(3, 0, "%s", buf);
    oled_printf(4, 0, passed == (int)N_KEYS ? "ALL OK :)" : "Some FAILED");
    oled_flush();
    printf("[KEYPAD WALK] Result: %s\n", buf);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return (passed == (int)N_KEYS);
}

/* ----------------------------------------------------------
 * T1: Buzzer
 * ---------------------------------------------------------- */
static bool test_buzzer(bool *skipped)
{
    *skipped = false;
    gpio_set_level(PIN_BUZZER, 1);   /* 1 = OFF (active-LOW buzzer) */
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_BUZZER, 0);   /* 0 = ON */
    printf("[BUZZER] ON for 1s\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(PIN_BUZZER, 1);   /* back OFF */
    printf("[BUZZER] OFF\n");
    return true; /* no automated pass/fail — the user has to hear it */
}

/* ----------------------------------------------------------
 * Battery ADC
 * ---------------------------------------------------------- */
void initBatteryADC(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VBAT_SENSE_EN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(VBAT_SENSE_EN, 0);

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VBAT_SENSE_CHANNEL, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .chan     = VBAT_SENSE_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_calibration = true;
    }
}

/* Reads the battery voltage through the 27k/47k divider, averaging
 * 16 samples. Returns the voltage already scaled back up to LiPo level. */
float readBatteryVoltage(void)
{
    gpio_set_level(VBAT_SENSE_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    const int samples = 16;
    int raw, sum = 0;
    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(adc_handle, VBAT_SENSE_CHANNEL, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gpio_set_level(VBAT_SENSE_EN, 0);
    raw = sum / samples;

    int voltage_mv;
    if (do_calibration) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = (raw * 3300) / 4095;
    }
    float adc_voltage = voltage_mv / 1000.0f;

    /* Divider: 27k / 47k -> scale ADC reading back up to actual battery voltage */
    float battery_voltage = adc_voltage * ((27.0f + 47.0f) / 47.0f);
    return battery_voltage;
}

/* ----------------------------------------------------------
 * T2: VBAT ADC — also prints the measured voltage on the OLED
 * ---------------------------------------------------------- */
static bool test_vbat(bool *skipped, char *result_buf, size_t buf_size)
{
    *skipped = false;
    float vbat = readBatteryVoltage();
    printf("[VBAT] %.2f V\n", vbat);
    snprintf(result_buf, buf_size, "%.2f V", vbat);
    return (vbat > 3.0f && vbat < 4.3f); /* plausible LiPo range */
}

/* ----------------------------------------------------------
 * T3: I2C scan + MCP23017 presence
 * ---------------------------------------------------------- */
static bool test_i2c(bool *skipped, char *result_buf, size_t buf_sz)
{
    *skipped = false;
    int found = 0;
    bool mcp_found = false;
    bool oled_found = false;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            found++;
            printf("[I2C] 0x%02X %s\n", addr,
                   addr == MCP23017_ADDR ? "<MCP23017>" :
                   addr == OLED_ADDR     ? "<OLED>"     : "");
            if (addr == MCP23017_ADDR) mcp_found = true;
            if (addr == OLED_ADDR)     oled_found = true;
        }
    }
    snprintf(result_buf, buf_sz, "%d dev MCP:%s OLED:%s",
             found, mcp_found ? "OK" : "NO", oled_found ? "OK" : "NO");
    return mcp_found; /* OLED is already running, so MCP is the critical one */
}


/* ----------------------------------------------------------
 * T4: SX1262 SPI GetStatus
 * ---------------------------------------------------------- */
static bool test_sx1262(bool *skipped, char *result_buf, size_t buf_sz)
{
    *skipped = false;

    gpio_set_level(PIN_SX_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_SX_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    int timeout = 5000;
    while (gpio_get_level(PIN_SX_BUSY) && --timeout) vTaskDelay(pdMS_TO_TICKS(1));
    if (!timeout) {
        snprintf(result_buf, buf_sz, "BUSY timeout");
        return false;
    }

    uint8_t tx[2] = { SX1262_CMD_GET_STATUS, SX1262_NOP };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_transmit(sx_spi, &t);
    if (err != ESP_OK) {
        snprintf(result_buf, buf_sz, "SPI err %s", esp_err_to_name(err));
        return false;
    }

    uint8_t status   = rx[1];
    uint8_t chipmode = (status >> 4) & 0x07;
    snprintf(result_buf, buf_sz, "St=0x%02X ChipMode=%d", status, chipmode);
    printf("[SX1262] status=0x%02X chipMode=%d\n", status, chipmode);

    /* chipMode 2=STBY_RC, 3=STBY_XOSC — both are valid right after reset */
    return (chipmode == 2 || chipmode == 3);
}

/* ----------------------------------------------------------
 * T5: INTA / INTB — press any key on each port
 * ---------------------------------------------------------- */
static bool test_interrupts(bool *skipped, char *result_buf, size_t buf_sz)
{
    *skipped = false;
    mcp_clear_events();

    oled_printf(5, 0, "Press any Port-A key");
    oled_flush();
    printf("[INT] Waiting for INTA (Port A key)...\n");

    uint32_t v;
    if (xQueueReceive(inta_queue, &v, pdMS_TO_TICKS(10000)) != pdTRUE) {
        snprintf(result_buf, buf_sz, "INTA timeout");
        mcp_clear_events();
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t fa, ca, ga;
    mcp_read(MCP_INTFA, &fa);
    mcp_read(MCP_INTCAPA, &ca);
    mcp_read(MCP_GPIOA, &ga);
    printf("[INT] INTA fired INTFA=0x%02X INTCAPA=0x%02X\n", fa, ca);

    oled_printf(5, 0, "INTA OK! Now Port-B");
    oled_flush();
    mcp_clear_events();

    if (xQueueReceive(intb_queue, &v, pdMS_TO_TICKS(10000)) != pdTRUE) {
        snprintf(result_buf, buf_sz, "INTA OK, INTB TO");
        mcp_clear_events();
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t fb, cb, gb;
    mcp_read(MCP_INTFB, &fb);
    mcp_read(MCP_INTCAPB, &cb);
    mcp_read(MCP_GPIOB, &gb);
    printf("[INT] INTB fired INTFB=0x%02X INTCAPB=0x%02X\n", fb, cb);

    snprintf(result_buf, buf_sz, "A:0x%02X B:0x%02X OK", fa, fb);
    mcp_clear_events();
    return true;
}

/* =========================================================
 * ╔══════════════════════════════════════╗
 * ║         SELF-TEST RUNNER             ║
 * ╚══════════════════════════════════════╝
 * ========================================================= */

typedef struct {
    const char *name;
    bool        has_detail; /* show the result string on the OLED */
} TestEntry;

static const TestEntry TESTS[] = {
    { "Buzzer",         false },
    { "VBAT ADC",       true  },
    { "I2C Scan + MCP", true  },
    { "Keypad Walk",    false },    
    { "SX1262 SPI",     true  },
    { "INT A + B",      true  },
};
#define N_TESTS (sizeof(TESTS) / sizeof(TESTS[0]))


/* Wait TEST_INTER_DELAY_MS between tests, or skip immediately if R_2 is pressed */
static void wait_or_skip_to_next_test(void)
{
    oled_printf(7, 0, "[R2]=Next  wait %ds", TEST_INTER_DELAY_MS / 1000);
    oled_flush();

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(TEST_INTER_DELAY_MS);
    while (xTaskGetTickCount() < deadline) {
        uint32_t v;
        bool got = (xQueueReceive(inta_queue, &v, pdMS_TO_TICKS(50)) == pdTRUE) ||
                   (xQueueReceive(intb_queue, &v, pdMS_TO_TICKS(1))  == pdTRUE);
        if (!got) continue;

        vTaskDelay(pdMS_TO_TICKS(20)); /* debounce */
        while (xQueueReceive(inta_queue, &v, 0) == pdTRUE) {}
        while (xQueueReceive(intb_queue, &v, 0) == pdTRUE) {}

        uint8_t gb;
        mcp_read(MCP_GPIOB, &gb);
        if (!(gb & (1 << R2_BIT))) {
            wait_for_release(R2_PORT, R2_BIT);
            mcp_clear_events();
            return;
        }
    }
}

static void run_self_tests(void)
{
    int pass = 0, fail = 0, skip = 0;

    for (int t = 0; t < (int)N_TESTS; t++) {
        const char *name = TESTS[t].name;
        bool skipped = false;
        bool ok = false;
        char detail[22] = {0};

        oled_test_header(t + 1, (int)N_TESTS, name);
        oled_test_status(STATUS_RUNNING, NULL);
        printf("\n========================================\n");
        printf("[TEST %d/%d] %s\n", t + 1, (int)N_TESTS, name);
        printf("========================================\n");

        switch (t) {
            case 0: ok = test_buzzer(&skipped); break;
            case 1: ok = test_vbat(&skipped, detail, sizeof(detail)); break;
            case 2: ok = test_i2c(&skipped, detail, sizeof(detail)); break;
            case 3: ok = test_keypad_walk(&skipped); break;
            case 4: ok = test_sx1262(&skipped, detail, sizeof(detail)); break;
            case 5: 
                oled_printf(4, 0, "Trigger interrupt");
                oled_flush(); 
                ok = test_interrupts(&skipped, detail, sizeof(detail)); 
                break;
        }

        if (skipped) {
            oled_test_status(STATUS_SKIP, NULL);
            skip++;
            printf("  Result: SKIPPED\n");
        } else if (ok) {
            oled_test_status(STATUS_OK, TESTS[t].has_detail ? detail : NULL);
            pass++;
            printf("  Result: PASS  %s\n", detail);
        } else {
            oled_test_status(STATUS_FAIL, TESTS[t].has_detail ? detail : NULL);
            fail++;
            printf("  Result: FAIL  %s\n", detail);
        }

        if (!skipped) wait_or_skip_to_next_test();
    }

    printf("\n+--------------------------------------------+\n");
    printf("|             TEST SUMMARY                    |\n");
    printf("|  PASS: %2d   FAIL: %2d   SKIP: %2d              |\n", pass, fail, skip);
    printf("+--------------------------------------------+\n");

    oled_clear();
    oled_printf(0, 0, "=== SUMMARY ===");
    oled_hline(1);
    oled_printf(3, 0, "PASS: %d", pass);
    oled_printf(4, 0, "FAIL: %d", fail);
    oled_printf(5, 0, "SKIP: %d", skip);
    oled_printf(7, 0, fail == 0 ? "ALL GOOD :)" : "Check UART log");
    oled_flush();
}

/* =========================================================
 * Peripheral init
 * ========================================================= */
static void init_gpio(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_BUZZER) | (1ULL << PIN_VBAT_SENSE_EN) |
                        (1ULL << PIN_SX_RST)  | (1ULL << PIN_SX_RF_SW1),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_BUZZER,        1); /* 1 = OFF, active-LOW */
    gpio_set_level(PIN_VBAT_SENSE_EN, 0);
    gpio_set_level(PIN_SX_RST,        1);
    gpio_set_level(PIN_SX_RF_SW1,     0);

    gpio_config_t sx_in = {
        .pin_bit_mask = (1ULL << PIN_SX_BUSY) | (1ULL << PIN_SX_DIO1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sx_in));

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << PIN_K_INTA) | (1ULL << PIN_K_INTB),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));

    inta_queue = xQueueCreate(8, sizeof(uint32_t));
    intb_queue = xQueueCreate(8, sizeof(uint32_t));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_K_INTA, isr_inta, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_K_INTB, isr_intb, NULL));
    ESP_LOGI(TAG, "GPIO OK");
}

static void init_i2c(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C OK (SDA=GPIO%d SCL=GPIO%d)", PIN_I2C_SDA, PIN_I2C_SCL);
}

static void init_spi(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_SX_MOSI,
        .miso_io_num     = PIN_SX_MISO,
        .sclk_io_num     = PIN_SX_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 256,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_SX_NSS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &dev, &sx_spi));
    ESP_LOGI(TAG, "SPI OK");
}

/* =========================================================
 * app_main
 * ========================================================= */
void app_main(void)
{
    printf("\n");
    printf("+--------------------------------------------+\n");
    printf("|       RetroMeshDevice -- Boot               |\n");
    printf("|  XIAO ESP32-S3 Plus + SX1262 + MCP23017     |\n");
    printf("|  OLED SH1106 128x64                         |\n");
    printf("+--------------------------------------------+\n\n");
    init_gpio();
    init_i2c();
    init_spi();
    initBatteryADC();
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Bring up the OLED first so the user sees something immediately */
    oled_init();
    oled_clear();
    oled_printf(0, 0, "RetroMeshDevice");
    oled_hline(1);
    oled_printf(3, 0, "Initializing...");
    oled_flush();
    vTaskDelay(pdMS_TO_TICKS(500));

    

    mcp_setup();
    ESP_LOGI(TAG, "MCP23017 configured");


    run_self_tests();


}