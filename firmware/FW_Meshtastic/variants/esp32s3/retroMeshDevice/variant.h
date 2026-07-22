
/// =========================================================
// BATTERY
// =========================================================
#define ADC_CTRL 43 
#define ADC_CTRL_ENABLED HIGH
#define BATTERY_PIN 4 
#define ADC_CHANNEL ADC_CHANNEL_3
#define ADC_MULTIPLIER 1.57


/// =========================================================
// COMMS PROTOCOLS
// =========================================================
#define I2C_SDA         5              // GPIO5 / D4
#define I2C_SCL         6              // GPIO6 / D5

/// =========================================================
// OLED SCREEN
// =========================================================
#define USCREEN_SSD1306


// =========================================================
// SX1262 LoRa — WIO SX1262 module via SPI2
// =========================================================
#define USE_SX1262
 
#define LORA_MISO       8              // GPIO8  / D9
#define LORA_SCK        7              // GPIO7  / D8
#define LORA_MOSI       9              // GPIO9  / D10
#define LORA_CS         41             // GPIO41 / D14  (NSS, active-LOW)
#define LORA_RESET      42             // GPIO42 / D15
#define LORA_DIO1       39             // GPIO39 / D12  (IRQ)
#define LORA_DIO2       38             // GPIO38 / D11  (RF switch / RXEN)

#define SX126X_CS               LORA_CS
#define SX126X_DIO1             LORA_DIO1
#define SX126X_BUSY             40     // GPIO40 / D13
#define SX126X_RESET            LORA_RESET
 
// DIO2 drives the RF antenna switch (TX/RX path)
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_RXEN             38     // GPIO38 / D11 = SX_RF_SW1
#define SX126X_TXEN             RADIOLIB_NC
 
// TCXO voltage controlled by DIO3 (same as WIO SX1262 reference design)
#define SX126X_DIO3_TCXO_VOLTAGE       1.8


// =========================================================
// BUZZER (GPIO3 / D2)
// =========================================================
//#define PIN_BUZZER      3              // GPIO3 / D2


// =========================================================
// MCP23017 — T9 keypad expander, same I2C bus
// Address: 0x20 (A0=A1=A2=GND)
// INTA → GPIO1 (D0), INTB → GPIO2 (D1)
// =========================================================
#define HAS_KEYBOARD    1
//#define KB_I2C_ADDR     0x20

#define CUSTOM_MCP23017_MAP 

// Port A (Bits 0-7)
#define MCP23017_KEYMAP_0   7
#define MCP23017_KEYMAP_1   10
#define MCP23017_KEYMAP_2   9
#define MCP23017_KEYMAP_3   6
#define MCP23017_KEYMAP_4   3
#define MCP23017_KEYMAP_5   0
#define MCP23017_KEYMAP_6   1
#define MCP23017_KEYMAP_7   4

// Port B (Bits 8-15)
#define MCP23017_KEYMAP_8   12
#define MCP23017_KEYMAP_9   13
#define MCP23017_KEYMAP_10  14
#define MCP23017_KEYMAP_11  2
#define MCP23017_KEYMAP_12  5
#define MCP23017_KEYMAP_13  8
#define MCP23017_KEYMAP_14  11
#define MCP23017_KEYMAP_15  255

#define LONG_PRESS_THRESHOLD 1000
#define MULTI_TAP_THRESHOLD  2000