/**
 * @file pin_definitions.h
 * @brief GPIO pin definitions for RetroMeshDevice (XIAO ESP32-S3 Plus)
 *
 * Schematic: ESP32S3_PLUS_XIAO (U5)
 * Board: Seeed XIAO ESP32-S3 Plus
 */

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include "driver/gpio.h"
#include "driver/adc.h"

/* =========================================================
 * POWER CONTROL
 * ========================================================= */

/** 3V3 LDO enable — connected to ESP32-S3 EN pin (pin 26) via slide switch.
 *  HIGH = system on. Driven externally by TPS79533 3V3_EN line. */
// #define PIN_3V3_EN          GPIO_NUM_NC   // EN pin, not controllable from firmware

/** VBAT sense enable — MOSFET gate driving the 27K/47K resistor divider.
 *  HIGH = divider connected; LOW = divider floating (saves quiescent current). */
#define PIN_VBAT_SENSE_EN   GPIO_NUM_43   // D6 / GPIO43 / TX

/* =========================================================
 * BATTERY MEASUREMENT (ADC)
 * ========================================================= */

/** Battery voltage sense — midpoint of 27K/47K divider.
 *  Enable PIN_VBAT_SENSE_EN before reading.
 *  Scale factor: Vbat = Vadc * (27 + 47) / 47 */
#define PIN_VBAT_SENSE      GPIO_NUM_4    // D3 / GPIO4
#define ADC_VBAT_CHANNEL    ADC1_CHANNEL_3  // GPIO4 → ADC1_CH3

/* =========================================================
 * BUZZER
 * ========================================================= */

/** Active buzzer control (active-LOW).
 *  LOW (0) = buzzer ON; HIGH (1) = buzzer OFF. */
#define PIN_BUZZER          GPIO_NUM_3    // D2 / GPIO3

/* =========================================================
 * I2C BUS  (MCP23017 — address 0x20, A0/A1/A2 tied to GND)
 * ========================================================= */

#define PIN_I2C_SDA         GPIO_NUM_5    // D4 / GPIO5 / SDA
#define PIN_I2C_SCL         GPIO_NUM_6    // D5 / GPIO6 / SCL
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000        // 400 kHz Fast-mode

/** MCP23017 I2C address (A2=A1=A0=GND → 0b0100000 = 0x20) */
#define MCP23017_ADDR       0x20

/* =========================================================
 * MCP23017 INTERRUPT LINES
 * ========================================================= */

/** MCP23017 INTA — mirrors port A interrupt flag.
 *  Active-LOW; use GPIO pull-up + falling-edge trigger. */
#define PIN_K_INTA          GPIO_NUM_1    // D0 / GPIO1

/** MCP23017 INTB — mirrors port B interrupt flag.
 *  Active-LOW; use GPIO pull-up + falling-edge trigger. */
#define PIN_K_INTB          GPIO_NUM_2    // D1 / GPIO2

/* =========================================================
 * SPI BUS  (WIO SX1262 LoRa module)
 * ========================================================= */

#define PIN_SX_MOSI         GPIO_NUM_9    // D10 / GPIO9  / MOSI0
#define PIN_SX_MISO         GPIO_NUM_8    // D9  / GPIO8  / MISO0
#define PIN_SX_SCK          GPIO_NUM_7    // D8  / GPIO7  / SCK0
#define PIN_SX_NSS          GPIO_NUM_41   // D14 / GPIO41 (Chip Select, active-LOW)
#define SPI_HOST            SPI2_HOST
#define SPI_FREQ_HZ         8000000       // 8 MHz

/* =========================================================
 * SX1262 CONTROL / STATUS PINS
 * ========================================================= */

/** SX1262 RESET — active-LOW pulse to reset the radio. */
#define PIN_SX_RST          GPIO_NUM_42   // D15 / GPIO42

/** SX1262 BUSY — HIGH while the radio is processing a command.
 *  Poll LOW before issuing next SPI transaction. */
#define PIN_SX_BUSY         GPIO_NUM_40   // D13 / GPIO40

/** SX1262 DIO1 — IRQ line (TxDone, RxDone, etc.).
 *  Rising-edge triggered. */
#define PIN_SX_DIO1         GPIO_NUM_39   // D12 / GPIO39

/** SX1262 RF switch control (antenna path selection). */
#define PIN_SX_RF_SW1       GPIO_NUM_38   // D11 / GPIO38

/* =========================================================
 * TEST TIMING
 * ========================================================= */

/** Delay between test steps, in milliseconds. */
#define TEST_INTER_DELAY_MS  5000


#endif /* PIN_DEFINITIONS_H */