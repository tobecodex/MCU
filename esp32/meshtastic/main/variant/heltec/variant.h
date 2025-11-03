#pragma once

#include "driver/gpio.h"

// Heltec WiFi LoRa 32 V3 (USB-C)
// Pin definitions verified for V3 hardware

// I2C (OLED Display - SSD1306)
#define I2C_SDA (gpio_num_t)17
#define I2C_SCL (gpio_num_t)18
#define OLED_RST (gpio_num_t)21
#define VEXT_CTRL (gpio_num_t)36  // External power control (LOW=ON)

// SPI (LoRa Radio - SX1262)
#define SPI_MOSI (gpio_num_t)10
#define SPI_MISO (gpio_num_t)11
#define SPI_CLK (gpio_num_t)9
#define LORA_CS (gpio_num_t)8
#define LORA_IRQ (gpio_num_t)14
#define LORA_RST (gpio_num_t)12
#define LORA_BUSY (gpio_num_t)13

// Built-in LED
#define LED_PIN (gpio_num_t)35

// Display configuration
#define OLED_I2C_ADDR 0x3c
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Radio type
#define RADIO_SX1262  // V3 uses SX1262, not SX127x

#define BOARD_NAME "Heltec V3"