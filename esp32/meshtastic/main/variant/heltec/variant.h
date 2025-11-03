#pragma once

// Heltec WiFi LoRa 32 V3 (USB-C)
// Pin definitions verified for V3 hardware

// I2C (OLED Display - SSD1306)
#define I2C_SDA 17
#define I2C_SCL 18

// SPI (LoRa Radio - SX1262)
#define SPI_MOSI 10
#define SPI_MISO 11
#define SPI_CLK 9
#define LORA_CS 8
#define LORA_IRQ 14
#define LORA_RST 12
#define LORA_BUSY 13

// Built-in LED
#define LED_PIN 35

// Display configuration
#define OLED_I2C_ADDR 0x3c
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Radio type
#define RADIO_SX1262  // V3 uses SX1262, not SX127x

#define BOARD_NAME "Heltec V3"

#define BOARD_NAME "Heltec"