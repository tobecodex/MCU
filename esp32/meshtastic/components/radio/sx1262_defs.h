#pragma once

#include <stdint.h>

// SX1262 LoRa Radio Command Opcodes
// Based on Semtech SX1262 datasheet

// Operating Mode Commands
#define SX1262_CMD_SET_SLEEP              0x84
#define SX1262_CMD_SET_STANDBY            0x80
#define SX1262_CMD_SET_FS                 0xC1
#define SX1262_CMD_SET_TX                 0x83
#define SX1262_CMD_SET_RX                 0x82
#define SX1262_CMD_SET_RXDUTYCYCLE        0x94
#define SX1262_CMD_SET_CAD                0xC5
#define SX1262_CMD_SET_TXCONTINUOUSWAVE   0xD1
#define SX1262_CMD_SET_TXCONTINUOUSPREAMBLE 0xD2

// Configuration Commands
#define SX1262_CMD_SET_PACKETTYPE         0x8A
#define SX1262_CMD_GET_PACKETTYPE         0x11
#define SX1262_CMD_SET_RFFREQUENCY        0x86
#define SX1262_CMD_SET_TXPARAMS           0x8E
#define SX1262_CMD_SET_PACONFIG           0x95
#define SX1262_CMD_SET_CADPARAMS          0x88
#define SX1262_CMD_SET_BUFFERBASEADDRESS  0x8F
#define SX1262_CMD_SET_MODULATIONPARAMS   0x8B
#define SX1262_CMD_SET_PACKETPARAMS       0x8C

// Status Commands
#define SX1262_CMD_GET_RXBUFFERSTATUS     0x13
#define SX1262_CMD_GET_PACKETSTATUS       0x14
#define SX1262_CMD_GET_RSSIINST           0x15
#define SX1262_CMD_GET_STATS              0x10
#define SX1262_CMD_RESET_STATS            0x00
#define SX1262_CMD_GET_STATUS             0xC0
#define SX1262_CMD_GET_ERROR              0x17
#define SX1262_CMD_CLR_ERROR              0x07

// IRQ Commands
#define SX1262_CMD_CFG_DIOIRQ             0x08
#define SX1262_CMD_GET_IRQSTATUS          0x12
#define SX1262_CMD_CLR_IRQSTATUS          0x02

// Calibration Commands
#define SX1262_CMD_CALIBRATE              0x89
#define SX1262_CMD_CALIBRATEIMAGE         0x98

// Register/Buffer Commands
#define SX1262_CMD_WRITE_REGISTER         0x0D
#define SX1262_CMD_READ_REGISTER          0x1D
#define SX1262_CMD_WRITE_BUFFER           0x0E
#define SX1262_CMD_READ_BUFFER            0x1E

// Other Commands
#define SX1262_CMD_SET_REGULATORMODE      0x96
#define SX1262_CMD_SET_TCXOMODE           0x97
#define SX1262_CMD_SET_TXFALLBACKMODE     0x93
#define SX1262_CMD_SET_RFSWITCHMODE       0x9D
#define SX1262_CMD_SET_STOPRXTIMERONPREAMBLE 0x9F
#define SX1262_CMD_SET_LORASYMBTIMEOUT    0xA0

// Packet Types
#define SX1262_PACKET_TYPE_GFSK           0x00
#define SX1262_PACKET_TYPE_LORA           0x01

// Standby Modes
#define SX1262_STANDBY_RC                 0x00
#define SX1262_STANDBY_XOSC               0x01

// Regulator Modes
#define SX1262_REGULATOR_LDO              0x00
#define SX1262_REGULATOR_DC_DC            0x01

// IRQ Masks
#define SX1262_IRQ_TX_DONE                0x0001
#define SX1262_IRQ_RX_DONE                0x0002
#define SX1262_IRQ_PREAMBLE_DETECTED      0x0004
#define SX1262_IRQ_SYNC_WORD_VALID        0x0008
#define SX1262_IRQ_HEADER_VALID           0x0010
#define SX1262_IRQ_HEADER_ERROR           0x0020
#define SX1262_IRQ_CRC_ERROR              0x0040
#define SX1262_IRQ_CAD_DONE               0x0080
#define SX1262_IRQ_CAD_DETECTED           0x0100
#define SX1262_IRQ_TIMEOUT                0x0200
#define SX1262_IRQ_ALL                    0x03FF
