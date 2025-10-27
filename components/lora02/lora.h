#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Pin configuration (make these match your wiring) ---
#define LORA_MOSI   23
#define LORA_MISO   19
#define LORA_SCK    18
#define LORA_CS     5
#define LORA_RST    15   // match your app_main
#define LORA_DIO0   4   // match your app_main


// --- SX1278 registers ---
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_PKT_SNR_VALUE       0x19
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG_3      0x26
#define REG_FIFO_RX_CURRENT     0x10
#define REG_VERSION             0x42

// --- LoRa modes ---
#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03
#define MODE_RX_CONTINUOUS      0x05

// API
esp_err_t lora_init(void);
void lora_set_frequency(long freq);
void lora_send_packet(const uint8_t *data, int len);
void lora_enable_rx(void);
int lora_receive_packet(uint8_t *buf, int maxlen);

void lora_write_reg(uint8_t reg, uint8_t val);
uint8_t lora_read_reg(uint8_t reg);


#ifdef __cplusplus
}
#endif
