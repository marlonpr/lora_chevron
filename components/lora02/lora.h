#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"   // <-- add this line


// Initialize LoRa radio (configure SPI and reset the SX1278)
esp_err_t lora_init(void);

// Set frequency in Hz (e.g., 433E6)
void lora_set_frequency(long frequency);

// Send a packet (blocking)
void lora_send_packet(const uint8_t *data, int len);

// Put the radio into continuous receive mode
void lora_enable_rx(void);

// Receive a packet (non-blocking)
// Returns number of bytes received, 0 if none
int lora_receive_packet(uint8_t *buf, int maxlen);
