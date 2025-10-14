#include "lora.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "LORA"

// --- Pin configuration ---
#define LORA_MOSI 23
#define LORA_MISO 19
#define LORA_SCK  18
#define LORA_CS    5
#define LORA_RST  14
#define LORA_DIO0 26

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
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG_3      0x26
#define REG_FIFO_RX_CURRENT     0x10
#define REG_DIO_MAPPING_1       0x40
#define REG_VERSION             0x42

#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03
#define MODE_RX_CONTINUOUS      0x05

static spi_device_handle_t spi;

static void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t out[2] = { (uint8_t)(reg | 0x80), val };
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = out,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    spi_device_transmit(spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg) {
    uint8_t cmd = reg & 0x7F;
    uint8_t val = 0;
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = &cmd,
        .rx_buffer = &val,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    uint8_t tmp[2] = {cmd, 0x00};
    t.tx_buffer = tmp;
    uint8_t rx[2];
    t.rx_buffer = rx;
    spi_device_transmit(spi, &t);
    return rx[1];
}

static void lora_reset(void) {
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

esp_err_t lora_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO,
        .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    gpio_reset_pin(LORA_RST);
    lora_reset();

    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found (version=0x%02X)", version);
        return ESP_FAIL;
    }

    // Sleep then standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // Basic config: 125kHz BW, SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(REG_PA_CONFIG, 0x8F); // max power

    // Set base addresses
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    return ESP_OK;
}

void lora_set_frequency(long frequency) {
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_send_packet(const uint8_t *data, int len) {
    // Standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    // Set FIFO pointer
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);
    // Write payload
    for (int i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
    lora_write_reg(REG_PAYLOAD_LENGTH, len);
    // TX
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    // Wait for TX done
    while ((lora_read_reg(REG_IRQ_FLAGS) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    // Clear IRQ
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

void lora_enable_rx(void) {
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

int lora_receive_packet(uint8_t *buf, int maxlen) {
    int len = 0;
    if (lora_read_reg(REG_IRQ_FLAGS) & 0x40) { // RX_DONE
        len = lora_read_reg(REG_RX_NB_BYTES);
        if (len > maxlen) len = maxlen;
        lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT));
        for (int i = 0; i < len; i++) {
            buf[i] = lora_read_reg(REG_FIFO);
        }
        // Clear IRQ flags
        lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    }
    return len;
}
