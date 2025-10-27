// lora_tx_hello.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG = "LORA_TX_HELLO";

void app_main(void)
{
    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "lora_init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);

    uint32_t counter = 0;
    char payload[64];

    for (;;) {
        int len = snprintf(payload, sizeof(payload), "Hello World #%u", (unsigned)counter);
        if (len <= 0) len = 0;
        if (len >= (int)sizeof(payload)) len = (int)sizeof(payload)-1;

        // Standby, set FIFO pointer and clear IRQs
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
        vTaskDelay(pdMS_TO_TICKS(2));
        lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE_ADDR));
        lora_write_reg(REG_IRQ_FLAGS, 0xFF);

        // Write payload
        for (int i = 0; i < len; ++i) lora_write_reg(REG_FIFO, (uint8_t)payload[i]);
        lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);

        // PA config: PA_BOOST for most modules; switch to 0x4F if your board uses RFO
        lora_write_reg(REG_PA_CONFIG, 0x8F);

        // Start TX
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
        ESP_LOGI(TAG, "TX start: \"%s\"", payload);

        // Wait for TX_DONE
        int timeout = 1000;
        bool done = false;
        while (timeout-- > 0) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            if (flags & 0x08) {
                done = true;
                lora_write_reg(REG_IRQ_FLAGS, 0xFF);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (done) {
            ESP_LOGI(TAG, "TX_DONE");
        } else {
            ESP_LOGW(TAG, "TX timeout");
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}








/*
//----------------------------------------------------------------------------SLAVE---------------------------------------------------------------------------
// rx_minimal_fixed.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

static const char *TAG = "RX_MINIMAL_FIXED";
static QueueHandle_t dio0_q;

// Minimal IRAM ISR: only send a one-byte marker, non-blocking
static void IRAM_ATTR dio0_isr(void *arg)
{
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    // try to send; if the queue is full, drop the marker (avoid blocking in ISR)
    xQueueSendFromISR(dio0_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

// Replace your lora_rx_task with this compact variant
static void lora_rx_task(void *pv)
{
    uint8_t marker;
    uint8_t buf[256];
    uint32_t rx_count = 0;
    uint32_t crc_errors = 0;
    uint32_t timeouts = 0;

    // Prepare continuous RX once
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    ESP_LOGI(TAG, "Receiver ready (compact diagnostics)");

    for (;;) {
        // wait for marker from ISR (marker-only ISR required)
        if (xQueueReceive(dio0_q, &marker, portMAX_DELAY) == pdTRUE) {
            // read flags and quick diagnostics
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            int8_t snr = (int8_t)lora_read_reg(0x19);          // REG_PKT_SNR_VALUE
            uint8_t rssi = lora_read_reg(REG_PKT_RSSI_VALUE);  // REG_PKT_RSSI_VALUE

            if (flags & 0x40) { // RX_DONE
                int len = lora_read_reg(REG_RX_NB_BYTES);
                if (len > 0 && len <= (int)sizeof(buf)) {
                    uint8_t fifo_cur = lora_read_reg(REG_FIFO_RX_CURRENT);
                    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_cur);
                    for (int i = 0; i < len; ++i) buf[i] = lora_read_reg(REG_FIFO);

                    rx_count++;
                    if (flags & 0x20) { // CRC error
                        crc_errors++;
                        ESP_LOGW(TAG, "CRC ERR #%u len=%d flags=0x%02X SNR=%d RSSI=%d", crc_errors, len, flags, (int)snr, (int)rssi);
                    } else {
                        ESP_LOGI(TAG, "RX #%u len=%d flags=0x%02X SNR=%d RSSI=%d first=%02X",
                                 rx_count, len, flags, (int)snr, (int)rssi, buf[0]);
                    }
                } else {
                    ESP_LOGW(TAG, "Bad len=%d flags=0x%02X", len, flags);
                }
            } else {
                // Not RX_DONE: handle timeout or other IRQs
                if (flags & 0x10) { // RX_TIMEOUT
                    timeouts++;
                    ESP_LOGW(TAG, "RX_TIMEOUT #%u flags=0x%02X SNR=%d RSSI=%d", timeouts, flags, (int)snr, (int)rssi);
                } else {
                    ESP_LOGW(TAG, "IRQ not RX_DONE flags=0x%02X SNR=%d RSSI=%d", flags, (int)snr, (int)rssi);
                }
            }

            // Clear IRQs and re-enter RX continuous
            lora_write_reg(REG_IRQ_FLAGS, 0xFF);
            lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
        }
    }
}


void app_main(void)
{
    // Reset radio
    gpio_reset_pin(LORA_RST);
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Create queue BEFORE installing ISR
    dio0_q = xQueueCreate(8, sizeof(uint8_t));
    if (!dio0_q) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "lora_init failed");
        return;
    }

    lora_set_frequency(433000000);

    // Ensure FIFO base/pointer
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Configure DIO0 input + internal pull-down to prevent floating noise
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);

    // Install ISR service and add handler AFTER queue created and radio configured
    gpio_install_isr_service(0);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    // Start RX task - give it extra stack
    xTaskCreate(lora_rx_task, "rx_task", 6144, NULL, 5, NULL);
    ESP_LOGI(TAG, "DIO0 ISR installed on GPIO%d", LORA_DIO0);
    
}


*/





















/*
//master 2

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG = "LORA_TX_TEST";

void app_main(void)
{
    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "lora_init failed");
        return;
    }
    lora_set_frequency(433000000);

    // Ensure TX base
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    uint8_t payload = 0xA5;

    while (1) {
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
        vTaskDelay(pdMS_TO_TICKS(2));

        // set TX pointer and clean IRQs
        lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE_ADDR));
        lora_write_reg(REG_IRQ_FLAGS, 0xFF);

        lora_write_reg(REG_FIFO, payload);
        lora_write_reg(REG_PAYLOAD_LENGTH, 1);

        // Use PA_BOOST for most breakouts (change to 0x4F if board needs RFO)
        lora_write_reg(REG_PA_CONFIG, 0x8F);

        // Start TX
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
        ESP_LOGI(TAG, "TX started");

        // Poll for TX_DONE
        int timeout = 1000;
        while (timeout-- > 0) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            if (flags & 0x08) {
                ESP_LOGI(TAG, "TX_DONE flags=0x%02X", flags);
                lora_write_reg(REG_IRQ_FLAGS, 0xFF);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Dump a few regs for debug
        ESP_LOGI(TAG, "DBG: OP=0x%02X FRF=%02X%02X%02X PA=0x%02X IRQ=0x%02X",
                 lora_read_reg(REG_OP_MODE),
                 lora_read_reg(REG_FRF_MSB), lora_read_reg(REG_FRF_MID), lora_read_reg(REG_FRF_LSB),
                 lora_read_reg(REG_PA_CONFIG), lora_read_reg(REG_IRQ_FLAGS));

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

*/













































//----------------------------2
/*
// lora_rx_printstring.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

static const char *TAG = "LORA_RX_PRINT";
static QueueHandle_t dio0_q;

// Minimal ISR: only queue a marker (no SPI)
static void IRAM_ATTR dio0_isr(void *arg)
{
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    xQueueSendFromISR(dio0_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

static void lora_rx_task(void *pv)
{
    uint8_t marker;
    uint8_t buf[256];
    uint32_t rx_count = 0;
    uint32_t crc_errors = 0;
    uint32_t timeouts = 0;

    // Prepare RX continuous
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    ESP_LOGI(TAG, "Receiver ready");

    for (;;) {
        if (xQueueReceive(dio0_q, &marker, portMAX_DELAY) == pdTRUE) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            int8_t snr = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE);
            uint8_t rssi = lora_read_reg(REG_PKT_RSSI_VALUE);

            if (flags & 0x40) { // RX_DONE
                int len = lora_read_reg(REG_RX_NB_BYTES);
                if (len > 0 && len < (int)sizeof(buf)) {
                    uint8_t fifo_cur = lora_read_reg(REG_FIFO_RX_CURRENT);
                    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_cur);
                    for (int i = 0; i < len; ++i) buf[i] = lora_read_reg(REG_FIFO);
                    buf[len] = '\0';

                    rx_count++;
                    if (flags & 0x20) { // CRC error
                        crc_errors++;
                        ESP_LOGW(TAG, "CRC ERR #%u len=%d flags=0x%02X SNR=%d RSSI=%d",
                                 crc_errors, len, flags, (int)snr, (int)rssi);
                    } else {
                        ESP_LOGI(TAG, "RX #%u len=%d flags=0x%02X SNR=%d RSSI=%d payload=\"%s\"",
                                 rx_count, len, flags, (int)snr, (int)rssi, (char*)buf);
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid RX length %d flags=0x%02X", len, flags);
                }
            } else {
                if (flags & 0x10) {
                    timeouts++;
                    ESP_LOGW(TAG, "RX_TIMEOUT #%u flags=0x%02X SNR=%d RSSI=%d", timeouts, flags, (int)snr, (int)rssi);
                } else {
                    ESP_LOGW(TAG, "IRQ not RX_DONE flags=0x%02X SNR=%d RSSI=%d", flags, (int)snr, (int)rssi);
                }
            }

            // Clear IRQ flags and continue listening
            lora_write_reg(REG_IRQ_FLAGS, 0xFF);
            lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
        }
    }
}

void app_main(void)
{
    // Reset radio
    gpio_reset_pin(LORA_RST);
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Create queue and init radio
    dio0_q = xQueueCreate(8, sizeof(uint8_t));
    if (!dio0_q) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "lora_init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Configure DIO0 pin with internal pull-down and install ISR
    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    xTaskCreatePinnedToCore(lora_rx_task, "lora_rx_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "DIO0 ISR installed on GPIO%d", LORA_DIO0);
}





*/



