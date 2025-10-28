//----------------------------------------SLAVE----------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

#define LED_GPIO     GPIO_NUM_2
#define SLAVE_ID     1       // unique ID for this slave (0..NUM_SLAVES-1)
#define NUM_SLAVES   2       // total devices including master

#define IRQ_RX_DONE  0x40
#define IRQ_ALL      0xFF

static const char *TAG = "LORA_SLAVE_SYNC";
static QueueHandle_t dio0_q;
static volatile uint32_t master_timestamp = 0;

// Minimal DIO0 ISR: just notify queue
static void IRAM_ATTR dio0_isr(void *arg) {
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    xQueueSendFromISR(dio0_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

// LED task synchronized with master timestamp
void led_task(void *arg) {
    while (true) {
        // LED ON only if timestamp modulo NUM_SLAVES equals this slave's ID
        if ((master_timestamp % NUM_SLAVES) == SLAVE_ID)
            gpio_set_level(LED_GPIO, 1);
        else
            gpio_set_level(LED_GPIO, 0);

        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms refresh
    }
}

void app_main(void)
{
    // Initialize LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // Initialize LoRa
    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    // Create DIO0 queue
    dio0_q = xQueueCreate(4, sizeof(uint8_t));

    // Install ISR for DIO0
    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    // Start LED task
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

    uint8_t buf[32];

    while (true) {
        uint8_t marker;
        if (xQueueReceive(dio0_q, &marker, portMAX_DELAY)) {
            int len = lora_read_reg(REG_RX_NB_BYTES);
            if (len > 0 && len < sizeof(buf)) {
                uint8_t fifo_addr = lora_read_reg(REG_FIFO_RX_CURRENT);
                lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);
                for (int i = 0; i < len; i++)
                    buf[i] = lora_read_reg(REG_FIFO);
                buf[len] = '\0';

                // Read SNR and RSSI
                int8_t snr = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE);
                uint8_t rssi = lora_read_reg(REG_PKT_RSSI_VALUE);
                int rssi_dbm = -157 + rssi; // approximate for 433 MHz SX1278

                // Synchronize timestamp
                master_timestamp = atoi((char*)buf);

                ESP_LOGI(TAG, "RX timestamp: %lu, SNR=%d, RSSI=%d dBm",
                         (unsigned long)master_timestamp, (int)snr, rssi_dbm);
            } else {
                ESP_LOGW(TAG, "Invalid RX length: %d", len);
            }

            // Clear IRQ flags
            lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
        }
    }
}















/*


//----------------------------------MASTER---------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

#define LED_GPIO     GPIO_NUM_2
#define MASTER_ID    0       // master is device 0
#define NUM_SLAVES   2       // total devices including master

#define IRQ_TX_DONE  0x08
#define IRQ_ALL      0xFF

static const char *TAG = "LORA_MASTER_SYNC";

void send_timestamp(uint32_t timestamp) {
    char msg[16];
    snprintf(msg, sizeof(msg), "%lu", (unsigned long)timestamp);

    // Standby mode before TX
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
    lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE_ADDR));

    // Write payload to FIFO
    for (int i = 0; msg[i]; i++)
        lora_write_reg(REG_FIFO, msg[i]);
    lora_write_reg(REG_PAYLOAD_LENGTH, strlen(msg));

    // Start TX
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void app_main(void)
{
    // Initialize LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // Initialize LoRa
    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_PA_CONFIG, 0x8F);

    uint32_t timestamp = 0;
    TickType_t last_tick = xTaskGetTickCount();

    while (true) {
        // Wait 1 second interval
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(1000));
        last_tick = xTaskGetTickCount();

        // Broadcast timestamp
        send_timestamp(timestamp);
        ESP_LOGI(TAG, "TX timestamp: %lu", (unsigned long)timestamp);

        // Poll TX_DONE
        int timeout = 200; // ms
        while (!(lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) && timeout--) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);

        // LED ON/OFF according to master ID
        if ((timestamp % NUM_SLAVES) == MASTER_ID)
            gpio_set_level(LED_GPIO, 1);
        else
            gpio_set_level(LED_GPIO, 0);

        timestamp++;
    }
}

*/


































//----------------------------------------NEW 0-----------------------------------
/*
//1️⃣ Master — low-power, timestamp broadcast
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

#define LED_GPIO   GPIO_NUM_2
#define IRQ_TX_DONE 0x08
#define IRQ_ALL     0xFF

static const char *TAG = "LORA_MASTER_SYNC";
static QueueHandle_t dio0_q;

void IRAM_ATTR dio0_isr(void *arg) {
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    xQueueSendFromISR(dio0_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

void send_timestamp(uint32_t timestamp) {
    char msg[16];
    snprintf(msg, sizeof(msg), "%lu", (unsigned long)timestamp);

    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
    lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE_ADDR));
    for (int i = 0; msg[i]; i++)
        lora_write_reg(REG_FIFO, msg[i]);
    lora_write_reg(REG_PAYLOAD_LENGTH, strlen(msg));
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    gpio_set_level(LED_GPIO, 1);
}

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_PA_CONFIG, 0x8F);

    dio0_q = xQueueCreate(8, sizeof(uint8_t));

    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    uint32_t timestamp = 0;
    char buf[64];
    TickType_t last_tick = xTaskGetTickCount();

    while (true) {
        // Wait for 1-second interval
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(1000));
        last_tick = xTaskGetTickCount();

        send_timestamp(timestamp);
        ESP_LOGI(TAG, "TX timestamp: %lu", (unsigned long)timestamp);

        // Poll TX_DONE
        uint8_t marker;
        if (xQueueReceive(dio0_q, &marker, pdMS_TO_TICKS(500)) == pdTRUE) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            if (flags & IRQ_TX_DONE) {
                ESP_LOGI(TAG, "TX_DONE #%lu", (unsigned long)timestamp);
            }
            lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
        } else {
            ESP_LOGW(TAG, "TX timeout #%lu", (unsigned long)timestamp);
        }
        gpio_set_level(LED_GPIO, 0);

        // RX window for ACKs (optional)
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
        vTaskDelay(pdMS_TO_TICKS(100));
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

        timestamp++;
    }
}














#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

#define LED_GPIO    GPIO_NUM_2
#define SLAVE_ID    0   // each slave: 0 .. NUM_SLAVES-1
#define NUM_SLAVES  2   // total number of slaves

#define IRQ_RX_DONE 0x40
#define IRQ_ALL     0xFF

static QueueHandle_t dio0_q;
static volatile uint32_t master_timestamp = 0;

static void IRAM_ATTR dio0_isr(void *arg) {
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    xQueueSendFromISR(dio0_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

// LED task synchronized with master timestamp
void led_task(void *arg) {
    while (true) {
        if (master_timestamp == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Sleep until next 1-second tick
        vTaskDelay(pdMS_TO_TICKS(1000));

        // LED ON only if current timestamp matches slave ID modulo NUM_SLAVES
        if ((master_timestamp % NUM_SLAVES) == SLAVE_ID)
            gpio_set_level(LED_GPIO, 1);
        else
            gpio_set_level(LED_GPIO, 0);

        master_timestamp++;
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    if (lora_init() != ESP_OK) {
        ESP_LOGE("LORA", "init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    dio0_q = xQueueCreate(4, sizeof(uint8_t));

    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

    uint8_t buf[32];

    while (true) {
        uint8_t marker;
        if (xQueueReceive(dio0_q, &marker, portMAX_DELAY)) {
            int len = lora_read_reg(REG_RX_NB_BYTES);
            if (len > 0 && len < sizeof(buf)) {
                uint8_t fifo_addr = lora_read_reg(REG_FIFO_RX_CURRENT);
                lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);
                for (int i = 0; i < len; i++)
                    buf[i] = lora_read_reg(REG_FIFO);
                buf[len] = '\0';
                master_timestamp = atoi((char*)buf); // synchronize timestamp
            }
            lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
        }
    }
}












*/
































/*

//--------------------------------------------------------------------------//      OLD   --------------------------------------------------------------------------
// lora_tx_hello_isr.c improved with TX ISR
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG = "LORA_TX_ISR";
static QueueHandle_t tx_done_q;

// --- IRQ Flags ---
#define IRQ_RX_TIMEOUT      0x80
#define IRQ_RX_DONE         0x40
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_VALID_HEADER    0x10
#define IRQ_TX_DONE         0x08
#define IRQ_ALL             0xFF

// --- ISR ---
static void IRAM_ATTR dio0_isr(void *arg)
{
    uint8_t marker = 1;
    BaseType_t xHigher = pdFALSE;
    xQueueSendFromISR(tx_done_q, &marker, &xHigher);
    if (xHigher) portYIELD_FROM_ISR();
}

void app_main(void)
{
    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);

    // Map DIO0 to TxDone (default in SX127x)
    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);

    tx_done_q = xQueueCreate(4, sizeof(uint8_t));
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    ESP_LOGI(TAG, "TX ISR ready on GPIO%d", LORA_DIO0);

    uint32_t counter = 0;
    char payload[64];

    for (;;) {
        int len = snprintf(payload, sizeof(payload), "Hello World #%u", (unsigned)counter);
        if (len <= 0) len = 0;
        if (len >= (int)sizeof(payload)) len = (int)sizeof(payload)-1;

        // Standby, reset FIFO, clear IRQ
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
        vTaskDelay(pdMS_TO_TICKS(2));
        lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_TX_BASE_ADDR));
        lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);

        // Write payload
        for (int i = 0; i < len; ++i) lora_write_reg(REG_FIFO, (uint8_t)payload[i]);
        lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);
        lora_write_reg(REG_PA_CONFIG, 0x8F);  // PA_BOOST

        // Start TX
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
        ESP_LOGI(TAG, "TX start: \"%s\"", payload);

        // Wait for TX_DONE via ISR
        uint8_t marker;
        if (xQueueReceive(tx_done_q, &marker, pdMS_TO_TICKS(1000)) == pdTRUE) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            if (flags & IRQ_TX_DONE) {
                ESP_LOGI(TAG, "TX_DONE #%u", counter);
            } else {
                ESP_LOGW(TAG, "Unexpected IRQ flags=0x%02X", flags);
            }
            lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
        } else {
            ESP_LOGW(TAG, "TX timeout #%u", counter);
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


































//-----------------🟢 lora_rx_printstring.c (improved)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lora.h"

static const char *TAG = "LORA_RX_ISR";
static QueueHandle_t dio0_q;

// --- IRQ Flags ---
#define IRQ_RX_TIMEOUT      0x80
#define IRQ_RX_DONE         0x40
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_VALID_HEADER    0x10
#define IRQ_TX_DONE         0x08
#define IRQ_ALL             0xFF

// --- ISR: trigger on DIO0 rising edge ---
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

    // Continuous RX
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    ESP_LOGI(TAG, "Receiver ready (Continuous RX)");

    for (;;) {
        if (xQueueReceive(dio0_q, &marker, portMAX_DELAY) == pdTRUE) {
            uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
            int8_t raw_snr = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE);
            float snr_db = raw_snr / 4.0f;

            uint8_t raw_rssi = lora_read_reg(REG_PKT_RSSI_VALUE);
            const int rssi_offset = -164; // for 433 MHz (LF port)
            float rssi_dbm = rssi_offset + raw_rssi;
            if (snr_db < 0.0f) rssi_dbm += snr_db;

            if (flags & IRQ_RX_DONE) {
                int len = lora_read_reg(REG_RX_NB_BYTES);
                if (len > 0 && len < (int)sizeof(buf)) {
                    uint8_t fifo_cur = lora_read_reg(REG_FIFO_RX_CURRENT);
                    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_cur);

                    for (int i = 0; i < len; ++i)
                        buf[i] = lora_read_reg(REG_FIFO);
                    buf[len] = '\0';

                    rx_count++;
                    if (flags & IRQ_PAYLOAD_CRC_ERR) {
                        crc_errors++;
                        ESP_LOGW(TAG,
                            "CRC ERR #%u len=%d flags=0x%02X SNR=%.2f dB RSSI=%.1f dBm",
                            crc_errors, len, flags, snr_db, rssi_dbm);
                    } else {
                        ESP_LOGI(TAG,
                            "RX #%u len=%d flags=0x%02X SNR=%.2f dB RSSI=%.1f dBm payload=\"%s\"",
                            rx_count, len, flags, snr_db, rssi_dbm, (char*)buf);
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid RX length %d flags=0x%02X", len, flags);
                }
            } else if (flags & IRQ_RX_TIMEOUT) {
                timeouts++;
                ESP_LOGW(TAG, "RX_TIMEOUT #%u flags=0x%02X", timeouts, flags);
            } else {
                ESP_LOGW(TAG, "IRQ not RX_DONE flags=0x%02X", flags);
            }

            // Clear IRQ and keep listening
            lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
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

    // Queue for DIO0 ISR
    dio0_q = xQueueCreate(8, sizeof(uint8_t));
    if (!dio0_q) {
        ESP_LOGE(TAG, "Failed to create DIO0 queue");
        return;
    }

    if (lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    lora_set_frequency(433000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // DIO0 pin setup
    gpio_install_isr_service(0);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO0, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO0, dio0_isr, NULL);

    ESP_LOGI(TAG, "DIO0 ISR ready on GPIO%d", LORA_DIO0);

    xTaskCreatePinnedToCore(lora_rx_task, "lora_rx_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}




















*/


//--------------------------------------------------------------------------//--------------------------------------------------------------------------




