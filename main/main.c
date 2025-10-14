//               ----------------------------------------------MASTER------------------------------------------//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "lora.h"
#include <stdio.h>

#define LED_GPIO GPIO_NUM_2

void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    lora_init();
    lora_set_frequency(433E6);

    uint8_t state = 0;

    while (1) {
        // Light LED if it's our turn (state 0)
        gpio_set_level(LED_GPIO, state == 0);

        // Broadcast current state
        lora_send_packet(&state, 1);
        printf("Master sent state %u\n", state);

        // Next state (0→1→2→0…)
        state = (state + 1) % 3;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/*

		//--------------------------------------SLAVE--------------------------------------------------------//
		
	
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "lora.h"
#include <stdio.h>
#include <stdint.h>

#define LED_GPIO  GPIO_NUM_2
#define SLAVE_ID  1   // <<< 1 or 2 depending on board

void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    lora_init();
    lora_set_frequency(433E6);
    lora_enable_rx();

    uint8_t buf[4];

    while (1) {
        int len = lora_receive_packet(buf, sizeof(buf));
        if (len == 1) {
            uint8_t state = buf[0];
            printf("Slave %d got state %u\n", SLAVE_ID, state);

            // LED ON only when state matches our ID
            gpio_set_level(LED_GPIO, state == SLAVE_ID);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
*/

//---------------------------------------------------------------------------------------------------------------------------------





/*#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "lora.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LED_GPIO  GPIO_NUM_2   // onboard LED or external
#define SLAVE_ID  0            // change to 0 or 1 for each slave

void app_main(void)
{
    // Configure LED
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    if (lora_init() != ESP_OK) {
        printf("LoRa init failed!\n");
        return;
    }
    lora_set_frequency(433E6);
    lora_enable_rx();

    uint8_t buf[32];
    while (1) {
        int len = lora_receive_packet(buf, sizeof(buf) - 1);
        if (len > 0) {
            buf[len] = '\0';
            uint32_t ts = strtoul((char*)buf, NULL, 10);
            printf("Slave %d got: %lu\n", SLAVE_ID, (unsigned long)ts);
            gpio_set_level(LED_GPIO, ((ts % 2) == SLAVE_ID) ? 1 : 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


*/



/*

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include <stdio.h>
#include <string.h>

void app_main(void)
{
    if (lora_init() != ESP_OK) {
        printf("LoRa init failed!\n");
        return;
    }
    lora_set_frequency(433E6);   // 433 MHz

    uint32_t counter = 0;
    while (1) {
        char msg[16];
        sprintf(msg, "%lu", (unsigned long)counter);
        lora_send_packet((uint8_t*)msg, strlen(msg));
        printf("Master sent: %s\n", msg);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

*/