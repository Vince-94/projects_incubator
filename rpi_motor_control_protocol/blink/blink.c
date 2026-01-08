#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"


#define LED_DELAY_MS 100


void BlinkTask(void* params) {
    (void) params;

    const TickType_t delay_ticks = pdMS_TO_TICKS(LED_DELAY_MS);

    while(true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        printf("Led ON\n");
        vTaskDelay(delay_ticks);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        printf("Led OFF\n");
        vTaskDelay(delay_ticks);
    }
}


int main()
{
    printf("Initialization...\n");
    stdio_init_all();

    // Initialize gpio
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Create task
    BaseType_t res = xTaskCreate(
        BlinkTask,
        "blink",
        256,
        NULL,
        1,
        NULL  // TaskHandle_t task_handle -> &task_handle
    );

    /* Start the FreeRTOS scheduler. This does not return. */
    vTaskStartScheduler();
}
