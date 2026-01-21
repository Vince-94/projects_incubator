/*
 * blink_priority.c
 *
 * RP2350 (Pico 2) - FreeRTOS fundamentals demo
 *
 * Purpose / Overview
 * ------------------
 * - Demonstrates core FreeRTOS primitives and safe ISR → task communication.
 * - Tasks:
 *     * blink1_task  : toggles on-board LED at 500 ms (low priority periodic task)
 *     * blink2_task : toggles external LED at 750 ms (low priority periodic task)
 *     * btn_handler_task : waits for ISR notification (button press), then sends log msg
 *     * cli_task    : non-blocking poll of USB CDC stdin, sends log messages
 *     * logger_task : consumes log messages from a queue and prints to USB serial
 *
 * - Synchronization & design choices:
 *     * ISR uses vTaskNotifyGiveFromISR -> btn_handler_task: extremely low-cost signaling
 *     * Logging uses a queue of fixed-size message structs (no pointer lifetime issues)
 *     * All lengthy work (snprintf, queue copies) is done in task context, NOT in the ISR
 *     * Periodic tasks use vTaskDelay (cooperative with the scheduler)
 *
 * Safety / Improvements (notes inside comments):
 * - Do NOT send pointers to stack buffers over queues; this demo uses value-copy messages.
 * - Enable stack overflow hooks and configASSERT in FreeRTOSConfig.h in your project.
 * - Keep ISRs minimal; defer work to tasks as shown.
 *
 * Build: This file expects Pico SDK + FreeRTOS available in your project. (Per user request,
 *       no CMake or FreeRTOSConfig provided here — only code.)
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* --------- Definitions ---------- */
#define LED1_DELAY_MS 500
#define LED2_DELAY_MS 750

#define LED2_GPIO 15
#define BUTTON_GPIO 14


/* --------- Small logging message type (queue carries these by value) ---------- */
typedef struct {
    char text[96];
} LogMsg_t;


/* --------- RTOS objects (file-scope so ISR and tasks can access) ---------- */
static TaskHandle_t xBtnTaskHandle = NULL;
static QueueHandle_t xLogQueue;


void Blink1Task(void* params) {
    (void) params;

    const TickType_t delay_ticks = pdMS_TO_TICKS(LED1_DELAY_MS);

    while(true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        printf("Led 1 ON\n");
        vTaskDelay(delay_ticks);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        printf("Led 1 OFF\n");
        vTaskDelay(delay_ticks);
    }
}


void Blink2Task(void* params) {
    (void) params;

    const TickType_t delay_ticks = pdMS_TO_TICKS(LED2_DELAY_MS);

    while(true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        printf("Led 2 ON\n");
        vTaskDelay(delay_ticks);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        printf("Led 2 OFF\n");
        vTaskDelay(delay_ticks);
    }
}


void BottonHandlerTask(void* params) {
    while(true) {

        // ulTaskNotifyTake: clears notification value when pdTRUE passed as first arg
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // block until ISR signals

        // Compose log message in task context
        LogMsg_t m;
        int n = snprintf(m.text, sizeof(m.text), "Button pressed on GPIO %d\r\n", BUTTON_GPIO);

        /* Ensure NULL-termination */
        if (n < 0) {
            strncpy(m.text, "Button pressed (snprintf error)\r\n", sizeof(m.text));
            m.text[sizeof(m.text) - 1] = '\0';
        }

        /* Send to logger queue; if queue full, drop message (0 timeout) to avoid blocking here.
           In other designs you may block with a timeout or implement a larger queue. */
        xQueueSend(xLogQueue, &m, 0);
    }
}


// Logger task consumes messages from queue and prints on USB serial
void logger_task(void* params) {
    LogMsg_t recv;

    while(true) {
        // Wait for a message (block forever) — logger should always be able to print incoming logs
        if (xQueueReceive(xLogQueue, &recv, portMAX_DELAY) == pdTRUE) {
            printf("%s", recv.text);
        }
    }
}


/* ISR callback used by pico SDK for GPIO interrupts
*  The callback has to be short and Non-blocking. In this case it just calls
*  another tasks.
*/
static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the FreeRTOS task (BottonHandlerTask)
    if (xBtnTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xBtnTaskHandle, &xHigherPriorityTaskWoken);
    }

    // If the task we woke has higher priority, do a context switch immediately
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Simple CLI task:
   - Polls stdin non-blocking using pico stdio getchar_timeout_us(0)
   - On any received char, formats a short message and posts to xLogQueue
   - Non-blocking read + small vTaskDelay reduces CPU usage
*/
void cli_task(void *pv) {
    while(true) {
        int c = getchar_timeout_us(0); /* 0 => non-blocking; returns <0 if no char */
        if (c >= 0) {
            LogMsg_t m;
            int n = snprintf(m.text, sizeof(m.text), "CLI: got char '%c' (0x%02X)\r\n", (char)c, (unsigned)c);
            if (n < 0) {
                strncpy(m.text, "CLI: snprintf error\r\n", sizeof(m.text));
                m.text[sizeof(m.text) - 1] = '\0';
            }
            /* If queue is full we drop the message; for demos that's acceptable. */
            xQueueSend(xLogQueue, &m, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


int main() {
    /* --------- Initialization ---------- */
    printf("Initialization...\n");
    stdio_init_all();

    // Initialize gpio LED1
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Initialize gpio LED2
    gpio_init(LED2_GPIO);
    gpio_set_dir(LED2_GPIO, GPIO_OUT);

    // Initialize botton input with pull-up and falling-edge IRQ
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    /* Create a queue capable of holding 8 LogMsg_t structures (value-copied).
       Using value-copies here avoids pointer lifetime bugs and is simpler for demos. */
    xLogQueue = xQueueCreate(8, sizeof(LogMsg_t));
    configASSERT(xLogQueue);

    /* --------- Tasks/ISR ---------- */
    BaseType_t res;

    res = xTaskCreate(Blink1Task, "BLINK1", 256, NULL, 1, NULL);
    configASSERT(res == pdPASS);

    res = xTaskCreate(Blink2Task, "BLINK2", 256, NULL, 2, NULL);
    configASSERT(res == pdPASS);

    res = xTaskCreate(BottonHandlerTask, "BTN", 512, NULL, 3, &xBtnTaskHandle);
    configASSERT(res == pdPASS);

    res = xTaskCreate(logger_task, "LOG", 1024, NULL, 1, NULL);
    configASSERT(res == pdPASS);

    res = xTaskCreate(cli_task, "CLI", 512, NULL, 1, NULL);
    configASSERT(res == pdPASS);

    /* Enable the GPIO interrupt and register the callback.
       gpio_set_irq_enabled_with_callback sets the callback for all IRQs; we enable only the
       falling-edge for this specific pin. */
    gpio_set_irq_enabled_with_callback(
        BUTTON_GPIO,         // (gpio) GPIO14
        GPIO_IRQ_EDGE_FALL,  // (event) trigger when pin goes from HIGH → LOW
        true,                // enable gpio_callback
        &gpio_callback       // call this function on interrupt
    );

    /* --------- Start the FreeRTOS scheduler ---------- */
    vTaskStartScheduler();

    return 0;
}
