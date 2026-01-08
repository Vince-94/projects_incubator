/*
 * dshot.c
 *
 */
#include "FreeRTOS.h"
#include "task.h"

#define DSHOT_16BIT 1
#define DSHOT_PIO   2
#define DSHOT_TYPE  DSHOT_16BIT

#if DSHOT_TYPE == DSHOT_16BIT
    #include "src/dshot_16bit.h"
    static const char *DSHOT_STR = "DSHOT 16-bit (bitbang)";
#elif DSHOT_TYPE == DSHOT_PIO
    #include "src/dshot_pio.h"
    static const char *DSHOT_STR = "DSHOT PIO";
#else
    #error "No DSHOT implementation selected. Define DSHOT_USE_16BIT or DSHOT_USE_PIO in build."
#endif



static void demo_task(void* params) {
    float pct = 0.0f;
    bool up = true;

    while(true) {
        bool ok = dshot_set_throttle_percent(pct);
        if (!ok) {
            printf("WARNING: dshot_set_throttle_percent failed\r\n");
        }

        if (up) {  // rump-up
            pct += 5.0f;
            if (pct >= 100.0f) {
                pct = 100.0f;
                up = false;
            }
        } else {  // ramp-down
            pct -= 5.0f;
            if (pct <= 0.0f) {
                pct = 0.0f;
                up = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second step
    }
}


/* ------------------------- Main ------------------------- */

int main() {
    stdio_init_all();
    printf("\r\nDSHOT demo starting: %s\r\n", DSHOT_STR);

    const uint pin = 18u;
    const uint baud = 150000;
    bool status = dshot_init(pin, baud);
    if (!status) {
        printf("dshot_init() failed — aborting demo\r\n");
        while (1) tight_loop_contents();
    }

    BaseType_t r = xTaskCreate(demo_task, "DEMO", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
    configASSERT(r == pdPASS);

    vTaskStartScheduler();

    return 0;
}
