#include "dshot_pio.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


// ----------------- Configuration ----------------- //

// PIO
static PIO pio_inst = pio0;  // choose pio0 by default
static uint pio_sm_index = (uint)0xFFFFFFFFu;  // PIO SM index (default "not claimed")
static uint pio_sm_offset = PIO_INVALID_OFFSET;  // program offset returned by pio_add_program
static float pio_clkdiv = 1.0f;  // configured divider
static float pio_target_freq_hz = 12000000.0f;    // target pio clock frequency 12 MHz
static float pio_freq_hz = 0.0f;    // actual pio clock frequency after divider
static float pio_cycle_us = 0.0f;   // microseconds per PIO instruction (1 / pio_freq_hz * 1e6)

// DSHOT properties
static uint dshot_pin = 0;
static uint dshot_baud = DSHOT_DEFAULT_BAUD;

// Per-bit microsecond timings
static float T_frame = 0;
static uint32_t t_bit_us = 0;
static uint32_t t_high_1_us = 0;
static uint32_t t_high_0_us = 0;
static uint32_t t_low_1_us = 0;
static uint32_t t_low_0_us = 0;

// FreeRTOS allocation
static QueueHandle_t xDshotThrottleQueue = NULL;
static QueueHandle_t xDshotCommandQueue = NULL;
static TaskHandle_t xDshotTaskHandle = NULL;

// Telemetry
static volatile uint8_t dshot_request_telemetry_flag = 0u;
static volatile uint8_t dshot_telemetry_continuous = 0u;
static volatile uint16_t telemetry_freq = 100;  // Hz
static TickType_t dshot_telemetry_interval_ticks = 0;



static uint16_t throttle_percent_to_dshot(float percent) {
    if (percent <= 0.0f) return DSHOT_THROTTLE_MIN;
    if (percent >= 100.0f) return DSHOT_THROTTLE_MAX;

    const float p = percent / 100.0f; // 0..1

    uint32_t value = (uint32_t)roundf(p * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) + DSHOT_THROTTLE_MIN);
    if (value < DSHOT_THROTTLE_MIN) value = DSHOT_THROTTLE_MIN;
    if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
    return (uint16_t)value;
}


static void set_dshot_timings(uint baud) {
    // DSHOT bit period
    float T = 1000000.0f / baud;  // [µs/bit]
    t_bit_us = (uint32_t)roundf(T);

    // Total 16-bit frame period
    T_frame = T * 16;  // [µs/bit]

    // Per-bit timings encoding
    t_high_1_us = (uint32_t)roundf(0.75f * T);   // 1 -> HIGH 75% T
    if (t_high_1_us == 0) t_high_1_us = 1;

    t_low_1_us = (uint32_t)roundf(T - t_high_1_us);
    if (t_low_1_us  == 0) t_low_1_us  = 1;

    t_high_0_us = (uint32_t)roundf(0.375f * T);  // 0 -> HIGH 37.5% T
    if (t_high_0_us == 0) t_high_0_us = 1;

    t_low_0_us = (uint32_t)roundf(T - t_high_0_us);
    if (t_low_0_us  == 0) t_low_0_us  = 1;
}


static uint16_t build_dshot_packet(uint16_t throttle, bool request_telemetry) {
    if (throttle > DSHOT_THROTTLE_MAX) throttle = DSHOT_THROTTLE_MAX;

    // Compute 12-bit payload: throttle + telemerty
    uint16_t payload = (uint16_t)((throttle << 1) | (request_telemetry ? 1u : 0u));

    // Compute checksum 4-bit
    uint8_t csum = 0;
    uint16_t tmp = payload;
    for (int i = 0; i < 3; ++i) {
        csum ^= (uint8_t)(tmp & 0xF);
        tmp >>= 4;
    }
    csum &= 0xF;

    // Compose 16-bit packet
    uint16_t packet = (uint16_t)((payload << 4) | csum);

    return packet;
}


static bool configure_pio_sm(uint pin, uint offset) {
    // Claim an unused SM
    int sm = pio_claim_unused_sm(pio_inst, true);
    if (sm < 0) return false;
    pio_sm_index = (uint)sm;

    // Get default config and set pins/sideset
    pio_sm_config c = dshot_out_program_get_default_config(offset);  // default config for the program
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_out_shift(&c, false, false, 32); // not used, but safe
    sm_config_set_in_shift(&c, false, false, 32);
    pio_sm_set_consecutive_pindirs(pio_inst, pio_sm_index, pin, 1, true);  // set pindirs controlled by the SM

    // Apply clock divider computed
    sm_config_set_clkdiv(&c, pio_clkdiv);

    // Init sm at offset
    pio_sm_init(pio_inst, pio_sm_index, offset, &c);


    /* configure the GPIO for PIO (handle pio0 vs pio1) */
    pio_gpio_init(pio_inst, pin);
    if (pio_inst == pio0) gpio_set_function(pin, GPIO_FUNC_PIO0);
    else gpio_set_function(pin, GPIO_FUNC_PIO1);

    /* enable */
    pio_sm_set_enabled(pio_inst, pio_sm_index, true);

    return true;
}


static void configure_pio_clock(float pio_target_freq_hz) {
    // Compute a PIO clock divider that gives good microsecond resolution: pio_target_freq_hz ~ 10..20 MHz
    uint32_t sys_hz = clock_get_hz(clk_sys);
    float clkdiv = (float)sys_hz / pio_target_freq_hz;
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    pio_clkdiv = clkdiv;

    // Compute actual PIO frequency & cycle time (keep float precision)
    pio_freq_hz = (float)sys_hz / pio_clkdiv;
    pio_cycle_us = 1e6f / pio_freq_hz;
}


static inline uint32_t us_to_pio_cycles(float us) {
    float cycles = us / pio_cycle_us;
    if (cycles < 1.0f) cycles = 1.0f;
    return (uint32_t)roundf(cycles);
}


static void pio_dshot_send_frame(uint16_t packet) {
    for (int bit = 15; bit >= 0; --bit) {
        bool one = ((packet >> bit) & 1u) != 0u;
        float high_us = one ? (float)t_high_1_us : (float)t_high_0_us;
        float low_us  = one ? (float)t_low_1_us  : (float)t_low_0_us;
        uint32_t high_cycles = us_to_pio_cycles(high_us);
        uint32_t low_cycles  = us_to_pio_cycles(low_us);
        /* push high then low (PIO program expects 32-bit counts) */
        pio_sm_put_blocking(pio_inst, pio_sm_index, high_cycles);
        pio_sm_put_blocking(pio_inst, pio_sm_index, low_cycles);
    }
}


//---------------------------------- RTOS ----------------------------------//

static void dshot_task(void* params) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t last_cmd_value = DSHOT_THROTTLE_MIN;  // Set to 0% throttle

    // Sending frame period in ms
    const uint32_t T_ms = 1000u / (uint32_t)DSHOT_UPDATE_FREQ;
    const TickType_t send_period_ticks = pdMS_TO_TICKS(T_ms);

    // Infinite loop
    while (true) {

        uint16_t new_cmd;
        // Read latest throttle/command if any (non-blocking with short timeout)
        if (xQueueReceive(xDshotCommandQueue, &new_cmd, pdMS_TO_TICKS(0)) == pdTRUE) {
            last_cmd_value = new_cmd;
        } else if (xQueueReceive(xDshotThrottleQueue, &new_cmd, pdMS_TO_TICKS(0)) == pdTRUE) {
            last_cmd_value = new_cmd;
        }

        // Build packet
        bool request_telemetry = false;
        if (dshot_telemetry_continuous) {
            request_telemetry = true;
        } else if (dshot_request_telemetry_flag) {
            request_telemetry = true;
            dshot_request_telemetry_flag = 0u;  // Reset to 0
        }
        uint16_t dshot_pkt = build_dshot_packet(last_cmd_value, request_telemetry);

        pio_dshot_send_frame(dshot_pkt);

        vTaskDelayUntil(&last_wake, send_period_ticks);
    }

}


//------------------------------- Public API -------------------------------//

bool dshot_init(uint pin, float baud) {
    dshot_pin = pin;
    dshot_baud = baud;

    // Configure gpio
    gpio_init(dshot_pin);
    gpio_set_dir(dshot_pin, GPIO_OUT);
    gpio_put(dshot_pin, 0);

    configure_pio_clock(pio_target_freq_hz);

    // add program to PIO (if not already added)
    pio_sm_offset = pio_add_program(pio_inst, &dshot_out_program);
    if (pio_sm_offset == PIO_INVALID_OFFSET) return false;

    bool status = configure_pio_sm(dshot_pin, pio_sm_offset);
    if (status == false) return false;

    set_dshot_timings(dshot_baud);

    // Create queue
    xDshotThrottleQueue = xQueueCreate(DSHOT_THROTTLE_Q_SIZE, sizeof(uint16_t));
    if (xDshotThrottleQueue == NULL) return false;

    xDshotCommandQueue = xQueueCreate(DSHOT_COMMAND_Q_SIZE, sizeof(uint16_t));
    if (xDshotCommandQueue == NULL) return false;

    // Create task
    BaseType_t res = xTaskCreate(dshot_task, "DSHOT", DSHOT_TASK_STACK_WORDS, NULL, DSHOT_TASK_PRIORITY, &xDshotTaskHandle);
    if (res != pdPASS) return false;

    return true;
}


bool dshot_set_raw_value(uint16_t raw) {
    if (xDshotThrottleQueue == NULL) return false;

    BaseType_t ok;
    if (raw >= DSHOT_THROTTLE_MIN) {  // Throttle command
        if (raw > DSHOT_THROTTLE_MAX) raw = DSHOT_THROTTLE_MAX;
        ok = xQueueOverwrite(xDshotThrottleQueue, &raw);
    } else {  // Reserved command
        if (raw < DSHOT_COMMAND_MIN) return false;  // Invalid command
        ok = xQueueSendToBack(xDshotCommandQueue, &raw);
    }

    if (ok != pdTRUE) return false;

    return true;
}


bool dshot_set_throttle_percent(float percent) {
    if (xDshotThrottleQueue == NULL) return false;

    uint16_t v = throttle_percent_to_dshot(percent);

    BaseType_t ok = xQueueOverwrite(xDshotThrottleQueue, &v);
    return (ok == pdTRUE);
}


bool dshot_request_telemetry_once(void) {
    dshot_request_telemetry_flag = 1u;  // atomic write of byte on most MCUs
    return true;
}

bool dshot_request_telemetry_continuous(bool enable, uint16_t frequency) {
    dshot_telemetry_continuous = enable ? 1u : 0u;
    // telemetry_freq = frequency;
    return true;
}

void dshot_deinit(void) {
    /* 1) Delete the RTOS task (if running) */
    if (xDshotTaskHandle != NULL) {
        vTaskDelete(xDshotTaskHandle);
        xDshotTaskHandle = NULL;
    }

    /* 2) Delete queues */
    if (xDshotThrottleQueue != NULL) {
        vQueueDelete(xDshotThrottleQueue);
        xDshotThrottleQueue = NULL;
    }
    if (xDshotCommandQueue != NULL) {
        vQueueDelete(xDshotCommandQueue);
        xDshotCommandQueue = NULL;
    }

    /* 3) Disable the PIO state machine (if claimed) and restore GPIO */
    if (pio_sm_index != (uint)0xFFFFFFFFu) {
        /* Stop the state machine */
        pio_sm_set_enabled(pio_inst, pio_sm_index, false);

        /* Flush the SM FIFOs (defensive; safe even if implementation changes) */
#if defined(pio_sm_is_claimed) /* compile-time check - harmless if undefined */
        /* nothing - keep backward compatible */
#else
        /* Most SDKs provide pio_sm_set_enabled which is enough; do not attempt to unclaim program */
#endif

        /* Return pin to SIO GPIO and drive low to be safe */
        gpio_put(dshot_pin, 0);
        gpio_set_dir(dshot_pin, GPIO_OUT);
        gpio_set_function(dshot_pin, GPIO_FUNC_SIO);

        /* Mark SM as no longer in-use by this module */
        pio_sm_index = (uint)UINT32_MAX;
    }

    /* 4) Reset telemetry flags/state */
    dshot_request_telemetry_flag = 0u;
    dshot_telemetry_continuous = 0u;
    dshot_telemetry_interval_ticks = 0;

    /* 5) Leave program in PIO instruction memory (removing it is not commonly supported).
       If you must reclaim instruction RAM, you would need a different initialization strategy
       or reboot the device. */
}
