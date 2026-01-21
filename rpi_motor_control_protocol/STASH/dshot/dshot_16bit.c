#include "dshot_16bit.h"


static uint16_t throttle_percent_to_dshot(float percent) {
    if (percent <= 0.0f) return DSHOT_THROTTLE_MIN;
    if (percent >= 100.0f) return DSHOT_THROTTLE_MAX;

    const float p = percent / 100.0f; // 0..1

    uint32_t value = (uint32_t)roundf(p * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) + DSHOT_THROTTLE_MIN);
    if (value < DSHOT_THROTTLE_MIN) value = DSHOT_THROTTLE_MIN;
    if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
    return (uint16_t)value;
}


static void set_dshot_timings(float baud) {
    float T = 1000000.0f / baud;  // [µs/bit]

    t_bit_us = (uint32_t)roundf(T);
    t_high_1_us = (uint32_t)roundf(0.75f * T);   // 1 -> high 75% T
    t_high_0_us = (uint32_t)roundf(0.375f * T);  // 0 -> high 37.5% T
    t_low_1_us = (uint32_t)roundf(T - t_high_1_us);
    t_low_0_us = (uint32_t)roundf(T - t_high_0_us);

    /* ensure none are zero (safety) */
    if (t_high_1_us == 0) t_high_1_us = 1;
    if (t_high_0_us == 0) t_high_0_us = 1;
    if (t_low_1_us  == 0) t_low_1_us  = 1;
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


static void dshot_send_frame_bitbang(uint16_t packet, uint pin) {
    // From MSB to LSB
    for (int bit=15; bit>=0; --bit) {
        // Extract the current bit
        bool one = ((packet >> bit) & 1u) != 0u;

        // Drive PIN high
        gpio_put(pin, 1);
        if (one) {
            busy_wait_us_32(t_high_1_us);
        } else {
            busy_wait_us_32(t_high_0_us);
        }

        // Drive PIN low
        gpio_put(pin, 0);
        if (one) {
            busy_wait_us_32(t_low_1_us);
        } else {
            busy_wait_us_32(t_low_0_us);
        }
    }
}


static void dshot_task(void* params) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t last_cmd_value = DSHOT_THROTTLE_MIN;  // Set to 0% throttle

    // Sending frame period in ms
    const float T_ms = 1000 / DSHOT_FREQ;  // [ms]
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

        // "Critical region" where it disables interrupts/preemption briefly
        taskENTER_CRITICAL();
        dshot_send_frame_bitbang(dshot_pkt, dshot_pin);
        taskEXIT_CRITICAL();

        // Wait until next frame; this determines update rate
        vTaskDelayUntil(&last_wake, send_period_ticks);
    }
}


bool dshot_init(uint pin, uint baud) {
    dshot_pin = pin;
    dshot_baud = baud;

    // Configure gpio
    gpio_init(dshot_pin);
    gpio_set_dir(dshot_pin, GPIO_OUT);
    gpio_put(dshot_pin, 0);

    // Compute timings
    set_dshot_timings(dshot_baud);

    // Create queue
    xDshotThrottleQueue = xQueueCreate(DSHOT_THROTTLE_Q_SIZE, sizeof(uint16_t));
    if (xDshotThrottleQueue == NULL) return false;

    xDshotCommandQueue = xQueueCreate(DSHOT_COMMAND_Q_SIZE, sizeof(uint16_t));
    if (xDshotCommandQueue == NULL) return false;

    // Create task
    BaseType_t res = xTaskCreate(dshot_task, "DSHOT", DSHOT_TASK_STACK_WORDS, NULL, DSHOT_TASK_PRIORITY, &xDshotTaskHandle);
    if (res != pdPASS) return false;
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
