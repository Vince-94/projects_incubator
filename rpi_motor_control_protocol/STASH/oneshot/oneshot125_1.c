/**
 * @file oneshot125.c
 * @author 
 * @brief 
 * 
 *  This module implements:
 *   - OneShot125 PWM generation (125-250µs pulses) for a BLHeli S ESC (PWM_GPIO).
 *   - PWM for a status LED (LED2_GPIO) proportional to throttle.
 *   - Button input (BUTTON_GPIO) to toggle throttle between min and a preset value.
 *   - Optional: Serial/USB debug output. *
 * 
 * @version 0.1
 * @date 2025-10-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// Common libs
#include <string.h>
#include <stdio.h>
// #include <stdin.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>  // dynamic allocation (malloc)

#include "config.h"

// RPi lib
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

// FreeRTOS lib
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"



/* ---------------------------- GLOBAL OBJECTS ---------------------------- */

#define NUM_PWM_PINS 16

static const uint PWM_GPIOS[NUM_PWM_PINS] = {
    0, 1,   // slice 0, channels A/B
    2, 3,   // slice 1
    4, 5,   // slice 2
    6, 7,   // slice 3
    8, 9,   // slice 4
    10, 11, // slice 5
    12, 13, // slice 6
    14, 15  // slice 7
};


// PWM control structures
typedef struct {
    uint gpio;
    uint slice;
    uint channel;
} pwm_control_t;


typedef struct {
    pwm_control_t* motors_pwm;
    uint num_pins;
} oneshot_task_params_t;


typedef struct {
    uint8_t count;       // Number of motors
    float throttles[NUM_PWM_PINS];    // TODO Array of throttles
} throttle_msg_t;


// Persistent array of current targets (one per PWM pin)
static float targets[NUM_PWM_PINS];

// FreeRTOS handle
static TaskHandle_t xControlTaskHandle = NULL;  // task handle
static QueueHandle_t xThrottleQueue = NULL;  // queue handle


/* --------------------------- HELPER FUNCTIONS --------------------------- */

static void pwm_setup_pin(uint pin, uint wrap, float target_freq, pwm_control_t* pwm) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm->slice = pwm_gpio_to_slice_num(pin);
    pwm->channel = pwm_gpio_to_channel(pin);

    // Compute divider
    const uint32_t sys_clk = clock_get_hz(clk_sys);
    float divider = (float)sys_clk / ((float)(wrap + 1) * target_freq); // +1 for inclusive wrap
    if (divider < 1.0f) divider = 1.0f;  // Prevent invalid divider

    pwm_set_clkdiv(pwm->slice, divider);
    pwm_set_wrap(pwm->slice, wrap);
    pwm_set_chan_level(pwm->slice, pwm->channel, 0);
    pwm_set_enabled(pwm->slice, true);
}


static void pwm_set_duty_percent(float duty_percent, pwm_control_t* pwm) {
    if (duty_percent <= 0.0f) {
        pwm_set_chan_level(pwm->slice, pwm->channel, 0);
        return;
    }
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t level = (uint32_t)roundf((duty_percent / 100.0f) * (float)(PWM_WRAP + 1));
    if (level > PWM_WRAP) level = PWM_WRAP;

    pwm_set_chan_level(pwm->slice, pwm->channel, level);
}


/**
 * @brief Map input throttle into duty-cycle.
 * 1. Normalize throttle [0~1]
 * 2. Mapping throttle to pulse in µs
 * 
 * @param throttle 
 * @return float 
 */
static float map_throttle_duty(const float throttle) {
    // Normalize throttle [0~1]
    float rpm_norm = (throttle - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN);

    // Map to pulse width in microseconds
    float pulse_us = ESC_MIN_US + rpm_norm * (ESC_MAX_US - ESC_MIN_US);

    // Convert to duty cycle percentage based on PWM period
    float duty = (pulse_us / PWM_TARGET_PERIOD_US) * 100.0f; // e.g., 1000µs / 20000µs = 5%
    return duty;
}


/* ------------------------------- FREERTOS ------------------------------- */

void OneShotTask(void* params) {
    // Unpack params
    oneshot_task_params_t* task_params = (oneshot_task_params_t*)params;

    pwm_control_t* motors_pwm = task_params->motors_pwm;
    uint pwm_pins = task_params->num_pins;

    // initialize persistent targets to safe min
    for (uint i = 0; i < pwm_pins; ++i) targets[i] = THROTTLE_MIN;

    // 
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop_period_ms = (uint32_t)(1000u / LOOP_FREQ_HZ);
    const TickType_t loop_ticks = pdMS_TO_TICKS(loop_period_ms);

    // Set up PWM for motor
    for (uint i=0; i<pwm_pins; i++) {
        pwm_setup_pin(motors_pwm[i].gpio, PWM_WRAP, PWM_TARGET_FREQ, &motors_pwm[i]);
        pwm_set_duty_percent(0.0f, &motors_pwm[i]);  // initially set to 0%
    }

    while(true) {
        // Delay
        vTaskDelayUntil(&last_wake, loop_ticks);

        // TODO Manual STOP
        // Wait until the controller is enable by pressing the button (enable_btn_callback)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, loop_ticks);
        if (notified > 0) {
            for (uint i = 0; i < pwm_pins; ++i) targets[i] = THROTTLE_MIN;
            printf("[DEBUG] STOP motors!");
        }

        // Drain all messages present (non-blocking)
        throttle_msg_t msg;
        while (xQueueReceive(xThrottleQueue, &msg, 0) == pdTRUE) {
            if (msg.count == 0) continue; // ignore empty

            if (msg.count > pwm_pins) {
                printf("[WARN] throttle msg count %d > pwm_pins %d. Clamping.\n", msg.count, pwm_pins);
            }

            uint copy_n = (msg.count <= pwm_pins) ? msg.count : pwm_pins;

            // Clamping PWM
            for (uint i = 0; i < copy_n; ++i) {
                float t = msg.throttles[i];
                if (t < THROTTLE_MIN) t = THROTTLE_MIN;
                if (t > THROTTLE_MAX) t = THROTTLE_MAX;
                targets[i] = t;
            }

        }

        // Send PWM outputs once using the persistent targets[]
        for (uint i = 0; i < pwm_pins; ++i) {
            float motor_duty = map_throttle_duty(targets[i]);
            pwm_set_duty_percent(motor_duty, &motors_pwm[i]);
        }

    }

}


void gpio_global_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_GPIO) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xControlTaskHandle == NULL) {
            return;
        }

        // Notify task from ISR
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);

        // If the new task has higher priority, then execute it
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/* --------------------------------- MAIN --------------------------------- */


int main() {

    // Select the GPIOs
    static const uint pwm_pins_needed = 4;

    // static pwm_control_t motors_pwm[pwm_pins_needed];
    pwm_control_t* motors_pwm = malloc(pwm_pins_needed * sizeof(pwm_control_t));  // TODO malloc
    if (motors_pwm == NULL) {
        printf("Failed to allocate memory for motors_pwm\n");
        return -1; // Handle allocation failure
    }

    // Initialize the motors_pwm array
    for (int i = 0; i < pwm_pins_needed; i++) {
        motors_pwm[i].gpio = PWM_GPIOS[i];
    }

    static oneshot_task_params_t oneshot125_params;
    oneshot125_params.motors_pwm = motors_pwm;
    oneshot125_params.num_pins = pwm_pins_needed;


    stdio_init_all();

    printf("OneShot125 initialization...\n");

    // Initialize gpio BUTTON
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    // Initialize ADC
    adc_init();
    adc_gpio_init(JOY_Y_GPIO); // ADC1

    // FreeRTOS queues
    int queue_length = 8;
    xThrottleQueue = xQueueCreate(queue_length, sizeof(throttle_msg_t));
    if (xThrottleQueue == NULL) {
        printf("Failed to create throttle queue\n");
        return -1;
    }

    // FreeRTOS tasks
    BaseType_t res;

    res = xTaskCreate(OneShotTask, "ONESHOT125", PWM_TASK_STACK_WORDS, &oneshot125_params, 1, &xControlTaskHandle);
    configASSERT(res == pdPASS && xControlTaskHandle != NULL);

    // ISR
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_global_callback);

    /* Start the FreeRTOS scheduler. This does not return. */
    vTaskStartScheduler();

    return 0;
}
