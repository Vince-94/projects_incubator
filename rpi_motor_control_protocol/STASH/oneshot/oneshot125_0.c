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

static TaskHandle_t xControlTaskHandle = NULL;

// PWM control structures for motor and LED
typedef struct {
    uint slice;
    uint channel;
} pwm_control_t;

static pwm_control_t motor_pwm;
static pwm_control_t led_pwm;


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
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop_period_ms = (uint32_t)(1000u / LOOP_FREQ_HZ);

    const TickType_t loop_ticks = pdMS_TO_TICKS(loop_period_ms);

    // Set up PWM for motor
    pwm_setup_pin(PWM_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &motor_pwm);
    pwm_set_duty_percent(0.0f, &motor_pwm);  // initially set to 0%

    // Set up PWM for LED
    pwm_setup_pin(LED2_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led_pwm);
    pwm_set_duty_percent(0.0f, &led_pwm);  // initially set to 0%

    // Loop variable initialization
    bool joystick_control = true; // Start with joystick control enabled
    TickType_t last_adjustment = xTaskGetTickCount(); // For rate limiting

    // Dynamic params
    float target_throttle = THROTTLE_MIN;   // TODO hardcoded -> can be set via CLI

    while(true) {
        // Delay
        vTaskDelayUntil(&last_wake, loop_ticks);

        // Wait until the controller is enable by pressing the button (enable_btn_callback)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, loop_ticks);

        // TODO Manual STOP
        if (notified > 0) {
            target_throttle = THROTTLE_MIN;
            printf("[DEBUG] STOP motor!");
        }

        // TODO Read joystick (Y-axis, ADC1)
        if (joystick_control) {
            adc_select_input(1); // ADC1 = GP27
            uint16_t adc_value = adc_read();
            float joystick_norm = ((float)adc_value / ADC_MAX_RESOLUTION) * 2.0f - 1.0f; // Map 0-4095 to [-1, 1]

            // Rate limit adjustments (every 100ms to avoid rapid changes)
            if (xTaskGetTickCount() - last_adjustment >= pdMS_TO_TICKS(100)) {
                if (joystick_norm > JOY_THRESHOLD) {
                    target_throttle += THROTTLE_STEP; // Increase by 5%
                    if (target_throttle > THROTTLE_MAX) target_throttle = THROTTLE_MAX;
                    last_adjustment = xTaskGetTickCount();
                } else if (joystick_norm < -JOY_THRESHOLD) {
                    target_throttle -= THROTTLE_STEP; // Decrease by 5%
                    if (target_throttle < THROTTLE_MIN) target_throttle = THROTTLE_MIN;
                    last_adjustment = xTaskGetTickCount();
                }
            }
        }

        // Compute motor PWM duty cycle
        float motor_duty = map_throttle_duty(target_throttle);
        printf("[DEBUG] target_throttle=%.2f, duty=%.2f%%\n", target_throttle, motor_duty);

        pwm_set_duty_percent(motor_duty, &motor_pwm);

        // Set LED intensity proportional to motor speed
        float led_norm = (target_throttle - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN);
        pwm_set_duty_percent(led_norm * 100.0f, &led_pwm);
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
    printf("OneShot125 initialization...\n");
    stdio_init_all();

    // Initialize gpio LED1
    gpio_init(LED1_GPIO);
    gpio_set_dir(LED1_GPIO, GPIO_OUT);

    // Initialize gpio LED2
    gpio_init(LED2_GPIO);
    gpio_set_dir(LED2_GPIO, GPIO_OUT);

    // Initialize gpio BUTTON
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    // Initialize ADC
    adc_init();
    adc_gpio_init(JOY_Y_GPIO); // ADC1

    // Tasks
    BaseType_t res;

    res = xTaskCreate(OneShotTask, "ONESHOT125", PWM_TASK_STACK_WORDS, NULL, 1, &xControlTaskHandle);
    configASSERT(res == pdPASS && xControlTaskHandle != NULL);

    // ISR
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_global_callback);

    /* Start the FreeRTOS scheduler. This does not return. */
    vTaskStartScheduler();

    return 0;
}
