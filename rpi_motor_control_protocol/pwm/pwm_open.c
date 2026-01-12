/**
 * @file pwm.c
 * @author 
 * @brief PWM + Encoder closed-loop controller for RP2350 (RP2040 family) using FreeRTOS.
 * 
 *   This module implements:
 *     - PWM generation for a motor (PWM_GPIO) and a status LED (LED2_GPIO).
 *     - A momentary button (BUTTON_GPIO) to toggle the target between 0 and a preset value.
 *
 *   Inputs:
 *     - BUTTON_GPIO: falling-edge button to toggle target.
 *     - Optional: serial/USB (stdio) for debug prints.
 *
 *   Outputs:
 *     - PWM_GPIO: PWM signal to motor driver (configured @ PWM_TARGET_FREQ, wrap = PWM_WRAP).
 *     - LED2_GPIO: PWM driven LED to show motor speed proportionally.
 *     - USB serial debugging prints (stdio).
 *
 *   FreeRTOS objects:
 *     - Task: PwmTask — main control loop running at LOOP_FREQ_HZ.
 *     - ISR: gpio_global_callback — handles button press,
 *            notifies PwmTask via task notification from ISR.
 *
 *   Key parameters:
 *     - LOOP_FREQ_HZ: control loop update rate (Hz) -> sample period Ts = 1/LOOP_FREQ_HZ
 *     - PWM_TARGET_FREQ: desired PWM switching frequency (Hz)
 *     - PWM_WRAP: PWM top / resolution (the SDK counts 0..wrap inclusive)
 *
 *   Workflow:
 *     - On startup PwmTask configures PWM for motor and LED, and sets an initial target.
 *     - Each control loop iteration:
 *         1. Map target to a duty % (map_throttle_duty).
 *         2. Write duty to PWM for motor and LED.
 *     - Button press (ISR) notifies PwmTask to toggle between target values.
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
// #include "hardware/adc.h"

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
    float period_us = PWM_TARGET_PERIOD_US; // e.g., 20000µs at 50Hz
    float duty = (pulse_us / period_us) * 100.0f; // e.g., 1000µs / 20000µs = 5%
    return duty;
}


/* ------------------------------- FREERTOS ------------------------------- */

void PwmTask(void* params) {
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
    bool change_rev = false;

    // Dynamic params
    float target_rpm = THROTTLE_MIN;   // TODO hardcoded -> can be set via CLI

    while(true) {
        // Delay
        vTaskDelayUntil(&last_wake, loop_ticks);

        // Wait until the controller is enable by pressing the button (enable_btn_callback)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, loop_ticks);

        // TODO Set manually
        if (notified > 0) {
            change_rev = !change_rev;
            if (change_rev == true) {
                target_rpm = 20.0;
            } else {
                target_rpm = THROTTLE_MIN;
            }
            printf("[DEBUG] target_rpm=%.2f\n", target_rpm);
        }

        // Compute motor PWM duty cycle
        float motor_duty = map_throttle_duty(target_rpm);
        printf("[DEBUG] target_rpm=%.2f, duty=%.2f\n", target_rpm, motor_duty);

        pwm_set_duty_percent(motor_duty, &motor_pwm);

        // Set LED intensity proportional to motor speed
        pwm_set_duty_percent(motor_duty, &led_pwm);
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
    printf("PWM initialization...\n");
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

    // TODO Initialize ADC
    // adc_init();
    // adc_gpio_init(JOY_Y_GPIO); // ADC1

    // Tasks
    BaseType_t res;

    res = xTaskCreate(PwmTask, "PWM", PWM_TASK_STACK_WORDS, NULL, 1, &xControlTaskHandle);
    configASSERT(res == pdPASS && xControlTaskHandle != NULL);

    // ISR
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_global_callback);

    /* Start the FreeRTOS scheduler. This does not return. */
    vTaskStartScheduler();

    return 0;
}
