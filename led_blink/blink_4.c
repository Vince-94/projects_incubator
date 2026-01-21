// Common libs
#include <stdio.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Rpi libs
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// FreeRTOS libs
#include "FreeRTOS.h"
#include "task.h"


/* --------- Definitions ---------- */

#define LED0_DELAY_MS 1000
#define LOOP_PERIOD 500
#define PWM_PERIOD 2

#define LED0_GPIO PICO_DEFAULT_LED_PIN
#define LED1_GPIO 16
#define LED2_GPIO 17
#define LED3_GPIO 18
#define LED4_GPIO 19
#define LED5_GPIO 20

#define BUTTON_GPIO 14

#define PWM_WRAP (20000u - 1)  // PWM wrap/resolution, aka number of counts in one PWM cycle (Depends on MCU)
#define PWM_TARGET_FREQ 50u    // desired PWM signal frequency [Hz]  (depends on ESC spec)
#define PWM_TARGET_PERIOD_US (1000000.0f / PWM_TARGET_FREQ)    // PWM period [µs]
#define THROTTLE_MIN 0.0f
#define THROTTLE_MAX 100.0f
#define ESC_MIN_US 1000u  // PWM pulse width [µs] for zero throttle (0%) on ESC -> STANDARD (depends on ESC spec)
#define ESC_MAX_US 2000u  // PWM pulse width [µs] for full throttle (100%) on ESC -> STANDARD (depends on ESC spec)


/* --------- Global variables ---------- */

enum Mode {
    OFF,
    ON,
    BLINK,
    BLINK_QUARTER,
    DIMMER,
    DIMMER_ALT,
    // DIMMER_QUARTER,  // TODO
    __COUNT__
};

typedef struct {
    uint slice;
    uint channel;
} pwm_control_t;

static TaskHandle_t xButtonTaskHandle = NULL;  // task handle

static pwm_control_t led1_pwm;
static pwm_control_t led2_pwm;
static pwm_control_t led3_pwm;
static pwm_control_t led4_pwm;
static pwm_control_t led5_pwm;


/* --------- Utils functions ---------- */

static void pwm_setup_pin(uint pin, uint wrap, float target_freq, pwm_control_t* pwm) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm->slice = pwm_gpio_to_slice_num(pin);
    pwm->channel = pwm_gpio_to_channel(pin);

    // Compute divider
    const uint32_t sys_clk = clock_get_hz(clk_sys);
    float divider = (float)sys_clk / ((float)(wrap + 1) * target_freq);  // +1 for inclusive wrap
    if (divider < 1.0f) divider = 1.0f;

    pwm_set_clkdiv(pwm->slice, divider);
    pwm_set_wrap(pwm->slice, wrap);
    pwm_set_chan_level(pwm->slice, pwm->channel, 0);
    pwm_set_enabled(pwm->slice, true);
}


static void pwm_disable_pin(uint pin, pwm_control_t* pwm) {
    pwm_set_enabled(pwm->slice, false);
    gpio_set_function(pin, GPIO_FUNC_SIO);
}


static void pwm_set_duty_percent(float duty_percent, pwm_control_t* pwm) {
    if (duty_percent <= 0.0f) {
        pwm_set_chan_level(pwm->slice, pwm->channel, 0);
        return;
    }
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // uint32_t wrap = pwm_hw->slice[pwm_slice].top;  // TODO wrap -> PWM_WRAP

    uint32_t level = (uint32_t)roundf((duty_percent / 100.0f) * (float)(PWM_WRAP + 1));
    if (level > PWM_WRAP) level = PWM_WRAP;

    pwm_set_chan_level(pwm->slice, pwm->channel, level);
}


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


/* --------- Tasks ---------- */

void Blink0Task(void* params) {
    (void) params;

    const TickType_t delay_ticks = pdMS_TO_TICKS(LED0_DELAY_MS);

    while(true) {
        gpio_put(LED0_GPIO, true);
        vTaskDelay(delay_ticks);
        gpio_put(LED0_GPIO, false);
        vTaskDelay(delay_ticks);
    }
}


void LoopTask(void* params) {
    (void) params;

    bool enabled = false;
    enum Mode mode = 0;
    enum Mode prev_mode = mode-1;

    TickType_t loop_ticks = pdMS_TO_TICKS(LOOP_PERIOD);
    TickType_t pwm_ticks = pdMS_TO_TICKS(PWM_PERIOD);
    uint32_t idle_time = 1000;
    TickType_t last_adjustment = xTaskGetTickCount(); // For rate limiting

    float target_val = THROTTLE_MIN;
    bool ascending = true;


    while(true) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, 0);
        if (notified > 0 && xTaskGetTickCount() - last_adjustment >= pdMS_TO_TICKS(idle_time)) {
            mode++;
            prev_mode++;
            if (mode == __COUNT__) mode = 0;
            if (prev_mode == __COUNT__) prev_mode = 0;
        }


        if (mode == DIMMER || mode == DIMMER_ALT) {
            // Init PWM
            pwm_setup_pin(LED1_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led1_pwm);
            pwm_setup_pin(LED2_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led2_pwm);
            pwm_setup_pin(LED3_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led3_pwm);
            pwm_setup_pin(LED4_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led4_pwm);
            pwm_setup_pin(LED5_GPIO, PWM_WRAP, PWM_TARGET_FREQ, &led5_pwm);
        } else if ((mode != DIMMER || mode != DIMMER_ALT) && (prev_mode == DIMMER || prev_mode == DIMMER_ALT)) {
            pwm_disable_pin(LED1_GPIO, &led1_pwm);
            pwm_disable_pin(LED2_GPIO, &led2_pwm);
            pwm_disable_pin(LED3_GPIO, &led3_pwm);
            pwm_disable_pin(LED4_GPIO, &led4_pwm);
            pwm_disable_pin(LED5_GPIO, &led5_pwm);

            target_val = THROTTLE_MIN;
            ascending = true;
        }



        switch (mode) {
            case OFF:
                gpio_put(LED1_GPIO, false);
                gpio_put(LED2_GPIO, false);
                gpio_put(LED3_GPIO, false);
                gpio_put(LED4_GPIO, false);
                gpio_put(LED5_GPIO, false);
                vTaskDelay(loop_ticks);
                break;

            case ON:
                gpio_put(LED1_GPIO, true);
                gpio_put(LED2_GPIO, true);
                gpio_put(LED3_GPIO, true);
                gpio_put(LED4_GPIO, true);
                gpio_put(LED5_GPIO, true);
                vTaskDelay(loop_ticks);
                break;

            case BLINK:
                gpio_put(LED1_GPIO, true);
                gpio_put(LED2_GPIO, true);
                gpio_put(LED3_GPIO, true);
                gpio_put(LED4_GPIO, true);
                gpio_put(LED5_GPIO, true);
                vTaskDelay(loop_ticks/2);

                gpio_put(LED1_GPIO, false);
                gpio_put(LED2_GPIO, false);
                gpio_put(LED3_GPIO, false);
                gpio_put(LED4_GPIO, false);
                gpio_put(LED5_GPIO, false);
                vTaskDelay(loop_ticks/2);

                break;


            case BLINK_QUARTER:
                gpio_put(LED1_GPIO, true);
                vTaskDelay(loop_ticks/4);
                gpio_put(LED1_GPIO, false);

                gpio_put(LED2_GPIO, true);
                vTaskDelay(loop_ticks/4);
                gpio_put(LED2_GPIO, false);

                gpio_put(LED3_GPIO, true);
                vTaskDelay(loop_ticks/4);
                gpio_put(LED3_GPIO, false);

                gpio_put(LED4_GPIO, true);
                vTaskDelay(loop_ticks/4);
                gpio_put(LED4_GPIO, false);

                gpio_put(LED5_GPIO, true);
                vTaskDelay(loop_ticks/4);
                gpio_put(LED5_GPIO, false);

                break;

            case DIMMER:
                if (ascending) {
                    target_val += 0.5;
                    if (target_val >= 100.0) ascending = false;
                } else {
                    target_val -= 0.5;
                    if (target_val <= 0.0) ascending = true;
                }
                pwm_set_duty_percent(target_val, &led1_pwm);
                pwm_set_duty_percent(target_val, &led2_pwm);
                pwm_set_duty_percent(target_val, &led3_pwm);
                pwm_set_duty_percent(target_val, &led4_pwm);
                pwm_set_duty_percent(target_val, &led5_pwm);
                vTaskDelay(pwm_ticks);

                break;

            case DIMMER_ALT:
                if (ascending) {
                    target_val += 0.5;
                    if (target_val >= 100.0) ascending = false;
                } else {
                    target_val -= 0.5;
                    if (target_val <= 0.0) ascending = true;
                }
                pwm_set_duty_percent(target_val, &led1_pwm);
                pwm_set_duty_percent(100-target_val, &led2_pwm);
                pwm_set_duty_percent(target_val, &led3_pwm);
                pwm_set_duty_percent(100-target_val, &led4_pwm);
                pwm_set_duty_percent(target_val, &led5_pwm);
                vTaskDelay(pwm_ticks);

                break;

            // case DIMMER_QUARTER:
            //     if (ascending) {
            //         target_val += 0.5;
            //         if (target_val >= 100.0) ascending = false;
            //     } else {
            //         target_val -= 0.5;
            //         if (target_val <= 0.0) ascending = true;
            //     }
            //     pwm_set_duty_percent(target_val, &led1_pwm);
            //     vTaskDelay(pwm_ticks/4);
            //     pwm_set_duty_percent(target_val, &led2_pwm);
            //     vTaskDelay(pwm_ticks/4);
            //     pwm_set_duty_percent(target_val, &led3_pwm);
            //     vTaskDelay(pwm_ticks/4);
            //     pwm_set_duty_percent(100-target_val, &led4_pwm);
            //     vTaskDelay(pwm_ticks);
            //     break;

            default:
                vTaskDelay(loop_ticks);
                break;
        }


    }
}


/* --------- ISR ---------- */

static void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_GPIO) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xButtonTaskHandle == NULL) {
            return;
        }

        vTaskNotifyGiveFromISR(xButtonTaskHandle, &xHigherPriorityTaskWoken);

        // If the task we woke has higher priority, do a context switch immediately
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



/* --------- Main ---------- */

int main() {
    stdio_init_all();

    // Initialize gpio LED0
    gpio_init(LED0_GPIO);
    gpio_set_dir(LED0_GPIO, GPIO_OUT);

    // Initialize gpio LED1
    gpio_init(LED1_GPIO);
    gpio_set_dir(LED1_GPIO, GPIO_OUT);

    // Initialize gpio LED2
    gpio_init(LED2_GPIO);
    gpio_set_dir(LED2_GPIO, GPIO_OUT);

    // Initialize gpio LED3
    gpio_init(LED3_GPIO);
    gpio_set_dir(LED3_GPIO, GPIO_OUT);

    // Initialize gpio LED4
    gpio_init(LED4_GPIO);
    gpio_set_dir(LED4_GPIO, GPIO_OUT);

    // Initialize gpio LED5
    gpio_init(LED5_GPIO);
    gpio_set_dir(LED5_GPIO, GPIO_OUT);

    // Initialize botton input with pull-up and falling-edge IRQ
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    /* --------- FreeRTOS Tasks ---------- */
    BaseType_t res;

    res = xTaskCreate(Blink0Task, "BLINK", 256, NULL, 1, NULL);
    configASSERT(res == pdPASS);

    res = xTaskCreate(LoopTask, "LOOP", 256, NULL, 2, &xButtonTaskHandle);
    configASSERT(res == pdPASS && xButtonTaskHandle != NULL);


    /* --------- FreeRTOS ISR ---------- */
    // Button ISR
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);



    vTaskStartScheduler();

    return 0;
}
