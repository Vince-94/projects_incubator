/*
 * motor_control.c
 *
 * RP2350 (Pico 2) - FreeRTOS motor control demo
 *
 * Workflow:
 *   1. An ISR read the encoder signals at ENC_PIN and increment an atomic variable
 *   2. At each 100ms, the pulse are read by a task and the counter reset
 *   3. Convert encoder counts into RPM and add a small low-pass filter
 *   4. Apply PID controller
 *   5. PMW conversion
 *
 * Purpose:
 *  - First incremental feature for the motor-control project.
 *  - Configure PWM pin, expose pwm_set_duty_percent(), and run a safe low-amplitude
 *    "ramp" task that moves duty between 0% and STARTUP_SAFE_DUTY.
 *  - Centralized logger task prints status over USB CDC.
 *
 * How to use:
 *  - Build & flash to the Pico 2. Open the serial console.
 *  - You should see periodic logs showing duty changes.
 *  - Verify with a multimeter/LED (through resistor) or better: logic analyzer/oscilloscope on PWM_PIN.
 *
 * Notes:
 *  - No motor connected while testing. Keep everything off load until later steps.
 *  - This file intentionally implements ONE feature: robust PWM output + simple ramp test.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "src/pid.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* ------------------------- Definitions ------------------------- */

#define ENC_PIN           16          // encoder / hall input pin (adjust as needed)
#define PWM_PIN           18          /* PWM output pin to motor/driver */
#define ENABLE_BTN_PIN    14          /* active-low enable/disable button pin */

#define PULSES_PER_REV    20u         // encoder pulses per mechanical revolution
#define CONTROL_FREQ_HZ   100u        // control loop frequency (Hz)
#define PID_DT_SECONDS    (1.0f / (float)CONTROL_FREQ_HZ)  // control loop sample time: 1/f

#define PWM_TARGET_FREQ   20000.0f    // PWM frequency (Hz)
#define PWM_WRAP          1000u       // PWM wrap/resolution

#define STARTUP_SAFE_DUTY  5.0f       // percent duty used immediately when enabling
#define LOG_QUEUE_SIZE 8

// PID constants
static float Kp = 0.8f;
static float Ki = 0.4f;
static float Kd = 0.01f;
static const float I_MAX = 2000.0f;
static const float u_MIN = 0.0f;
static const float u_MAX = 100.0f;

static const float DUTY_MIN = 0.0f;
static const float DUTY_MAX = 95.0f;


/* --------- RTOS objects (file-scope so ISR and tasks can access) --------- */

typedef struct { char text[96]; } LogMsg_t;

static QueueHandle_t xLogQueue = NULL;

static TaskHandle_t xControlTaskHandle = NULL;


/* ------------------------ Global variables ------------------------ */

// volatile to indicate shared with ISR; use atomic builtins for safe ops
static volatile uint32_t encoder_count = 0u;


/* ------------------------ Global functions ------------------------ */

// Increment the encoder counter (called from ISR)
static inline void atomic_inc_encoder(void) {
    __atomic_add_fetch(&encoder_count, 1u, __ATOMIC_RELAXED);
}


// Atomically read and clear the encoder counter (called from task)
static inline uint32_t atomic_read_and_clear_encoder(void) {
    return __atomic_exchange_n(&encoder_count, 0u, __ATOMIC_ACQ_REL);
}


/* ------------------------ Helpers ------------------------ */
static uint pwm_slice;
static uint pwm_chan;

static void pwm_setup_pin(uint pin, uint wrap, float target_freq) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(pin);
    pwm_chan = pwm_gpio_to_channel(pin);

    // compute divider
    const uint32_t sys_clk = clock_get_hz(clk_sys);
    float divider = (float)sys_clk / ((float)(wrap) * target_freq);
    if (divider < 1.0f) divider = 1.0f;
    pwm_set_clkdiv(pwm_slice, divider);
    pwm_set_wrap(pwm_slice, wrap);
    pwm_set_chan_level(pwm_slice, pwm_chan, 0);
    pwm_set_enabled(pwm_slice, true);
}


static void pwm_set_duty_percent(float duty_percent) {
    if (duty_percent <= 0.0f) {
        pwm_set_chan_level(pwm_slice, pwm_chan, 0);
        return;
    }
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t wrap = pwm_hw->slice[pwm_slice].top;  // <-- fix here

    uint32_t level = (uint32_t)roundf((duty_percent / 100.0f) * (float)(wrap + 1));
    if (level > wrap) level = wrap;

    pwm_set_chan_level(pwm_slice, pwm_chan, level);
}


// Map RPM to duty-cycle (linear mapping)
static float map_rpm_duty(const float u) {
    // Normalize u [0~1]
    float u_norm = (u - u_MIN) / (u_MAX - u_MIN);

    float duty = DUTY_MIN + u_norm * (DUTY_MAX - DUTY_MIN);
    return duty;
}


/* ------------------------ ISR ------------------------ */

void gpio_global_callback(uint gpio, uint32_t events) {

    // ISR encoder callback that receives signals from encoder
    if (gpio == ENC_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            atomic_inc_encoder();
        }
    }

    // ISR button callback that enables the controller
    if (gpio == ENABLE_BTN_PIN) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xControlTaskHandle != NULL) {
            // Notify task from ISR
            vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
            // If the new task has higher priority, then execute it
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}


/* ------------------------ Task ------------------------ */

// RPM converter task
static void control_task(void* params) {

    TickType_t last_wake = xTaskGetTickCount();

    // Sample time
    const TickType_t period_ticks = pdMS_TO_TICKS((uint32_t)(1000u / CONTROL_FREQ_HZ));
    const float Ts = PID_DT_SECONDS;

    // Constants
    bool motor_enabled = false;
    float rpm_filtered = 0.0f;
    int log_counter = 0;
    const int log_interval = CONTROL_FREQ_HZ / 5; /* ~5 logs/sec */

    // Parameters
    float rpm_filter_alpha = 0.3f;   // can be set via CLI
    float target_rpm = 0.0f;   // can be set via CLI

    // Setup PID
    PIDController motor_pid;
    PID_Init(&motor_pid, Kp, Ki, Kd, u_MIN, u_MAX);
    PID_SetSetpoint(&motor_pid, target_rpm);

    // Setup PWM
    pwm_setup_pin(PWM_PIN, PWM_WRAP, PWM_TARGET_FREQ);
    pwm_set_duty_percent(0.0f);  // initially set to 0

    while(true) {
        // Wait until the controller is enable by pressing the button (enable_btn_callback)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, period_ticks);

        vTaskDelayUntil(&last_wake, period_ticks);

        if (notified > 0) {
            // TODO add a debouncing mechanism

            // Toggel motor_enabled state when buttin is pressed
            motor_enabled = !motor_enabled;

            // logging messages
            LogMsg_t m;
            int n = snprintf(m.text, sizeof(m.text), "CTRL: motor %s\r\n", motor_enabled ? "ENABLED" : "DISABLED");
            if (n < 0) strncpy(m.text, "CTRL: snprintf error\r\n", sizeof(m.text));
            xQueueSend(xLogQueue, &m, 0);

            if (motor_enabled) {
                pwm_set_duty_percent(STARTUP_SAFE_DUTY);
            } else {
                // Stop controller task
                pwm_set_duty_percent(0.0f);
                rpm_filtered = 0.0f;
                PID_Reset(&motor_pid);
            }
        }

        // Atomically read and clear the counter occurred in the last sample
        uint32_t counts = atomic_read_and_clear_encoder();

        // If motor is disabled, skip
        if (!motor_enabled) {
            log_counter++;
            if (log_counter >= log_interval) {
                log_counter = 0;
                LogMsg_t m;
                snprintf(m.text, sizeof(m.text), "CTRL: disabled  counts=%u  rpm=%.1f\r\n", counts, rpm_filtered);
                xQueueSend(xLogQueue, &m, 0);
            }
            continue;
        }

        // Conver counts to rpm
        float revolutions = (float)counts / (float)PULSES_PER_REV;

        // Instantaneous RPM (may be noisy)
        float rpm_measured = (revolutions / Ts) * 60.0f;  // rev/sec -> RPM

        // Exponential moving average filter: f = (1 - alfa) * RMP_f + alfa * RMP_m
        rpm_filtered = (1.0f - rpm_filter_alpha) * rpm_filtered + rpm_filter_alpha * rpm_measured;

        // Print counts, raw RPM, filtered RPM
        printf("[RPM] counts=%u  raw=%.1f  filtered=%.1f RPM\r\n", counts, rpm_measured, rpm_filtered);

        // Compute PID
        float u = PID_Compute(&motor_pid, rpm_filtered, Ts);

        // Compute PWM
        float duty = map_rpm_duty(u);
        pwm_set_duty_percent(duty);

        // Logging
        log_counter++;
        if (log_counter >= log_interval) {
            log_counter = 0;
            LogMsg_t m;
            int n = snprintf(m.text, sizeof(m.text),
                             "CTRL: RPM=%.1f target=%.1f counts=%u duty=%.1f%%\r\n",
                             rpm_filtered, target_rpm, counts, duty);
            if (n < 0) strncpy(m.text, "CTRL: snprintf error\r\n", sizeof(m.text));
            xQueueSend(xLogQueue, &m, 0);
        }

    }
}


static void logger_task(void* params) {
    LogMsg_t m;

    while(true) {
        if (xQueueReceive(xLogQueue, &m, portMAX_DELAY) == pdTRUE) {
            printf("%s", m.text);
        }
    }
}



/* ------------------------- Main ------------------------- */
int main() {
    // Initialization
    stdio_init_all();

    // Initialize encoder gpio as input
    gpio_init(ENC_PIN);
    gpio_set_dir(ENC_PIN, GPIO_IN);
    gpio_pull_down(ENC_PIN);

    // Enable button pin (active-low)
    gpio_init(ENABLE_BTN_PIN);
    gpio_set_dir(ENABLE_BTN_PIN, GPIO_IN);
    gpio_pull_up(ENABLE_BTN_PIN);

    // Create log queue
    xLogQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMsg_t));
    configASSERT(xLogQueue != NULL);

    // Tasks
    BaseType_t res;

    res = xTaskCreate(control_task, "CTRL", 1024, NULL, tskIDLE_PRIORITY + 2, &xControlTaskHandle);
    configASSERT(res == pdPASS && xControlTaskHandle != NULL);

    res = xTaskCreate(logger_task, "LOG", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    configASSERT(res == pdPASS);

    // ISR
    gpio_set_irq_enabled_with_callback(ENC_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_global_callback);

    gpio_set_irq_enabled_with_callback(ENABLE_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_global_callback);

    // Start the scheduler
    vTaskStartScheduler();

    return 0;
}