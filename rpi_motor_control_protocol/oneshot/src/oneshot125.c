#include <string.h>
#include <stdio.h>
// #include <stdin.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>  // dynamic allocation (malloc)

#include "oneshot125.h"


//---------------------- PRIVATE API ----------------------//

void pwm_setup_pin(pwm_control_t* pwm, uint pin, uint wrap, float target_freq) {
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


void pwm_set_duty_percent(pwm_control_t* pwm, float duty_percent) {
    if (duty_percent <= 0.0f) {
        pwm_set_chan_level(pwm->slice, pwm->channel, 0);
        return;
    }
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t level = (uint32_t)roundf((duty_percent / 100.0f) * (float)(pwm->wrap + 1));
    if (level > pwm->wrap) level = pwm->wrap;

    pwm_set_chan_level(pwm->slice, pwm->channel, level);
}



//---------------------- PUBLIC API ----------------------//

int* getGpio(int requested_gpio, const uint32_t* gpio_list, int gpio_list_length) {
    if (requested_gpio > gpio_list_length) {
        printf("[ERROR] Requested %d GPIOs, but only %d available.\n", requested_gpio, gpio_list_length);
        return NULL;
    }

    int* requested_gpio_list = malloc(requested_gpio * sizeof(int));
    if (!requested_gpio_list) {
        printf("[ERROR] Memory allocation failed\n");
        return NULL;
    }

    for (int i = 0; i < requested_gpio; i++) {
        requested_gpio_list[i] = gpio_list[i];
    }

    return requested_gpio_list;
}


bool setConfig(oneshot_task_params_t* oneshot125_params, const int* gpio, int pwm_pins_requested, uint pwm_wrap, uint pwm_freq, uint esc_min_us, uint esc_max_us, float throttle_min, float throttle_max) {

    // static pwm_control_t motors_pwm[pwm_pins_requested];
    pwm_control_t* motors_pwm = malloc(pwm_pins_requested * sizeof(pwm_control_t));  // TODO malloc
    if (motors_pwm == NULL) {
        printf("[ERROR] Failed to allocate memory for motors_pwm\n");
        return false; // Handle allocation failure
    }

    // Initialize the motors_pwm array
    for (int i = 0; i < pwm_pins_requested; i++) {
        motors_pwm[i].gpio = (uint32_t)gpio[i];
        motors_pwm[i].wrap = pwm_wrap;
        motors_pwm[i].freq = pwm_freq;
        motors_pwm[i].esc_min_us = esc_min_us;
        motors_pwm[i].esc_max_us = esc_max_us;
        motors_pwm[i].slice = 0;
        motors_pwm[i].channel = 0;
    }

    oneshot125_params->motors_pwm = motors_pwm;
    oneshot125_params->num_pins = (uint32_t)pwm_pins_requested;
    oneshot125_params->throttle_threshold.throttle_min = throttle_min;
    oneshot125_params->throttle_threshold.throttle_max = throttle_max;

    return true;
}


void initPwm(pwm_control_t* motors_pwm, uint32_t pwm_pins) {
    for (uint i=0; i<pwm_pins; i++) {
        pwm_setup_pin(&motors_pwm[i], motors_pwm[i].gpio, motors_pwm[i].wrap, (float)motors_pwm[i].freq);
    }
}


float map_throttle_duty(const pwm_control_t* pwm, float throttle, throttle_thresholds throttle_threshold) {
    // Normalize throttle [0~1]
    float throttle_norm = (throttle - throttle_threshold.throttle_min) / (throttle_threshold.throttle_max - throttle_threshold.throttle_min);

    // Map to pulse width in microseconds
    float pulse_us = (float)pwm->esc_min_us + throttle_norm * (float)(pwm->esc_max_us - pwm->esc_min_us);

    // Convert to duty cycle percentage based on PWM period
    // dt = (pulse_us / pwm_period_us) * 100
    float duty_percent = (pulse_us * (float)pwm->freq) / 10000.0;
    return duty_percent;
}


void setDutyCycle(pwm_control_t* motors_pwm, uint32_t pwm_pins, const float* duty_percent) {
    for (uint32_t i=0; i<pwm_pins; i++) {
        pwm_set_duty_percent(&motors_pwm[i], duty_percent[i]);
        // printf("DT[%u] = %f, ", (unsigned)i, duty_percent[i]);
    }
    // printf("\n");
}
