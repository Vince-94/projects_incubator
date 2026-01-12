/**
 * @file oneshot125.h
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
#ifndef ONESHOT125_H
#define ONESHOT125_H

// Common libs
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// RPi lib
#include "pico/stdlib.h"
// #include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
// #include "hardware/adc.h"

#include "data_structures.h"


//---------------------- PRIVATE API ----------------------//

/**
 * @brief 
 * 
 * @param pwm 
 * @param pin 
 * @param wrap 
 * @param target_freq 
 */
void pwm_setup_pin(pwm_control_t* pwm, uint pin, uint wrap, float target_freq);


/**
 * @brief 
 * 
 * @param pwm 
 * @param duty_percent 
 */
void pwm_set_duty_percent(pwm_control_t* pwm, float duty_percent);



//---------------------- PUBLIC API ----------------------//

/**
 * @brief Get the list of GPIOs requested
 * 
 * @param requested_gpio 
 * @param gpio_list 
 * @param gpio_list_length 
 * @return int* 
 */
int* getGpio(int requested_gpio, const uint32_t* gpio_list, int gpio_list_length);


/**
 * @brief Set the oneshot125_params
 * 
 * @param oneshot125_params 
 * @param gpio 
 * @param pwm_pins_requested 
 * @param pwm_wrap 
 * @param pwm_freq 
 * @param esc_min_us 
 * @param esc_max_us 
 * @param throttle_min 
 * @param throttle_max 
 * @return true 
 * @return false 
 */
bool setConfig(oneshot_task_params_t* oneshot125_params, const int* gpio, int pwm_pins_requested, uint pwm_wrap, uint pwm_freq, uint esc_min_us, uint esc_max_us, float throttle_min, float throttle_max);


/**
 * @brief Initialize the Pwm object
 * 
 * @param motors_pwm 
 * @param pwm_pins 
 */
void initPwm(pwm_control_t* motors_pwm, uint32_t pwm_pins);


/**
 * @brief Map input throttle into duty-cycle.
 * 1. Normalize throttle [0~1]
 * 2. Mapping throttle to pulse in µs
 * 
 * @param pwm 
 * @param throttle 
 * @param throttle_threshold 
 * @return float 
 */
float map_throttle_duty(const pwm_control_t* pwm, float throttle, throttle_thresholds throttle_threshold);


/**
 * @brief Set the Duty Cycle object
 * 
 * @param motors_pwm 
 * @param pwm_pins 
 * @param duty_percent 
 */
void setDutyCycle(pwm_control_t* motors_pwm, uint32_t pwm_pins, const float* duty_percent);



#endif /* ONESHOT125_H */
