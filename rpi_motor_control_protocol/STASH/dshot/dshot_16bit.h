/*
 * dshot_16bit.c
 *
 * Minimal learning-first DSHOT sender for RP2350 (Pico 2).
 * - DSHOT150 bit-banged frames
 * - FreeRTOS task sends frames periodically
 * - Simple queue API to update throttle (percent)
 *
 * Properties:
 * - Frequency: f = 150 kHz
 * - Period: Ts = 1/f = 6.666 µs
 * - 16-bit packet:
 *   - Bits 15~5: 11-bit value (throttle or command)
 *   - Bit 4: telemetry request (0 or 1)
 *   - Bits 3~0: 4-bit checksum — computed by XOR of the three 4-bit nibbles of the 12-bit payload
 * - Bit value:
 *   - 1 -> is encoded by driving the line HIGH for 75% of Ts (≈5.0 µs), LOW for the remainder (~1.67 µs).
 *   - 0 (LOW) -> is encoded by driving the line HIGH for 37.5% of Ts (≈2.5 µs), LOW for the remainder (~4.17 µs).
 * - Total frame period: T = Ts * 16 = 107 µs
 *
 * Workflow:
 *   1. Map the desired throttle (0~100%) into a DSHOT value value (0~2047)
 *   2. Build the 16-bit packet
 *   3. For each outgoing bit (MSB → LSB):
 *      - Drive the output GPIO HIGH and busy-wait for the `high` duration
 *      - Drive the output GPIO LOW and busy-wait for the remaining duration
 *
 * Notes:
 * - To avoid interrupts (creating jitter), disable preemption/interrupts for that tiny window using taskENTER_CRITICAL() / taskEXIT_CRITICAL() around the bit-bang
 * - dshot_task runs at a comparatively high priority and sends frames periodically
 */
// TODO: reserved commands repeated send feature
// TODO: PID for throttle commands
// TODO: for continuous telemetry add a rate
#ifndef DSHOT_16BIT_H
#define DSHOT_16BIT_H
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


//---------------------------- Global section ----------------------------//

#define DSHOT_COMMAND_MIN  0u                           // 0~47 reserved for commands on many ESCs
#define DSHOT_THROTTLE_MIN 48u
#define DSHOT_THROTTLE_MAX 2047u

#define DSHOT_DEFAULT_BAUD 150000u                      // 150 kHz
#define DSHOT_FREQ 100u                                 // Many FCs send frames every 2-20 ms; 10ms == 100Hz is a safe start.

/* RTOS resources & defaults */
#define DSHOT_TASK_STACK_WORDS 512
#define DSHOT_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)
#define DSHOT_COMMAND_Q_SIZE   8
#define DSHOT_THROTTLE_Q_SIZE  1


static uint dshot_pin = 0;
static uint dshot_baud = DSHOT_DEFAULT_BAUD;

static volatile uint8_t dshot_request_telemetry_flag = 0u;
static volatile uint8_t dshot_telemetry_continuous = 0u;
static volatile uint16_t telemetry_freq = 100;  // Hz
static TickType_t dshot_telemetry_interval_ticks = 0;

// per-bit timings (microseconds, integer) computed for chosen baud
static uint32_t t_bit_us = 0;
static uint32_t t_high_1_us = 0;
static uint32_t t_high_0_us = 0;
static uint32_t t_low_1_us = 0;
static uint32_t t_low_0_us = 0;

// Freertos allocation
static QueueHandle_t xDshotThrottleQueue = NULL;
static QueueHandle_t xDshotCommandQueue = NULL;
static TaskHandle_t  xDshotTaskHandle = NULL;


//---------------------------- Functions prototype ----------------------------//

// Convert percentage [0~100] to DSHOT throttle value [48~2047] through linear mapping
static uint16_t throttle_percent_to_dshot(float percent);

// Timing computation
static void set_dshot_timings(float baud);

// Build 16-bit DSHOT packet
static uint16_t build_dshot_packet(uint16_t throttle, bool request_telemetry);

// Send frames by toggling the pin value and using a busy-wait delay. It must be called from a short critical region to avoid jitter.
static void dshot_send_frame_bitbang(uint16_t packet, uint pin);


//---------------------------------- RTOS ----------------------------------//

// Periodically sends the last throttle value
static void dshot_task(void* params);


//------------------------------- Public API -------------------------------//

/**
 * @brief Initialize module and start RTOS task.
 * 
 * @param pin GPIO pin used for DSHOT output (push-pull output)
 * @param baud choose DSHOT150/300/600 etc (here we support 150 reliably with bit-bang).
 * @return bool 
 */
bool dshot_init(uint pin, uint baud);

/**
 * @brief Update throttle in pertantage [0~100]. Overwrites queue latest value.
 * 
 * @param percent 
 * @return bool 
 */
bool dshot_set_throttle_percent(float percent);

/**
 * @brief Set raw throttle value [0~2047]
 * 
 * @param raw input value
 * @return bool 
 */
bool dshot_set_throttle_raw(uint16_t raw);

/**
 * @brief Request telemetry once
 * 
 * @return bool 
 */
bool dshot_request_telemetry_once(void);

/**
 * @brief Request telemetry at fixed rate
 * 
 * @return bool 
 */
bool dshot_request_telemetry_continuous(bool enable, uint32_t period_ms);


#endif // DSHOT_16BIT_H