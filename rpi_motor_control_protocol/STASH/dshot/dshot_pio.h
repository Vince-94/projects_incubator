/*
 * dshot_pio.c
 *
 * It's an upgrade of the bit-bang DSHOT, where the critical section is moved to from the CPU to the hw state machine,
 * using PIO (programmable I/O coprocessor) that execute instructions at deterministic time.
 *
 * Properties:
 * - DSHOT baud [bits/s]: baud = 300 kbaud -> period: T_us = 1e6 / baud
 * - DSHOT frame send frequency: how often you send whole 16-bit frames (update rate)
 * - System clock `sys_clk`: the MCU system clock (CPU/peripheral clock)
 * - PIO frequency `pio_freq_hz`: how fast the PIO state machine executes instructions, after applying the SM clock divider: pio_freq_hz = sys_clk / clkdiv
 * - PIO cycle time `pio_cycle_us`: pio_cycle_us = 1e6 / pio_freq_hz [µs]
 * - High/low durations for a DSHOT bit encoding
 *   - bit = 1 -> is encoded by driving the line HIGH for 75% of Ts (≈5.0 µs), LOW for the remainder (~1.67 µs).
 *   - bit = 0 -> is encoded by driving the line HIGH for 37.5% of Ts (≈2.5 µs), LOW for the remainder (~4.17 µs).
 * - PIO-counts: integer counts you push to the PIO that represent how many PIO cycles to wait: cycles_count = round(duration_us / pio_cycle_us)
 * - 16-bit packet:
 *   - Bits 15~5: 11-bit value (throttle or command)
 *   - Bit 4: telemetry request (0 or 1)
 *   - Bits 3~0: 4-bit checksum — computed by XOR of the three 4-bit nibbles of the 12-bit payload
 * 
 * Workflow:
 * - C:
 *   0. Initialization:
 *      - [set_dshot_timings] compute dshot timings (in microseconds) according to the baudrate using the bit encoding
 *   1. For each DSHOT bit
 *      1. compute two timed phases (HIGH and LOW) according the bit encoding:
 *        - high_cycles = round(high_us / pio_cycle_us)
 *        - low_cycles = round(low_us / pio_cycle_us)
 *      2. push the HIGH and LOW phases into the PIO TX FIFO*
 *      5. repeat for all the 16 bit of the frame
 * - PIO:
 *   1. waits for a 32-bit number from its TX FIFO, uses that value as a loop count, and toggles the output pin while “counting down”
 *     1. pulls the first number, sets the pin HIGH, spends that many PIO cycles spinning (so the pin stays HIGH for that period)
 *     2. pulls the second number, sets the pin LOW, and spins again
 */
#ifndef DSHOT_PIO_H
#define DSHOT_PIO_H

#include <stdin.h>
#include <stdbool.h>

#include "dshot.pio.h"


//---------------------------- Global section ----------------------------//

// DHSOT values range
#define DSHOT_COMMAND_MIN  0u                           // 0~47 reserved for commands on many ESCs
#define DSHOT_THROTTLE_MIN 48u
#define DSHOT_THROTTLE_MAX 2047u

// RTOS resources and defaults
#define DSHOT_TASK_STACK_WORDS 512
#define DSHOT_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)
#define DSHOT_COMMAND_Q_SIZE   8
#define DSHOT_THROTTLE_Q_SIZE  1
#define DSHOT_UPDATE_FREQ 100u                          // Frequency of the dshot_task
#define DSHOT_DEFAULT_BAUD 300000u                      // 150k/300k supported


//---------------------------- Functions prototype ----------------------------//

// Convert percentage [0~100] to DSHOT throttle value [48~2047] through linear mapping
static uint16_t throttle_percent_to_dshot(float percent);

// Build 16-bit DSHOT packet
static uint16_t build_dshot_packet(uint16_t throttle, bool request_telemetry);

// Configure PIO SM
static bool configure_pio_sm(uint pin, uint pio_sm_offset);

// Configure and compute PIO timing resolution
static void configure_pio_clock(float pio_target_freq_hz);

// Compute DSHOT timings encoding
static void set_dshot_timings(uint baud);

// Convert µs -> PIO cycles (float -> rounded uint32)
static inline uint32_t us_to_pio_cycles(float us);

// Push a DSHOT frame to the PIO SM by writing high/low cycle counts for each bit.
static void pio_dshot_send_frame(uint16_t packet);


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
bool dshot_init(uint pin, float baud);

/**
 * @brief Deinitialize DSHOT
 * 
 * @param pin 
 * @param baud 
 */
void dshot_deinit();

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
bool dshot_request_telemetry_continuous(bool enable, uint16_t frequency);


#endif // DSHOT_PIO_H