/* --------------------------- DEFINITIONS --------------------------- */

// GPIO map
#define LED1_GPIO    PICO_DEFAULT_LED_PIN
#define LED2_GPIO    17
#define PWM_GPIO     15
#define BUTTON_GPIO  14
#define JOY_X_GPIO   26
#define JOY_Y_GPIO   27

// FreeRTOS
#define BLINK_TASK_STACK_WORDS 256
#define PWM_TASK_STACK_WORDS 1024

// PWM
// - MCU
#define PWM_WRAP        62499u  // PWM wrap/resolution, aka number of counts in one PWM cycle (Depends on MCU)
// - ESC
#define PWM_TARGET_FREQ 2000u    // desired PWM signal frequency [Hz]  (depends on ESC spec)
#define PWM_TARGET_PERIOD_US (1000000.0f / PWM_TARGET_FREQ)    // PWM period [µs]
#define ESC_MIN_US      125u   // PWM pulse width [µs] for zero throttle (0%) on ESC -> STANDARD (depends on ESC spec)
#define ESC_MAX_US      250u   // PWM pulse width [µs] for full throttle (100%) on ESC -> STANDARD (depends on ESC spec)

// Joystick
#define ADC_MAX_RESOLUTION 4095u
#define JOY_THRESHOLD 0.25f
#define THROTTLE_STEP 5u

// Loop
#define LOOP_FREQ_HZ 500u  // Task loop frequency [Hz]

// Motor params
static const float THROTTLE_MIN = 0.0f;
static const float THROTTLE_MAX = 100.0f;
