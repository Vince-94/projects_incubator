/* --------------------------- DEFINITIONS --------------------------- */

// GPIO map
#define LED1_GPIO    PICO_DEFAULT_LED_PIN
#define LED2_GPIO    17
#define PWM_GPIO     15
#define BUTTON_GPIO  14
#define ENCODER_GPIO  16
// #define JOY_X_GPIO   26
// #define JOY_Y_GPIO   27

// FreeRTOS
#define BLINK_TASK_STACK_WORDS 256
#define PWM_TASK_STACK_WORDS 1024

// PWM
// - MCU
#define PWM_WRAP        (20000u - 1)  // PWM wrap/resolution, aka number of counts in one PWM cycle (Depends on MCU)
// - ESC
#define PWM_TARGET_FREQ 50u    // desired PWM signal frequency [Hz]  (depends on ESC spec)
#define PWM_TARGET_PERIOD_US (1000000.0f / PWM_TARGET_FREQ)    // PWM period [µs]
#define ESC_MIN_US      1000u  // PWM pulse width [µs] for zero throttle (0%) on ESC -> STANDARD (depends on ESC spec)
#define ESC_MAX_US      2000u  // PWM pulse width [µs] for full throttle (100%) on ESC -> STANDARD (depends on ESC spec)

// Joystick
#define ADC_MAX_RESOLUTION 4095u

// Encoder
#define PULSES_PER_REV 20u  // encoder pulses per mechanical revolution

// Loop
#define LOOP_FREQ_HZ 50u  // Task loop frequency [Hz]

// Motor params
static const float THROTTLE_MIN = 0.0f;
static const float THROTTLE_MAX = 100.0f;

// PID params
static float Kp = 0.1f;
static float Ki = 0.0f;
static float Kd = 0.0f;
