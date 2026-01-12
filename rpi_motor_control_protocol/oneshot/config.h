/* --------------------------- DEFINITIONS --------------------------- */

// MOTORS
#define MOTORS_NUM 4

// GPIO map
#define BUTTON_GPIO  14
#define JOY_X_GPIO   26
#define JOY_Y_GPIO   27

// FreeRTOS
#define ONESHOT_TASK_STACK_WORDS 1024
#define ONESHOT_TASK_PRIORITY 3u
#define MICROROS_TASK_STACK_WORDS 4096
#define MICROROS_TASK_PRIORITY 2u
#define QUEUE_SIZE 8

// PWM
#define PWM_WRAP        62499u  // PWM wrap/resolution, aka number of counts in one PWM cycle (Depends on MCU)
#define PWM_TARGET_FREQ 2000u   // desired PWM signal frequency [Hz]  (depends on ESC spec)
#define ESC_MIN_US      125u    // PWM pulse width [µs] for zero throttle (0%) on ESC -> STANDARD (depends on ESC spec)
#define ESC_MAX_US      250u    // PWM pulse width [µs] for full throttle (100%) on ESC -> STANDARD (depends on ESC spec)

// Joystick
#define ADC_MAX_RESOLUTION 4095u
#define JOY_THRESHOLD 0.25f
#define THROTTLE_STEP 5u

// Loop
#define LOOP_FREQ_HZ 500u  // Task loop frequency [Hz]

// Motor params
static const float THROTTLE_MIN = 0.0f;
static const float THROTTLE_MAX = 100.0f;
static const float THROTTLE_ARM = 18.0f;
