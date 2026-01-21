/* ---------------------------- GLOBAL OBJECTS ---------------------------- */

typedef struct {
    float throttle_min;
    float throttle_max;
} throttle_thresholds;


// PWM control structures
typedef struct {
    uint gpio;
    uint slice;
    uint channel;
    uint wrap;
    uint freq;
    uint esc_min_us;
    uint esc_max_us;
} pwm_control_t;


typedef struct {
    pwm_control_t* motors_pwm;
    uint num_pins;
    throttle_thresholds throttle_threshold;
} oneshot_task_params_t;


