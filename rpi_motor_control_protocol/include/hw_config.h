// RPi Pico 2 (RP235)
#define NUM_PWM_PINS 16

static const uint32_t PWM_GPIOS[NUM_PWM_PINS] = {
    2, 3,   // slice 1
    4, 5,   // slice 2
    6, 7,   // slice 3
    8, 9,   // slice 4
    10, 11, // slice 5
    12, 13, // slice 6
    14, 15  // slice 7
};

