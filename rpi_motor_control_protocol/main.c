// Common libs
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Rpi libs
#include "pico/stdlib.h"
// #include "hardware/adc.h"

// FreeRTOS libs
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// micro-ROS libs
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <rcutils/logging_macros.h>
// #include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>

// Custom libs
#include "hw_config.h"
#include "oneshot/oneshot125.h"
#include "oneshot/config.h"


/* ------------------------------ GLOBAL OBJECTS ------------------------------ */

// Queue element used by OneShotTask and MicroRosTask
typedef struct {
    uint8_t count;                  // Number of motors
    float throttles[NUM_PWM_PINS];  // Array of throttles (malloc)
} throttle_msg_t;

// Parameters passed to the MicroRosTask
typedef struct {
    QueueHandle_t queue;
    uint8_t motor_count;
} micro_ros_params_t;

// FreeRTOS handle
static TaskHandle_t xControlTaskHandle = NULL;  // task handle
static QueueHandle_t xThrottleQueue = NULL;  // queue handle


/* -------------------------------- CALLBACKS -------------------------------- */

// micro-ROS subscriber callback
void throttle_sub_cb(const void* msgin) {
    if (msgin == NULL) return;

    const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    // Build throttle message to send to oneshot library
    throttle_msg_t throttle_msg;

    // ROS incoming message size
    size_t msg_size = msg->data.size;

    if (msg_size != MOTORS_NUM) {
        RCUTILS_LOG_WARN_NAMED("micro_ros_node",
            "Throttle message length (%zu) != expected motors (%zu)",
            msg_size, MOTORS_NUM
        );
        return;
    }

    // Fast copy of contiguous floats (Float32MultiArray uses contiguous float array)
    memcpy(throttle_msg.throttles, msg->data.data, msg_size * sizeof(float));

    // Enqueue to oneshot queue (non-blocking)
    if (xThrottleQueue == NULL) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "xThrottleQueue is NULL (oneshot not running?)");
        return;
    }

    BaseType_t ok = xQueueSend(xThrottleQueue, &throttle_msg, 0);
    if (ok != pdTRUE) {
        RCUTILS_LOG_WARN_NAMED("micro_ros_task", "xThrottleQueue full, dropping throttle message");
    }
}


/* ------------------------------- FREERTOS ------------------------------- */

void OneShotTask(void* params) {
    // Unpack params
    oneshot_task_params_t* task_params = (oneshot_task_params_t*)params;

    pwm_control_t* motors_pwm = task_params->motors_pwm;  // TODO Prefer static arrays over malloc
    uint pwm_pins = task_params->num_pins;
    throttle_thresholds throttle_threshold = task_params->throttle_threshold;

    // Setup PWM and set duty cycle to 0
    initPwm(motors_pwm, pwm_pins);
    float motor_duty[pwm_pins];
    for (uint i=0; i<pwm_pins; i++) {
        motor_duty[i] = map_throttle_duty(motors_pwm, THROTTLE_MIN, throttle_threshold);
    }
    setDutyCycle(motors_pwm, pwm_pins, motor_duty);

    // Get tick
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop_period_ms = (uint32_t)(1000u / LOOP_FREQ_HZ);
    const TickType_t loop_ticks = pdMS_TO_TICKS(loop_period_ms);

    bool arm = false;

    while(true) {
        // Delay
        vTaskDelayUntil(&last_wake, loop_ticks);

        throttle_msg_t msg;

        // Manual ARM/DISARM
        uint32_t notified = ulTaskNotifyTake(pdTRUE, loop_ticks);
        if (notified > 0) {
            float throttle;
            if (arm == false) {
                throttle = THROTTLE_ARM;
                arm = true;
            } else {
                throttle = THROTTLE_MIN;
                arm = false;
            }

            for (uint i=0; i<pwm_pins; i++) {
                motor_duty[i] = map_throttle_duty(motors_pwm, throttle, throttle_threshold);
                printf("throttle = %f, dt = %f\n", throttle, motor_duty[i]);
            }
            setDutyCycle(motors_pwm, pwm_pins, motor_duty);
            printf("[DEBUG] Arm motors!\n");
            continue;
        }

        // Drain all messages present (non-blocking)
        while (xQueueReceive(xThrottleQueue, &msg, 0) == pdTRUE) {
            if (msg.count == 0) continue; // ignore empty

            // Check whenever the messages number does not coincide with the requested signals
            if (msg.count != pwm_pins) {
                printf("[WARN] throttle msg count %d != pwm_pins %d\n", msg.count, pwm_pins);
                continue;
            }

            // Clamping PWM
            for (uint i = 0; i < pwm_pins; ++i) {
                float throttle = msg.throttles[i];
                if (throttle < THROTTLE_MIN) throttle = THROTTLE_MIN;
                if (throttle > THROTTLE_MAX) throttle = THROTTLE_MAX;

                // Compute duty-cycle
                motor_duty[i] = map_throttle_duty(motors_pwm, throttle, throttle_threshold);
            }
        }

        // Send duty-cycle
        // setDutyCycle(motors_pwm, pwm_pins, motor_duty);
    }
}


void MicroRosTask(void* arg) {
    (void)arg;

    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_subscription_t sub;
    rclc_executor_t executor;
    std_msgs__msg__Float32MultiArray msg;

    // Initialize default support (argc=0)
    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "rclc_support_init failed: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    // Init node
    rc = rclc_node_init_default(&node, "oneshot_controller_node", "", &support);
    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "rclc_node_init_default failed: %d", rc);
        rclc_support_fini(&support);
        vTaskDelete(NULL);
        return;
    }

    // Init subscription
    rc = rclc_subscription_init_default(
        &sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motors/throttle"
    );
    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "subscription init failed: %d", rc);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        vTaskDelete(NULL);
        return;
    }

    // Executor
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "executor init failed: %d", rc);
        rcl_subscription_fini(&sub, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        vTaskDelete(NULL);
        return;
    }

    // Init message storage and add subscription callback
    std_msgs__msg__Float32MultiArray__init(&msg);
    rc = rclc_executor_add_subscription(&executor, &sub, &msg, &throttle_sub_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED("micro_ros_task", "executor add subscription failed: %d", rc);
        std_msgs__msg__Float32MultiArray__fini(&msg);
        rclc_executor_fini(&executor);
        rcl_subscription_fini(&sub, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        vTaskDelete(NULL);
        return;
    }

    RCUTILS_LOG_INFO_NAMED("micro_ros_task", "micro-ROS task initialized, spinning executor");

    // Task infinite loop
    const uint32_t spin_period_ms = 10;  // adjust as needed
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(spin_period_ms));
        vTaskDelay(pdMS_TO_TICKS(spin_period_ms));
    }
}

void gpio_global_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_GPIO) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xControlTaskHandle == NULL) {
            return;
        }

        // Notify task from ISR
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);

        // If the new task has higher priority, then execute it
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/* ------------------------------- MAIN ------------------------------- */

int main() {
    // Enable stdio
    stdio_init_all();
    // sleep_ms(10000);
    printf("OneShot initialization...\n");

    // Select the GPIOs from hw_config.h
    int* gpio = getGpio((int)MOTORS_NUM, PWM_GPIOS, NUM_PWM_PINS);
    if (!gpio) return 1;

    printf("GPIO list: ");
    for (int i = 0; i < MOTORS_NUM; i++) {
        printf("[%d] %d , ", i, gpio[i]);
    }
    printf("\n");

    // Initialize gpio BUTTON
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    // FreeRTOS queues
    xThrottleQueue = xQueueCreate(QUEUE_SIZE, sizeof(throttle_msg_t));
    if (xThrottleQueue == NULL) {
        printf("[ERROR] Failed to create throttle queue\n");
        free(gpio);
        return -1;
    }

    // FreeRTOS tasks
    BaseType_t res;

    // - OneShot task
    static oneshot_task_params_t oneshot125_params;
    bool status = setConfig(&oneshot125_params, gpio, MOTORS_NUM, PWM_WRAP, PWM_TARGET_FREQ, ESC_MIN_US, ESC_MAX_US, THROTTLE_MIN, THROTTLE_MAX);
    if (!status) {
        free(gpio);
        free(oneshot125_params.motors_pwm);
        return 1;
    }

    res = xTaskCreate(OneShotTask, "ONESHOT", ONESHOT_TASK_STACK_WORDS, &oneshot125_params, ONESHOT_TASK_PRIORITY, &xControlTaskHandle);
    configASSERT(res == pdPASS && xControlTaskHandle != NULL);

    // - MicroROS task
    res = xTaskCreate(MicroRosTask, "MICROROS", MICROROS_TASK_STACK_WORDS / sizeof(StackType_t), NULL, MICROROS_TASK_PRIORITY, NULL);
    configASSERT(res == pdPASS);

    // ISR
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_global_callback);

    /* Start the FreeRTOS scheduler. This does not return. */
    vTaskStartScheduler();
}
