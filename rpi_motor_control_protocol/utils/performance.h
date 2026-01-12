#include <unistd.h>

#include <rcutils/time.h>


/**
 * @brief Check if timer deadline is missed
 *
 * @param timer RCL timer object
 * @return bool Result
 */
static bool timer_deadline_missed(rcl_timer_t* timer) {
    int64_t elapsed_ns = 0;
    if (rcl_timer_get_time_since_last_call(timer, &elapsed_ns) != RCL_RET_OK) {
        // On error, treat as “missed” so you get alerted
        return true;
    }
    return (uint64_t)elapsed_ns > (uint64_t)timer_period_ns;
}


/**
 * @brief Compute the drift (actual−ideal) of a periodic timer in ms.
 *
 * @param timer     Your rcl_timer_t handle
 * @param ideal_ns  The requested period [ns]
 * @return float    Drift in milliseconds (positive = late, negative = early), or NAN on error
 */
static float timer_drift_ms(rcl_timer_t* timer, int64_t ideal_ns) {
    int64_t elapsed_ns = 0;
    if (rcl_timer_get_time_since_last_call(timer, &elapsed_ns) != RCL_RET_OK) {
        return NAN;
    }
    // drift = actual − ideal, convert to ms
    return (elapsed_ns - ideal_ns) * 1e-6f;
}


/**
 * @brief Compute the jitter (absolute drift) of a periodic timer in ms.
 *
 * @param timer     Your rcl_timer_t handle
 * @return float    Jitter in milliseconds, or NAN on error
 */
static float timer_jitter_ms(rcl_timer_t* timer) {
    float drift = timer_drift_ms(timer, timer_period_ns);
    if (isnan(drift)) {
        return NAN;
    }
    return fabsf(drift);
}
