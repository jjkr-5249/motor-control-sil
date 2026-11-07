#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

/**
 * PID Controller - Embedded Style
 *
 * Features:
 *   - State machine (IDLE-> STARTUP-> RUNNING->ERROR)
 *   - Anti-windup (integral clamping)
 *   - Derivative filter (low-pass on D term)
 *   - Output saturation
 *   - Sustained saturation fault detection
 *   - No dynamic memory allocation
 */

/* Controller states */
typedef enum {
    PID_STATE_IDLE,       /* Initialized, output held at zero              */
    PID_STATE_STARTUP,    /* First N cycles: derivative suppressed,        */
                          /*   integral held to prevent startup kick       */
    PID_STATE_RUNNING,    /* Normal PID operation                          */
    PID_STATE_ERROR       /* Fault detected (e.g. sustained saturation)    */
} PIDState;

/* Default startup cycles before transitioning to RUNNING */
#define PID_STARTUP_CYCLES  5

/* Consecutive saturated cycles before entering ERROR state */
#define PID_SATURATION_LIMIT  5000

typedef struct {
    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Limits */
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;

    /* Derivative filter coefficient (0.0 = no filter, 1.0 = max filter) */
    float d_filter_alpha;

    /* Internal state */
    float integral;
    float prev_error;
    float prev_derivative;
    float output;

    /* State machine */
    PIDState state;
    uint16_t startup_counter;     /* Cycles spent in STARTUP           */
    uint16_t saturation_counter;  /* Consecutive cycles at output limit */
} PIDController;

/**
 * Initialize PID controller with gains and limits.
 * Controller starts in IDLE state.
 */
void pid_init(PIDController *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max);

/**
 * Set derivative filter coefficient
 * @param alpha  Filter coefficient (0.0 = no filter, 0.9 = heavy filter)
 */
void pid_set_derivative_filter(PIDController *pid, float alpha);

/**
 * Compute PID output with state machine management.
 * State transitions:
 *   IDLE    -> STARTUP   (on first call with nonzero setpoint)
 *   STARTUP -> RUNNING   (after PID_STARTUP_CYCLES)
 *   RUNNING -> ERROR     (after PID_SATURATION_LIMIT consecutive saturated cycles)
 *   ERROR   -> IDLE      (via pid_reset)
 *
 * @param pid       Pointer to PID controller
 * @param setpoint  Desired value
 * @param measured  Current measured value
 * @param dt        Time step (s)
 * @return          Control output (clamped to [output_min, output_max])
 */
float pid_compute(PIDController *pid, float setpoint, float measured, float dt);

/**
 * Reset PID internal state. Returns controller to IDLE.
 */
void pid_reset(PIDController *pid);

/**
 * Update PID gains at runtime (for tuning)
 */
void pid_set_gains(PIDController *pid, float Kp, float Ki, float Kd);

/**
 * Get current controller state
 */
PIDState pid_get_state(const PIDController *pid);

/**
 * Get state name as string (for logging/debugging)
 */
const char *pid_state_name(PIDState state);

#endif /* PID_CONTROLLER_H */