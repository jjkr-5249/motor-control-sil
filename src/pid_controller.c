#include "pid_controller.h"

static const char *state_names[] = {
    "IDLE", "STARTUP", "RUNNING", "ERROR"
};

void pid_init(PIDController *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->output_min = output_min;
    pid->output_max = output_max;

    /* Default anti-windup limits match output limits */
    pid->integral_min = output_min;
    pid->integral_max = output_max;

    /* Default: no derivative filter */
    pid->d_filter_alpha = 0.0f;

    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output          = 0.0f;

    /* State machine */
    pid->state              = PID_STATE_IDLE;
    pid->startup_counter    = 0;
    pid->saturation_counter = 0;
}

void pid_set_derivative_filter(PIDController *pid, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;
    pid->d_filter_alpha = alpha;
}

float pid_compute(PIDController *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f) {
        return pid->output;
    }

    /* ---- State: IDLE ---- */
    if (pid->state == PID_STATE_IDLE) {
        pid->output = 0.0f;

        /* Transition to STARTUP when a nonzero setpoint is commanded */
        if (setpoint != 0.0f) {
            pid->state = PID_STATE_STARTUP;
            pid->startup_counter = 0;
        } else {
            return pid->output;
        }
    }

    /* ---- State: ERROR ---- */
    if (pid->state == PID_STATE_ERROR) {
        /* Hold output at zero; only pid_reset() exits this state */
        pid->output = 0.0f;
        return pid->output;
    }

    float error = setpoint - measured;

    /* --- Proportional term --- */
    float P = pid->Kp * error;

    /* --- Integral term with anti-windup --- */
    float I = 0.0f;

    if (pid->state == PID_STATE_STARTUP) {
        /* During startup: hold integral at zero to prevent kick */
        pid->integral = 0.0f;
    } else {
        pid->integral += error * dt;

        /* Clamp integral to prevent windup */
        if (pid->integral > pid->integral_max)
            pid->integral = pid->integral_max;
        if (pid->integral < pid->integral_min)
            pid->integral = pid->integral_min;
    }
    I = pid->Ki * pid->integral;

    /* --- Derivative term with optional low-pass filter --- */
    float raw_derivative;

    if (pid->state == PID_STATE_STARTUP) {
        /* During startup: suppress derivative to avoid kick */
        raw_derivative = 0.0f;
    } else {
        raw_derivative = (error - pid->prev_error) / dt;
    }

    /* Apply low-pass filter to derivative */
    float filtered_derivative = pid->d_filter_alpha * pid->prev_derivative
                              + (1.0f - pid->d_filter_alpha) * raw_derivative;
    pid->prev_derivative = filtered_derivative;

    float D = pid->Kd * filtered_derivative;

    /* --- Compute output --- */
    pid->output = P + I + D;

    /* --- Clamp output and track saturation --- */
    uint8_t saturated = 0;

    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
        saturated = 1;
    }
    if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
        saturated = 1;
    }

    /* --- Saturation fault detection --- */
    if (saturated) {
        pid->saturation_counter++;
    } else {
        pid->saturation_counter = 0;
    }

    if (pid->state == PID_STATE_RUNNING &&
        pid->saturation_counter >= PID_SATURATION_LIMIT) {
        pid->state = PID_STATE_ERROR;
        pid->output = 0.0f;
        return pid->output;
    }

    /* --- State transitions --- */
    if (pid->state == PID_STATE_STARTUP) {
        pid->startup_counter++;
        if (pid->startup_counter >= PID_STARTUP_CYCLES) {
            pid->state = PID_STATE_RUNNING;
        }
    }

    pid->prev_error = error;

    return pid->output;
}

void pid_reset(PIDController *pid)
{
    pid->integral           = 0.0f;
    pid->prev_error         = 0.0f;
    pid->prev_derivative    = 0.0f;
    pid->output             = 0.0f;
    pid->state              = PID_STATE_IDLE;
    pid->startup_counter    = 0;
    pid->saturation_counter = 0;
}

void pid_set_gains(PIDController *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

PIDState pid_get_state(const PIDController *pid)
{
    return pid->state;
}

const char *pid_state_name(PIDState state)
{
    if (state >= PID_STATE_IDLE && state <= PID_STATE_ERROR) {
        return state_names[state];
    }
    return "UNKNOWN";
}