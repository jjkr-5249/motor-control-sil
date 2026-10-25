#include "pid_controller.h"

void pid_init(PIDController *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->integral_min = output_min;
    pid->integral_max = output_max;

    pid->d_filter_alpha = 0.0f;

    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output         = 0.0f;
    pid->initialized    = 0;
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

    float error = setpoint - measured;

    float P = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;

    float I = pid->Ki * pid->integral;

    float raw_derivative;
    if (!pid->initialized) {
        raw_derivative = 0.0f;
        pid->initialized = 1;
    } else {
        raw_derivative = (error - pid->prev_error) / dt;
    }

    float filtered_derivative = pid->d_filter_alpha * pid->prev_derivative
                              + (1.0f - pid->d_filter_alpha) * raw_derivative;
    pid->prev_derivative = filtered_derivative;

    float D = pid->Kd * filtered_derivative;

    pid->output = P + I + D;
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    pid->prev_error = error;

    return pid->output;
}

void pid_reset(PIDController *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output          = 0.0f;
    pid->initialized     = 0;
}

void pid_set_gains(PIDController *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
