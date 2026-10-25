#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;

    float d_filter_alpha;

    float integral;
    float prev_error;
    float prev_derivative;
    float output;
    uint8_t initialized;
} PIDController;

void pid_init(PIDController *pid, float Kp, float Ki, float Kd,
              float output_min, float output_max);
void pid_set_derivative_filter(PIDController *pid, float alpha);
float pid_compute(PIDController *pid, float setpoint, float measured, float dt);
void pid_reset(PIDController *pid);
void pid_set_gains(PIDController *pid, float Kp, float Ki, float Kd);

#endif
