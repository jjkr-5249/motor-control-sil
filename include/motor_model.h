#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <stdint.h>

typedef struct {
    float R;
    float Ke;
    float Kt;
    float J;
    float B;
    float V_max;

    float omega;
    float theta;
    float current;
    float torque;
} MotorModel;

void motor_init(MotorModel *motor);
void motor_init_params(MotorModel *motor, float R, float Ke, float Kt,
                       float J, float B, float V_max);
void motor_update(MotorModel *motor, float voltage, float T_load, float dt);
void motor_reset(MotorModel *motor);
float motor_get_rpm(const MotorModel *motor);

#endif
