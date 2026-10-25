#include "motor_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void motor_init(MotorModel *motor)
{
    motor->R     = 1.0f;
    motor->Ke    = 0.01f;
    motor->Kt    = 0.01f;
    motor->J     = 0.001f;
    motor->B     = 0.001f;
    motor->V_max = 12.0f;

    motor->omega   = 0.0f;
    motor->theta   = 0.0f;
    motor->current = 0.0f;
    motor->torque  = 0.0f;
}

void motor_init_params(MotorModel *motor, float R, float Ke, float Kt,
                       float J, float B, float V_max)
{
    motor->R     = R;
    motor->Ke    = Ke;
    motor->Kt    = Kt;
    motor->J     = J;
    motor->B     = B;
    motor->V_max = V_max;

    motor->omega   = 0.0f;
    motor->theta   = 0.0f;
    motor->current = 0.0f;
    motor->torque  = 0.0f;
}

void motor_update(MotorModel *motor, float voltage, float T_load, float dt)
{
    if (voltage > motor->V_max)
        voltage = motor->V_max;
    if (voltage < -motor->V_max)
        voltage = -motor->V_max;

    motor->current = (voltage - motor->Ke * motor->omega) / motor->R;
    motor->torque = motor->Kt * motor->current;

    float alpha = (motor->torque - motor->B * motor->omega - T_load) / motor->J;

    motor->omega += alpha * dt;
    motor->theta += motor->omega * dt;

    if (motor->omega < 0.0f)
        motor->omega = 0.0f;
}

void motor_reset(MotorModel *motor)
{
    motor->omega   = 0.0f;
    motor->theta   = 0.0f;
    motor->current = 0.0f;
    motor->torque  = 0.0f;
}

float motor_get_rpm(const MotorModel *motor)
{
    return motor->omega * 60.0f / (2.0f * M_PI);
}
