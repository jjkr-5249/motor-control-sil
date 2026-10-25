#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "motor_model.h"
#include "pid_controller.h"
#include "hal.h"

#define SIM_MAX_STEPS       100000
#define SIM_DT_US           100
#define SIM_CONTROL_PERIOD  1000

typedef struct {
    float time;
    float setpoint;
    float speed_rpm;
    float error;
    float voltage;
    float current;
    float torque;
    float load_torque;
} LogEntry;

typedef struct {
    MotorModel    motor;
    PIDController pid;
    HAL           hal;

    float sim_duration;
    float dt;
    float setpoint_rpm;
    float load_torque;

    float step_time;
    float step_value_rpm;

    float load_step_time;
    float load_step_value;

    LogEntry *log;
    int log_count;
    int log_capacity;
} Simulator;

void sim_init(Simulator *sim);
void sim_set_step_input(Simulator *sim, float time_s, float rpm);
void sim_set_load_disturbance(Simulator *sim, float time_s, float torque);
void sim_set_pid_gains(Simulator *sim, float Kp, float Ki, float Kd);
void sim_set_duration(Simulator *sim, float duration_s);
int sim_run(Simulator *sim);
int sim_export_csv(const Simulator *sim, const char *filename);
void sim_print_summary(const Simulator *sim);
void sim_cleanup(Simulator *sim);

#endif
