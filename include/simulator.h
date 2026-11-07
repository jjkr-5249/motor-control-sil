#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "motor_model.h"
#include "pid_controller.h"
#include "hal.h"

/**
 * Software-in-the-Loop Simulator
 *
 * Connects the PID controller (firmware) to the motor model (plant)
 * through the HAL layer, and logs all signals to CSV for analysis.
 */

/* Simulation parameters */
#define SIM_MAX_STEPS       100000
#define SIM_DT_US           100       /* Plant simulation step: 100us */
#define SIM_CONTROL_PERIOD  1000      /* Control loop period: 1ms (1kHz) */

/* Data logging */
typedef struct {
    float time;
    float setpoint;
    float speed_rpm;
    float error;
    float voltage;
    float current;
    float torque;
    float load_torque;
    int   pid_state;      /* PIDState enum value */
} LogEntry;

typedef struct {
    MotorModel    motor;
    PIDController pid;
    HAL           hal;

    /* Simulation config */
    float sim_duration;       /* Total simulation time (s) */
    float dt;                 /* Plant time step (s) */
    float setpoint_rpm;       /* Target speed (RPM) */
    float load_torque;        /* External load (Nm) */

    /* Step input config */
    float step_time;          /* Time to apply step input (s) */
    float step_value_rpm;     /* Step target RPM */

    /* Load disturbance config */
    float load_step_time;     /* Time to apply load disturbance (s) */
    float load_step_value;    /* Load torque value (Nm) */

    /* Data log */
    LogEntry *log;
    int log_count;
    int log_capacity;
} Simulator;

/**
 * Initialize simulator with default parameters
 */
void sim_init(Simulator *sim);

/**
 * Configure speed step input
 * @param time_s    Time to apply step (s)
 * @param rpm       Target speed (RPM)
 */
void sim_set_step_input(Simulator *sim, float time_s, float rpm);

/**
 * Configure load disturbance
 * @param time_s    Time to apply load (s)
 * @param torque    Load torque (Nm)
 */
void sim_set_load_disturbance(Simulator *sim, float time_s, float torque);

/**
 * Configure PID gains
 */
void sim_set_pid_gains(Simulator *sim, float Kp, float Ki, float Kd);

/**
 * Configure simulation duration
 */
void sim_set_duration(Simulator *sim, float duration_s);

/**
 * Run the simulation
 * @return 0 on success, -1 on error
 */
int sim_run(Simulator *sim);

/**
 * Export logged data to CSV file
 * @param sim       Pointer to simulator
 * @param filename  Output CSV file path
 * @return 0 on success, -1 on error
 */
int sim_export_csv(const Simulator *sim, const char *filename);

/**
 * Print simulation summary to stdout
 */
void sim_print_summary(const Simulator *sim);

/**
 * Free allocated resources
 */
void sim_cleanup(Simulator *sim);

#endif /* SIMULATOR_H */