#include "simulator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void sim_init(Simulator *sim)
{
    memset(sim, 0, sizeof(Simulator));

    motor_init(&sim->motor);

    pid_init(&sim->pid, 0.5f, 2.0f, 0.001f, 0.0f, 12.0f);
    pid_set_derivative_filter(&sim->pid, 0.8f);

    hal_init(&sim->hal);
    hal_timer_set_period(&sim->hal, SIM_CONTROL_PERIOD);

    sim->sim_duration  = 2.0f;
    sim->dt            = (float)SIM_DT_US / 1000000.0f;
    sim->setpoint_rpm  = 0.0f;
    sim->load_torque   = 0.0f;

    sim->step_time      = 0.1f;
    sim->step_value_rpm = 500.0f;

    sim->load_step_time  = -1.0f;
    sim->load_step_value = 0.0f;

    sim->log_capacity = (int)(sim->sim_duration / sim->dt) + 100;
    sim->log = (LogEntry *)malloc(sim->log_capacity * sizeof(LogEntry));
    sim->log_count = 0;
}

void sim_set_step_input(Simulator *sim, float time_s, float rpm)
{
    sim->step_time = time_s;
    sim->step_value_rpm = rpm;
}

void sim_set_load_disturbance(Simulator *sim, float time_s, float torque)
{
    sim->load_step_time = time_s;
    sim->load_step_value = torque;
}

void sim_set_pid_gains(Simulator *sim, float Kp, float Ki, float Kd)
{
    pid_set_gains(&sim->pid, Kp, Ki, Kd);
}

void sim_set_duration(Simulator *sim, float duration_s)
{
    sim->sim_duration = duration_s;

    sim->log_capacity = (int)(duration_s / sim->dt) + 100;
    free(sim->log);
    sim->log = (LogEntry *)malloc(sim->log_capacity * sizeof(LogEntry));
    sim->log_count = 0;
}

int sim_run(Simulator *sim)
{
    if (!sim->log) return -1;

    motor_reset(&sim->motor);
    pid_reset(&sim->pid);
    sim->log_count = 0;

    float time = 0.0f;
    float control_voltage = 0.0f;
    float current_setpoint = 0.0f;
    float current_load = 0.0f;

    int total_steps = (int)(sim->sim_duration / sim->dt);
    int log_decimation = 10;

    printf("Running simulation: %.2fs, dt=%.0fus, control@%dHz\n",
           sim->sim_duration, sim->dt * 1e6f,
           (int)(1.0f / (SIM_CONTROL_PERIOD * 1e-6f)));
    printf("PID gains: Kp=%.3f Ki=%.3f Kd=%.4f\n",
           sim->pid.Kp, sim->pid.Ki, sim->pid.Kd);
    printf("Step input: %.0f RPM at t=%.2fs\n",
           sim->step_value_rpm, sim->step_time);

    if (sim->load_step_time >= 0.0f) {
        printf("Load disturbance: %.3f Nm at t=%.2fs\n",
               sim->load_step_value, sim->load_step_time);
    }

    for (int step = 0; step < total_steps; step++) {
        time = step * sim->dt;

        if (time >= sim->step_time) {
            current_setpoint = sim->step_value_rpm;
        }

        if (sim->load_step_time >= 0.0f && time >= sim->load_step_time) {
            current_load = sim->load_step_value;
        }

        hal_timer_tick(&sim->hal, (uint32_t)(sim->dt * 1e6f));

        if (hal_timer_check(&sim->hal)) {
            float measured_rpm = motor_get_rpm(&sim->motor);

            float control_dt = (float)SIM_CONTROL_PERIOD / 1000000.0f;
            control_voltage = pid_compute(&sim->pid, current_setpoint,
                                          measured_rpm, control_dt);

            uint8_t duty = (uint8_t)((control_voltage / sim->motor.V_max)
                           * (HAL_PWM_RESOLUTION - 1));
            hal_pwm_write(&sim->hal, 0, duty);
        }

        float applied_voltage = hal_pwm_get_voltage(&sim->hal, 0);
        motor_update(&sim->motor, applied_voltage, current_load, sim->dt);

        if (step % log_decimation == 0 && sim->log_count < sim->log_capacity) {
            LogEntry *entry = &sim->log[sim->log_count++];
            entry->time        = time;
            entry->setpoint    = current_setpoint;
            entry->speed_rpm   = motor_get_rpm(&sim->motor);
            entry->error       = current_setpoint - entry->speed_rpm;
            entry->voltage     = applied_voltage;
            entry->current     = sim->motor.current;
            entry->torque      = sim->motor.torque;
            entry->load_torque = current_load;
        }
    }

    printf("Simulation complete: %d data points logged\n", sim->log_count);
    return 0;
}

int sim_export_csv(const Simulator *sim, const char *filename)
{
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "Error: cannot open %s for writing\n", filename);
        return -1;
    }

    fprintf(fp, "time,setpoint,speed_rpm,error,voltage,current,torque,load_torque\n");

    for (int i = 0; i < sim->log_count; i++) {
        const LogEntry *e = &sim->log[i];
        fprintf(fp, "%.6f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                e->time, e->setpoint, e->speed_rpm, e->error,
                e->voltage, e->current, e->torque, e->load_torque);
    }

    fclose(fp);
    printf("Data exported to %s\n", filename);
    return 0;
}

void sim_print_summary(const Simulator *sim)
{
    if (sim->log_count == 0) {
        printf("No simulation data available.\n");
        return;
    }

    int start_idx = (int)(sim->log_count * 0.9f);
    float ss_speed = 0.0f;
    int count = 0;
    for (int i = start_idx; i < sim->log_count; i++) {
        ss_speed += sim->log[i].speed_rpm;
        count++;
    }
    ss_speed /= count;

    float peak_speed = 0.0f;
    float peak_time = 0.0f;
    for (int i = 0; i < sim->log_count; i++) {
        if (sim->log[i].speed_rpm > peak_speed) {
            peak_speed = sim->log[i].speed_rpm;
            peak_time = sim->log[i].time;
        }
    }

    float target = sim->step_value_rpm;
    float rise_start = -1.0f, rise_end = -1.0f;
    for (int i = 0; i < sim->log_count; i++) {
        if (rise_start < 0.0f && sim->log[i].speed_rpm >= 0.1f * target) {
            rise_start = sim->log[i].time;
        }
        if (rise_end < 0.0f && sim->log[i].speed_rpm >= 0.9f * target) {
            rise_end = sim->log[i].time;
        }
    }

    float settle_band = 0.02f * target;
    float settle_time = sim->log[sim->log_count - 1].time;
    for (int i = sim->log_count - 1; i >= 0; i--) {
        if (fabsf(sim->log[i].speed_rpm - ss_speed) > settle_band) {
            if (i + 1 < sim->log_count)
                settle_time = sim->log[i + 1].time;
            break;
        }
    }

    float overshoot = 0.0f;
    if (target > 0.0f)
        overshoot = ((peak_speed - target) / target) * 100.0f;

    printf("\n===== PERFORMANCE SUMMARY =====\n");
    printf("  Target Speed:    %.1f RPM\n", target);
    printf("  Steady-State:    %.1f RPM\n", ss_speed);
    printf("  SS Error:        %.2f%%\n",
           fabsf(target - ss_speed) / target * 100.0f);
    printf("  Peak Speed:      %.1f RPM at t=%.4fs\n", peak_speed, peak_time);
    printf("  Overshoot:       %.1f%%\n", overshoot > 0 ? overshoot : 0.0f);
    printf("  Rise Time:       %.4fs\n",
           (rise_start >= 0 && rise_end >= 0) ? rise_end - rise_start : -1.0f);
    printf("  Settling Time:   %.4fs (2%% band)\n", settle_time - sim->step_time);
    printf("===============================\n");
}

void sim_cleanup(Simulator *sim)
{
    if (sim->log) {
        free(sim->log);
        sim->log = NULL;
    }
    sim->log_count = 0;
    sim->log_capacity = 0;
}
