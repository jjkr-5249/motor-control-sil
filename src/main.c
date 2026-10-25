#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "simulator.h"

static void print_usage(const char *prog)
{
    printf("DC Motor PID Controller - Software-in-the-Loop Simulator\n\n");
    printf("Usage: %s [options]\n\n", prog);
    printf("Options:\n");
    printf("  -kp <value>      Proportional gain (default: 0.5)\n");
    printf("  -ki <value>      Integral gain (default: 2.0)\n");
    printf("  -kd <value>      Derivative gain (default: 0.001)\n");
    printf("  -rpm <value>     Target speed in RPM (default: 500)\n");
    printf("  -t <value>       Simulation duration in seconds (default: 2.0)\n");
    printf("  -load <time> <torque>  Load disturbance at time (Nm)\n");
    printf("  -o <filename>    Output CSV file (default: output.csv)\n");
    printf("  -h               Show this help\n");
    printf("\nExample:\n");
    printf("  %s -kp 0.8 -ki 3.0 -kd 0.002 -rpm 600 -load 1.0 0.005\n", prog);
}

int main(int argc, char *argv[])
{
    Simulator sim;
    sim_init(&sim);

    const char *output_file = "output.csv";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-kp") == 0 && i + 1 < argc) {
            sim.pid.Kp = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "-ki") == 0 && i + 1 < argc) {
            sim.pid.Ki = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "-kd") == 0 && i + 1 < argc) {
            sim.pid.Kd = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "-rpm") == 0 && i + 1 < argc) {
            sim_set_step_input(&sim, 0.1f, atof(argv[++i]));
        }
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            sim_set_duration(&sim, atof(argv[++i]));
        }
        else if (strcmp(argv[i], "-load") == 0 && i + 2 < argc) {
            float t = atof(argv[++i]);
            float torque = atof(argv[++i]);
            sim_set_load_disturbance(&sim, t, torque);
        }
        else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        }
        else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            sim_cleanup(&sim);
            return 0;
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            sim_cleanup(&sim);
            return 1;
        }
    }

    if (sim_run(&sim) != 0) {
        fprintf(stderr, "Simulation failed!\n");
        sim_cleanup(&sim);
        return 1;
    }

    sim_print_summary(&sim);

    if (sim_export_csv(&sim, output_file) != 0) {
        fprintf(stderr, "Failed to export data!\n");
        sim_cleanup(&sim);
        return 1;
    }

    printf("\nRun 'python3 scripts/plot_response.py %s' to visualize.\n",
           output_file);

    sim_cleanup(&sim);
    return 0;
}
