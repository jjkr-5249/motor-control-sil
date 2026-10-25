#!/usr/bin/env python3
import sys
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_csv(filename):
    data = {
        'time': [], 'setpoint': [], 'speed_rpm': [], 'error': [],
        'voltage': [], 'current': [], 'torque': [], 'load_torque': []
    }
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in data:
                data[key].append(float(row[key]))
    return data


def compute_metrics(data):
    target = max(data['setpoint'])
    if target == 0:
        return {}

    n = len(data['speed_rpm'])
    start = int(n * 0.9)
    ss_speed = sum(data['speed_rpm'][start:]) / (n - start)
    ss_error = abs(target - ss_speed) / target * 100

    peak = max(data['speed_rpm'])
    overshoot = max(0, (peak - target) / target * 100)

    rise_start = rise_end = None
    for i, s in enumerate(data['speed_rpm']):
        if rise_start is None and s >= 0.1 * target:
            rise_start = data['time'][i]
        if rise_end is None and s >= 0.9 * target:
            rise_end = data['time'][i]
            break

    rise_time = (rise_end - rise_start) if (rise_start and rise_end) else None

    settle_time = None
    band = 0.02 * target
    for i in range(n - 1, -1, -1):
        if abs(data['speed_rpm'][i] - ss_speed) > band:
            if i + 1 < n:
                settle_time = data['time'][i + 1]
            break

    return {
        'target': target, 'ss_speed': ss_speed, 'ss_error': ss_error,
        'overshoot': overshoot, 'rise_time': rise_time,
        'settle_time': settle_time, 'peak': peak
    }


def plot(data, output_prefix='sim_result'):
    metrics = compute_metrics(data)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('DC Motor PID Controller - SIL Simulation', fontsize=14,
                 fontweight='bold')

    ax1 = axes[0]
    ax1.plot(data['time'], data['speed_rpm'], 'b-', linewidth=1.5,
             label='Actual Speed')
    ax1.plot(data['time'], data['setpoint'], 'r--', linewidth=1.2,
             label='Setpoint')
    ax1.set_ylabel('Speed (RPM)')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Step Response')

    if metrics:
        text = (f"Overshoot: {metrics['overshoot']:.1f}%\n"
                f"Rise Time: {metrics['rise_time']*1000:.1f} ms\n"
                f"Settling: {(metrics['settle_time'] - 0.1)*1000:.0f} ms\n"
                f"SS Error: {metrics['ss_error']:.2f}%")
        ax1.text(0.98, 0.55, text, transform=ax1.transAxes, fontsize=9,
                 verticalalignment='center', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    ax2 = axes[1]
    ax2.plot(data['time'], data['voltage'], 'g-', linewidth=1.2)
    ax2.set_ylabel('Voltage (V)')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Control Effort (PWM Voltage)')

    ax3 = axes[2]
    ax3.plot(data['time'], data['current'], 'm-', linewidth=1.2,
             label='Current (A)')
    ax3.plot(data['time'], data['load_torque'], 'k--', linewidth=1.2,
             label='Load Torque (Nm)')
    ax3.set_ylabel('Current (A) / Torque (Nm)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Motor Current & Load')

    plt.tight_layout()

    png_file = f'{output_prefix}.png'
    plt.savefig(png_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to {png_file}")
    plt.close()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_response.py <csv_file> [output_prefix]")
        sys.exit(1)

    csv_file = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'sim_result'

    data = load_csv(csv_file)
    plot(data, prefix)
