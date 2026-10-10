import socket
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
from datetime import datetime
from pathlib import Path

# --- UDP Configuration ---
UDP_PORT = 5001

# Binary struct format matching C++ TelemetryPacket:
# 6 x float (4 bytes each) = 24 bytes total
# '<ffffff' = Little-endian, 6 floats (x, x_dot, theta, theta_dot, x_int, w_motor)
PACKET_FORMAT = "<ffffff"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 24 bytes

# --- Initialize Non-Blocking UDP Socket ---
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)  # Non-blocking so Matplotlib GUI won't freeze
    print(f"[UDP] Listening for telemetry broadcasts on port {UDP_PORT}...")
except Exception as e:
    print(f"Error opening UDP socket: {e}")
    exit()

# --- Initialize CSV Logging ---
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "signal_records"

if not DATA_DIR.exists():
    DATA_DIR.mkdir(parents=True, exist_ok=True)

csv_filename = f"{DATA_DIR}/telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)

# Write header row
csv_writer.writerow([
    "Timestamp_s", 
    "Position_m", 
    "Velocity_ms", 
    "Angle_rad", 
    "Angular_Velocity_rads", 
    "Integral_Position_ms", 
    "Control_Input_rads"
])
csv_file.flush()
print(f"Logging live telemetry to '{csv_filename}'...")

# --- Data Buffers ---
timestamps = []
linear_positions = []
linear_velocities = []
angles = []
angular_velocities = []
integral_positions = []
control_inputs = []
start_time = time.time()

# --- Control State Variables ---
is_paused = False
view_last_10s = False

# --- Setup Plot Window (4 Subplots) ---
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 11))
fig.canvas.manager.set_window_title('Telemetry: [Space]=Pause | [1]=Last 10s | [2]=Full History')

def on_key_press(event):
    global is_paused, view_last_10s
    if event.key == ' ':
        is_paused = not is_paused
        status = "PAUSED" if is_paused else "RUNNING"
        print(f"[Plot State]: {status}")
    elif event.key == '1':
        view_last_10s = True
        print("[View Mode]: Showing Last 10 Seconds Window")
    elif event.key == '2':
        view_last_10s = False
        print("[View Mode]: Showing Full Recorded History")

fig.canvas.mpl_connect('key_press_event', on_key_press)

def update_plot(frame):
    # Drain all queued UDP packets in non-blocking mode
    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except (BlockingIOError, socket.error):
            break  # Socket buffer empty, continue to plot rendering

        if len(data) == PACKET_SIZE:
            x, x_dot, theta, theta_dot, int_x, u = struct.unpack(PACKET_FORMAT, data)
            t_now = time.time() - start_time

            timestamps.append(t_now)
            linear_positions.append(x)
            linear_velocities.append(x_dot)
            angles.append(theta)
            angular_velocities.append(theta_dot)
            integral_positions.append(int_x)
            control_inputs.append(u)

            # Log to CSV immediately
            csv_writer.writerow([t_now, x, x_dot, theta, theta_dot, int_x, u])
            csv_file.flush()

    if is_paused or not timestamps:
        return

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    # 1. Linear Position and Velocity
    ax1.plot(timestamps, linear_positions, color='blue', linewidth=2, label='Position (m)')
    ax1.plot(timestamps, linear_velocities, color='cyan', linewidth=2, label='Velocity (m/s)')
    ax1.set_ylabel('Linear State\n(m, m/s)', fontweight='bold')
    ax1.set_title('Real-Time 5D LQI State Controller Performance (UDP Broadcast)', fontweight='bold')
    ax1.axhline(0, color='black', linestyle='--', linewidth=1)
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.legend(loc='upper right')

    # 2. Angle and Angular Velocity
    ax2.plot(timestamps, angles, color='green', linewidth=2, label='Angle (rad)')
    ax2.plot(timestamps, angular_velocities, color='lime', linewidth=2, label='Ang. Vel (rad/s)')
    ax2.set_ylabel('Angular State\n(rad, rad/s)', fontweight='bold')
    ax2.axhline(0, color='black', linestyle='--', linewidth=1)
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend(loc='upper right')

    # 3. Integral Position State
    ax3.plot(timestamps, integral_positions, color='purple', linewidth=2, label='Integral Position (m·s)')
    ax3.set_ylabel('Integral State\n(m·s)', fontweight='bold')
    ax3.axhline(0, color='black', linestyle='--', linewidth=1)
    ax3.grid(True, linestyle=':', alpha=0.7)
    ax3.legend(loc='upper right')

    # 4. Control Input
    ax4.plot(timestamps, control_inputs, color='red', linewidth=2, label='Command Velocity (rad/s)')
    ax4.set_xlabel('Time (s)', fontweight='bold')
    ax4.set_ylabel('Control Input\n(rad/s)', fontweight='bold')
    ax4.axhline(0, color='black', linestyle='--', linewidth=1)
    ax4.grid(True, linestyle=':', alpha=0.7)
    ax4.legend(loc='upper right')

    # Apply 10-Second Sliding Window
    if view_last_10s and timestamps:
        latest_time = timestamps[-1]
        x_min = max(0, latest_time - 10.0)
        x_max = max(10.0, latest_time)
        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)
        ax3.set_xlim(x_min, x_max)
        ax4.set_xlim(x_min, x_max)

ani = animation.FuncAnimation(fig, update_plot, interval=20, cache_frame_data=False)

plt.tight_layout()
plt.show()

# Clean shutdown
sock.close()
csv_file.close()
print(f"Session closed. Saved file: {csv_filename}")