import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
from datetime import datetime

# --- Helper to Auto-Detect Pico ---
def find_pico_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device or "Pico" in port.description or "ttyACM" in port.device:
            print(f"Found Pico on port: {port.device}")
            return port.device
    return None

# --- Initialize Serial ---
SERIAL_PORT = find_pico_port()
BAUD_RATE = 115200

if not SERIAL_PORT:
    print("Error: Raspberry Pi Pico not found! Check your USB connection.")
    exit()

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Successfully connected to {SERIAL_PORT}")
    time.sleep(1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# --- Initialize CSV Logging ---
csv_filename = f"telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
# Write header row
csv_writer.writerow(["Timestamp_s", "Position_m", "Velocity_ms", "Angle_rad", "Angular_Velocity_rads", "Integral_Position_ms", "Control_Input_rads"])
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
is_connected = True

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
    global is_connected
    if not is_connected:
        return

    # Read serial data in the background
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if ',' in line:
                parts = line.split(',')
                t_now = time.time() - start_time
                
                # Case 1: Firmware sends 6 values (x, x_dot, theta, theta_dot, integral_x, u)
                if len(parts) == 6:
                    x = float(parts[0])
                    x_dot = float(parts[1])
                    theta = float(parts[2])
                    theta_dot = float(parts[3])
                    int_x = float(parts[4])
                    u = float(parts[5])

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
                    
                # Case 2: Firmware sends 5 values (x, x_dot, theta, theta_dot, u) -> fallback integrate in Python
                elif len(parts) == 5:
                    x = float(parts[0])
                    x_dot = float(parts[1])
                    theta = float(parts[2])
                    theta_dot = float(parts[3])
                    u = float(parts[4])

                    timestamps.append(t_now)
                    linear_positions.append(x)
                    linear_velocities.append(x_dot)
                    angles.append(theta)
                    angular_velocities.append(theta_dot)
                    control_inputs.append(u)
                    
                    dt = (timestamps[-1] - timestamps[-2]) if len(timestamps) > 1 else 0.005
                    prev_int = integral_positions[-1] if integral_positions else 0.0
                    int_x = prev_int + x * dt
                    integral_positions.append(int_x)

                    # Log to CSV immediately
                    csv_writer.writerow([t_now, x, x_dot, theta, theta_dot, int_x, u])
                    csv_file.flush()

    except (serial.SerialException, OSError) as e:
        print(f"\n[Disconnected]: Robot cable pulled or connection lost ({e}).")
        print(f"All data up to disconnection saved to '{csv_filename}'.")
        is_connected = False
        return
    except (ValueError, UnicodeDecodeError):
        pass

    if is_paused:
        return

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    # 1. Linear Position and Velocity
    ax1.plot(timestamps, linear_positions, color='blue', linewidth=2, label='Position (m)')
    ax1.plot(timestamps, linear_velocities, color='cyan', linewidth=2, label='Velocity (m/s)')
    ax1.set_ylabel('Linear State\n(m, m/s)', fontweight='bold')
    ax1.set_title('Real-Time 5D LQI State Controller Performance', fontweight='bold')
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
if ser.is_open:
    ser.close()
csv_file.close()
print(f"Session closed. Saved file: {csv_filename}")