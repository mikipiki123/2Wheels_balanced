import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# --- Helper to Auto-Detect Pico on macOS/Linux/Windows ---
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

# --- Data Buffers ---
timestamps = []
angles = []
angular_velocities = []
voltages = []
start_time = time.time()

# --- Control State Variables ---
is_paused = False
view_last_10s = False

# --- Setup Plot Window (Now with 3 subplots) ---
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))
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
    # Always read serial data in the background
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if ',' in line:
                parts = line.split(',')
                # Expecting 3 values: angle, angular_velocity, voltage
                if len(parts) == 3:
                    angle = float(parts[0])
                    angular_vel = float(parts[1])
                    voltage = float(parts[2])
                    
                    timestamps.append(time.time() - start_time)
                    angles.append(angle)
                    angular_velocities.append(angular_vel)
                    voltages.append(voltage)
        except (ValueError, UnicodeDecodeError):
            pass

    # If paused, do not redraw the screen
    if is_paused:
        return

    # Clear axes for fresh rendering
    ax1.clear()
    ax2.clear()
    ax3.clear()

    # 1. Draw Angle Plot
    ax1.plot(timestamps, angles, color='blue', linewidth=2, label='Theta (rad)')
    ax1.set_ylabel('Angle\n(rad)', fontweight='bold')
    ax1.set_title('Real-Time Controller Performance', fontweight='bold')
    ax1.axhline(0, color='black', linestyle='--', linewidth=1)
    ax1.grid(True, linestyle=':', alpha=0.7)

    # 2. Draw Angular Velocity Plot
    ax2.plot(timestamps, angular_velocities, color='green', linewidth=2, label='Theta Dot (rad/s)')
    ax2.set_ylabel('Ang. Vel.\n(rad/s)', fontweight='bold')
    ax2.axhline(0, color='black', linestyle='--', linewidth=1)
    ax2.grid(True, linestyle=':', alpha=0.7)

    # 3. Draw Voltage Plot
    ax3.plot(timestamps, voltages, color='red', linewidth=2, label='Control Input (u)')
    ax3.set_xlabel('Time (s)', fontweight='bold')
    ax3.set_ylabel('Control Input\n(rad/s)', fontweight='bold')
    # ax3.set_ylim(-5.5, 5.5) # Locked to max supply range
    ax3.axhline(0, color='black', linestyle='--', linewidth=1)
    ax3.grid(True, linestyle=':', alpha=0.7)

    # Apply 10-Second Sliding Window Mode if active
    if view_last_10s and timestamps:
        latest_time = timestamps[-1]
        x_min = max(0, latest_time - 10.0)
        x_max = max(10.0, latest_time)
        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)
        ax3.set_xlim(x_min, x_max)

# Start animation loop (20ms interval = 50 FPS)
ani = animation.FuncAnimation(fig, update_plot, interval=20, cache_frame_data=False)

plt.tight_layout()
plt.show()

ser.close()
print("Serial port closed.")