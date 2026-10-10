# import numpy as np
# import matplotlib.pyplot as plt
# import control

# # =========================================================================
# # 1. Physical System Parameters & Sampling Configuration
# # =========================================================================
# Ts = 0.005  # Sampling Period: 5 ms (200 Hz FreeRTOS Control Loop)

# g_mm2_to_kg_m2 = 1e-9  # 1 g*mm^2 = 10^-9 kg*m^2

# mtot = 0.92 + 0.25                      # Total mass (kg)
# mp = 0.4 + 0.25                             # Pendulum mass (kg)
# lp = 120.17 * 0.001                   # Distance from pivot to center of mass (m)
# l_tot = (mp / mtot) * lp             # Pivot to center mass (m)

# mbat = 0.25                          # Battery mass (kg)
# lbat = 0.18                          # Battery distance from pivot to center of mass (m)
# Ibat = (1.0 / 12.0) * mbat * lbat**2 # Battery inertia (kg*m^2)

# I_total = 0.00470 + (Ibat + mbat * lbat**2) # Total inertia around axle
# g = 9.81                            # Gravity (m/s^2)
# R_wheel = 0.065 / 2                 # Wheel radius (m)


# # =========================================================================
# # 2. Reduced 2D Discrete Controller Design (Tilt Angle Only)
# # =========================================================================
# def compute_2d_discrete_controller():
#     print("=== 2D DISCRETE CONTROLLER DESIGN (Ts = 5 ms) ===")

#     A_2d = np.array([
#         [0.0, 1.0],
#         [(mtot * g * l_tot) / I_total, 0.0]
#     ])

#     B_2d = np.array([
#         [0.0],
#         [-(mtot * l_tot * R_wheel) / I_total]
#     ])

#     # Convert continuous system to discrete state-space (ZOH)
#     sys_c_2d = control.ss(A_2d, B_2d, np.eye(2), np.zeros((2, 1)))
#     sys_d_2d = control.c2d(sys_c_2d, Ts, method='zoh')
#     Ad_2d, Bd_2d = sys_d_2d.A, sys_d_2d.B

#     # DLQR Gain Computation
#     Q_2d = np.diag([5.0, 0.0])
#     R_2d = np.array([[1.0]])

#     K_d_2d, _, _ = control.dlqr(Ad_2d, Bd_2d, Q_2d, R_2d)
#     K_flat = K_d_2d.flatten()

#     cl_poles_2d = np.linalg.eigvals(Ad_2d - Bd_2d @ K_d_2d)
#     print(f"Discrete Closed-Loop Poles:\n{cl_poles_2d}")
#     print(f"double K_2D[2] = {{ {K_flat[0]:.4f}, {K_flat[1]:.4f} }}; // Controller gains\n")


# # =========================================================================
# # 3. Parametric 5D Discrete LQI Controller Design
# # =========================================================================
# def compute_5d_discrete_controller():
#     print("=== 5D DISCRETE LQI CONTROLLER DESIGN (Ts = 5 ms) ===")

#     A_theta = (mtot * g * l_tot) / I_total
#     B_theta = -(mtot * l_tot * R_wheel) / I_total

#     # 1. Base 4D Continuous State-Space Matrices: [x, x_dot, theta, theta_dot]
#     A_4d = np.array([
#         [0.0, 1.0, 0.0, 0.0],
#         [0.0, 0.0, 0.0, 0.0],
#         [0.0, 0.0, 0.0, 1.0],
#         [0.0, 0.0, A_theta, 0.0]
#     ])

#     B_4d = np.array([
#         [0.0],
#         [R_wheel],
#         [0.0],
#         [B_theta]
#     ])

#     # 2. Discretize 4D base plant using Zero-Order Hold (ZOH)
#     sys_c_4d = control.ss(A_4d, B_4d, np.eye(4), np.zeros((4, 1)))
#     sys_d_4d = control.c2d(sys_c_4d, Ts, method='zoh')
#     Ad_4d, Bd_4d = sys_d_4d.A, sys_d_4d.B

#     # 3. Augment with Discrete Forward Euler Integrator: x_int[k+1] = x_int[k] + Ts * x[k]
#     # State Vector: [x, x_dot, theta, theta_dot, integral_x]
#     Ad_5d = np.block([
#         [Ad_4d, np.zeros((4, 1))],
#         [np.array([[Ts, 0.0, 0.0, 0.0]]), np.array([[1.0]])]
#     ])

#     Bd_5d = np.block([
#         [Bd_4d],
#         [np.zeros((1, 1))]
#     ])

#     # 4. Compute Discrete LQR (DLQR) Gains
#     Q_5d = np.diag([
#     1000000.0,   # x
#     10000.0,  # x_dot
#     0.01,    # theta (lowered to soft-cap K2)
#     0.001,   # theta_dot (near-zero to prevent gyro noise chattering)
#     200.0    # integral_x
#     ])
#     R_5d = np.array([[80.0]])

#     K_d, _, _ = control.dlqr(Ad_5d, Bd_5d, Q_5d, R_5d)
#     K_flat = K_d.flatten()

#     Ad_cl = Ad_5d - Bd_5d @ K_d
#     cl_poles = np.linalg.eigvals(Ad_cl)

#     print(f"Discrete Closed Loop Poles:\n{cl_poles}\n")
#     print("// Copy-paste this array directly into C++ firmware (Core 1):")
#     print(
#         f"const double K[5] = {{ {K_flat[0]:.4f}f, {K_flat[1]:.4f}f, "
#         f"{K_flat[2]:.4f}f, {K_flat[3]:.4f}f, {K_flat[4]:.4f}f }}; // Controller gains\n"
#     )

#     # 5. Step-by-Step Discrete Simulation
#     t_sim = 15.0  # seconds
#     N = int(t_sim / Ts)
#     t_vec = np.linspace(0, t_sim, N)

#     x_states = np.zeros((N, 5))
#     u_accel = np.zeros(N)

#     # Initial condition: 0.1 m position offset
#     x_states[0] = np.array([0.2, 0.0, 0.0, 0.0, 0.0])

#     for k in range(N - 1):
#         # Calculate discrete control input: u[k] = -K_d * x[k]
#         u_accel[k] = -(K_d @ x_states[k])[0]

#         # Update state using discrete state equation: x[k+1] = Ad*x[k] + Bd*u[k]
#         x_states[k + 1] = Ad_cl @ x_states[k]

#     u_accel[-1] = -(K_d @ x_states[-1])[0]

#     # Extract states for plotting
#     x_pos = x_states[:, 0]
#     x_dot = x_states[:, 1]
#     theta = x_states[:, 2]
#     theta_dot = x_states[:, 3]
#     x_integral = x_states[:, 4]
#     w_motor = x_dot / R_wheel

#     # Plotting Discrete Simulation Results
#     fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
#     fig.canvas.manager.set_window_title("Discrete LQI Simulation Results (200 Hz)")

#     # Plot 1: System States
#     axs[0].plot(t_vec, x_pos, 'b', label='x (m)', linewidth=1.5)
#     axs[0].plot(t_vec, x_dot, 'r', label=r'$\dot{x}$ (m/s)', linewidth=1.5)
#     axs[0].plot(t_vec, theta, 'g', label=r'$\theta$ (rad)', linewidth=1.5)
#     axs[0].plot(t_vec, theta_dot, 'y', label=r'$\dot{\theta}$ (rad/s)', linewidth=1.5)
#     axs[0].plot(t_vec, x_integral, 'k', label='integral(x)', linewidth=1.5)
#     axs[0].set_ylabel('States')
#     axs[0].legend(loc='upper right')
#     axs[0].grid(True)
#     axs[0].set_title(f'Discrete System Response (Ts = {Ts*1000:.1f} ms)')

#     # Plot 2: Motor Angular Velocity
#     axs[1].plot(t_vec, w_motor, 'm', label=r'Motor Velocity $w[k]$', linewidth=1.5)
#     axs[1].set_ylabel('w (rad/s)')
#     axs[1].legend(loc='upper right')
#     axs[1].grid(True)

#     # Plot 3: Discrete Control Acceleration u[k]
#     axs[2].plot(t_vec, u_accel, 'r', label=r'Control Input $u[k]$', linewidth=1.5)
#     axs[2].set_xlabel('Time (seconds)')
#     axs[2].set_ylabel(r'$\alpha$ (rad/s$^2$)')
#     axs[2].legend(loc='upper right')
#     axs[2].grid(True)


# if __name__ == "__main__":
#     compute_5d_discrete_controller()
#     plt.tight_layout()
#     plt.show()


import numpy as np
import matplotlib.pyplot as plt
import control

# =========================================================================
# 1. Physical System Parameters & Sampling Configuration
# =========================================================================
Ts = 0.005  # Sampling Period: 5 ms (200 Hz FreeRTOS Control Loop)

g_mm2_to_kg_m2 = 1e-9  # 1 g*mm^2 = 10^-9 kg*m^2

mtot = 0.92 + 0.25                      # Total mass (kg)
mp = 0.4 + 0.25                         # Pendulum mass (kg)
lp = 120.17 * 0.001                     # Distance from pivot to center of mass (m)
l_tot = (mp / mtot) * lp                # Pivot to center mass (m)

mbat = 0.25                          # Battery mass (kg)
lbat = 0.18                          # Battery distance from pivot to center of mass (m)
Ibat = (1.0 / 12.0) * mbat * lbat**2 # Battery inertia (kg*m^2)

I_total = 0.00470 + (Ibat + mbat * lbat**2) # Total inertia around axle
g = 9.81                            # Gravity (m/s^2)
R_wheel = 0.065 / 2                 # Wheel radius (m)


# =========================================================================
# 2. Reduced 2D Discrete Controller Design (Tilt Angle Only)
# =========================================================================
def compute_2d_discrete_controller():
    print("=== 2D DISCRETE CONTROLLER DESIGN (Ts = 5 ms) ===")

    A_2d = np.array([
        [0.0, 1.0],
        [(mtot * g * l_tot) / I_total, 0.0]
    ])

    B_2d = np.array([
        [0.0],
        [-(mtot * l_tot * R_wheel) / I_total]
    ])

    # Convert continuous system to discrete state-space (ZOH)
    sys_c_2d = control.ss(A_2d, B_2d, np.eye(2), np.zeros((2, 1)))
    sys_d_2d = control.c2d(sys_c_2d, Ts, method='zoh')
    Ad_2d, Bd_2d = sys_d_2d.A, sys_d_2d.B

    # DLQR Gain Computation
    Q_2d = np.diag([5.0, 0.0])
    R_2d = np.array([[1.0]])

    K_d_2d, _, _ = control.dlqr(Ad_2d, Bd_2d, Q_2d, R_2d)
    K_flat = K_d_2d.flatten()

    cl_poles_2d = np.linalg.eigvals(Ad_2d - Bd_2d @ K_d_2d)
    print(f"Discrete Closed-Loop Poles:\n{cl_poles_2d}")
    print(f"double K_2D[2] = {{ {K_flat[0]:.4f}, {K_flat[1]:.4f} }}; // Controller gains\n")


# =========================================================================
# 3. Parametric 5D Discrete LQI Controller Design
# =========================================================================
def compute_5d_discrete_controller():
    print("=== 5D DISCRETE LQI CONTROLLER DESIGN (Ts = 5 ms) ===")

    A_theta = (mtot * g * l_tot) / I_total
    B_theta = -(mtot * l_tot * R_wheel) / I_total

    # 1. Base 4D Continuous State-Space Matrices: [x, x_dot, theta, theta_dot]
    A_4d = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, A_theta, 0.0]
    ])

    B_4d = np.array([
        [0.0],
        [R_wheel],
        [0.0],
        [B_theta]
    ])

    # 2. Discretize 4D base plant using Zero-Order Hold (ZOH)
    sys_c_4d = control.ss(A_4d, B_4d, np.eye(4), np.zeros((4, 1)))
    sys_d_4d = control.c2d(sys_c_4d, Ts, method='zoh')
    Ad_4d, Bd_4d = sys_d_4d.A, sys_d_4d.B

    # 3. Augment with Discrete Forward Euler Integrator: x_int[k+1] = x_int[k] + Ts * x[k]
    # State Vector: [x, x_dot, theta, theta_dot, integral_x]
    Ad_5d = np.block([
        [Ad_4d, np.zeros((4, 1))],
        [np.array([[Ts, 0.0, 0.0, 0.0]]), np.array([[1.0]])]
    ])

    Bd_5d = np.block([
        [Bd_4d],
        [np.zeros((1, 1))]
    ])

    # =========================================================================
    # 4. Compute Controller Gains
    # =========================================================================

    # --- METHOD A: Discrete LQR (DLQR) ---
    # Q_5d = np.diag([
    #     1000000.0,   # x
    #     10000.0,     # x_dot
    #     0.01,        # theta (lowered to soft-cap K2)
    #     0.001,       # theta_dot (near-zero to prevent gyro noise chattering)
    #     200.0        # integral_x
    # ])
    # R_5d = np.array([[80.0]])

    # K_d, _, _ = control.dlqr(Ad_5d, Bd_5d, Q_5d, R_5d)

    # --- METHOD B: Discrete Pole Placement (Uncomment to use instead of DLQR) ---
    # s_pos1 = -1.10    # Position convergence (~3.5 s settling time)
    # s_pos2 = -1.30    # Velocity damping
    # s_tilt_r = -2.80  # Tilt damping real part
    # s_tilt_i = 2.80   # Tilt oscillation frequency
    # s_int = -0.40     # Integral action

    x_ref = 0.30  # Desired position reference (m)

    s_pos1 = -1.10/(x_ref*0.9)    # Position convergence (~3.5 s settling time)
    s_pos2 = -1.30/(x_ref*0.9)    # Velocity damping
    s_tilt_r = -2.80  # Tilt damping real part
    s_tilt_i = 2.80   # Tilt oscillation frequency
    s_int = -0.40     # Integral action
    
    z_pos1 = np.exp(s_pos1 * Ts)
    z_pos2 = np.exp(s_pos2 * Ts)
    z_tilt1 = np.exp((s_tilt_r + 1j * s_tilt_i) * Ts)
    z_tilt2 = np.exp((s_tilt_r - 1j * s_tilt_i) * Ts)
    z_int = np.exp(s_int * Ts)
    
    desired_poles = [z_pos1, z_pos2, z_tilt1, z_tilt2, z_int]
    K_d = control.place(Ad_5d, Bd_5d, desired_poles)

    K_flat = K_d.flatten()


    Ad_cl = Ad_5d - Bd_5d @ K_d
    cl_poles = np.linalg.eigvals(Ad_cl)

    print(f"Discrete Closed Loop Poles:\n{cl_poles}\n")
    print("// Copy-paste this array directly into C++ firmware (Core 1):")
    print(
        f"const double K[5] = {{ {K_flat[0]:.4f}f, {K_flat[1]:.4f}f, "
        f"{K_flat[2]:.4f}f, {K_flat[3]:.4f}f, {K_flat[4]:.4f}f }}; // Controller gains\n"
    )

    # 5. Step-by-Step Discrete Simulation
    t_sim = 25.0  # seconds
    N = int(t_sim / Ts)
    t_vec = np.linspace(0, t_sim, N)

    x_states = np.zeros((N, 5))
    u_accel = np.zeros(N)

    # Initial condition: 0.2 m position offset
    x_states[0] = np.array([x_ref, 0.0, 0.0, 0.0, 0.0])

    for k in range(N - 1):
        # Calculate discrete control input: u[k] = -K_d * x[k]
        u_accel[k] = -(K_d @ x_states[k])[0]

        # Update state using discrete state equation: x[k+1] = Ad*x[k] + Bd*u[k]
        x_states[k + 1] = Ad_cl @ x_states[k]

    u_accel[-1] = -(K_d @ x_states[-1])[0]

    # Extract states for plotting
    x_pos = x_states[:, 0]
    x_dot = x_states[:, 1]
    theta = x_states[:, 2]
    theta_dot = x_states[:, 3]
    x_integral = x_states[:, 4]
    w_motor = x_dot / R_wheel

    # Plotting Discrete Simulation Results
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title("Discrete LQI Simulation Results (200 Hz)")

    # Plot 1: System States
    axs[0].plot(t_vec, x_pos, 'b', label='x (m)', linewidth=1.5)
    axs[0].plot(t_vec, x_dot, 'r', label=r'$\dot{x}$ (m/s)', linewidth=1.5)
    axs[0].plot(t_vec, theta, 'g', label=r'$\theta$ (rad)', linewidth=1.5)
    axs[0].plot(t_vec, theta_dot, 'y', label=r'$\dot{\theta}$ (rad/s)', linewidth=1.5)
    axs[0].plot(t_vec, x_integral, 'k', label='integral(x)', linewidth=1.5)
    axs[0].set_ylabel('States')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)
    axs[0].set_title(f'Discrete System Response (Ts = {Ts*1000:.1f} ms)')

    # Plot 2: Motor Angular Velocity
    axs[1].plot(t_vec, w_motor, 'm', label=r'Motor Velocity $w[k]$', linewidth=1.5)
    axs[1].set_ylabel('w (rad/s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True)

    # Plot 3: Discrete Control Acceleration u[k]
    axs[2].plot(t_vec, u_accel, 'r', label=r'Control Input $u[k]$', linewidth=1.5)
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel(r'$\alpha$ (rad/s$^2$)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)


if __name__ == "__main__":
    compute_5d_discrete_controller()
    plt.tight_layout()
    plt.show()