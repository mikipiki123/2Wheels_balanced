import numpy as np
import matplotlib.pyplot as plt
import control

# =========================================================================
# 1. Physical System Parameters
# =========================================================================
g_mm2_to_kg_m2 = 1e-9  # 1 g*mm^2 = 10^-9 kg*m^2

mtot = 0.85                          # Total mass (kg)
mp = 0.3                             # Pendulum mass (kg)
lp = 51.2 * 0.001                    # Distance from pivot to center of mass (m)
l_tot = (mp / mtot) * lp             # Pivot to center mass (m)
I_cm_motor = (1.291 * 10**5) * g_mm2_to_kg_m2  # Each motor COM inertia
I_cm_p = (1.749 * 10**5) * g_mm2_to_kg_m2      # Pendulum COM inertia
I_total = 2 * I_cm_motor + (I_cm_p + mp * lp**2)  # Total inertia around axle
g = 9.81                             # Gravity (m/s^2)
R_wheel = 0.065 / 2                  # Wheel radius (m)


# =========================================================================
# 2. Reduced 2D Controller Design (Tilt Angle Only: [theta, theta_dot])
# =========================================================================

def compute_2d_controller():
    print("=== 2D REDUCED CONTROLLER DESIGN ===")

    # State-Space Matrices
    A_2d = np.array([
        [0.0, 1.0],
        [(mtot * g * l_tot) / I_total, 0.0]
    ])

    B_2d = np.array([
        [0.0],
        [-(mtot * l_tot * R_wheel) / I_total]
    ])

    C_2d = np.eye(2)
    D_2d = np.zeros((2, 1))

    # Open-Loop Poles & Controllability
    open_loop_poles_2d = np.linalg.eigvals(A_2d)
    print(f"A_2D poles:\n{open_loop_poles_2d}")

    Wc_2d = control.ctrb(A_2d, B_2d)
    print(f"Rank of controllability matrix: {np.linalg.matrix_rank(Wc_2d)}")

    # LQR Gain Computation
    Q_2d = np.diag([5.0, 0.0])
    R_2d = np.array([[1.0]])

    K_2d, _, _ = control.lqr(A_2d, B_2d, Q_2d, R_2d)
    K_2d_flat = K_2d.flatten()

    cl_poles_2d = np.linalg.eigvals(A_2d - B_2d @ K_2d)
    print(f"Closed-Loop Poles:\n{cl_poles_2d}")
    print(f"double K[2] = {{ {K_2d_flat[0]:.4f}, {K_2d_flat[1]:.4f} }}; // Controller gains (u = -K*x)\n")

    sys_obs_2d = control.ss(A_2d - B_2d @ K_2d, np.zeros((2, 1)), C_2d, D_2d)
    t_2d = np.linspace(0, 5, 500)
    x0_2d = np.array([0.2, 0.0])

    response_2d = control.initial_response(sys_obs_2d, T=t_2d, X0=x0_2d)

    plt.figure("2D Observer Initial Response")
    plt.plot(response_2d.time, response_2d.outputs.T)
    plt.title("2D System + Observer Initial Condition Response")
    plt.xlabel("Time (seconds)")
    plt.ylabel("States")
    plt.legend(["theta", "theta_dot", "theta_hat", "theta_dot_hat"])
    plt.grid(True)


# =========================================================================
# 3. Parametric 5D Continuous LQI Controller Design
# =========================================================================
def compute_5d_controller():
    print("=== 5D LQI CONTROLLER DESIGN ===")

    A_theta = (mtot * g * l_tot) / I_total
    B_theta = -(mtot * l_tot * R_wheel) / I_total

    # Base 4D Continuous Matrices: [x, x_dot, theta, theta_dot]
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

    # 5D Augmentation for Integral Action: [x, x_dot, theta, theta_dot, integral_x]
    A_5d = np.block([
        [A_4d, np.zeros((4, 1))],
        [np.array([[1.0, 0.0, 0.0, 0.0]]), np.zeros((1, 1))]
    ])

    B_5d = np.block([
        [B_4d],
        [np.zeros((1, 1))]
    ])

    open_loop_poles_5d = np.linalg.eigvals(A_5d)
    print(f"Open Loop Poles:\n{open_loop_poles_5d}")

    # LQI Gain Computation
    Q_5d = np.diag([200.0, 20.0, 1000.0, 50.0, 500.0])
    R_5d = np.array([[1.0]])

    K_5d, _, _ = control.lqr(A_5d, B_5d, Q_5d, R_5d)
    K_5d_flat = K_5d.flatten()

    A_cl_5d = A_5d - B_5d @ K_5d
    cl_poles_5d = np.linalg.eigvals(A_cl_5d)
    print(f"Closed Loop Poles:\n{cl_poles_5d}")
    print(
        f"double K[5] = {{ {K_5d_flat[0]:.4f}, {K_5d_flat[1]:.4f}, "
        f"{K_5d_flat[2]:.4f}, {K_5d_flat[3]:.4f}, {K_5d_flat[4]:.4f} }}; // Controller gains\n"
    )


    # initial response simulation for 5D system
    sys_5d = control.ss(A_cl_5d, np.zeros((5, 1)), np.eye(5), np.zeros((5, 1)))
    x0_5d = np.array([-0.1, 0.0, 0.0, 0.0, 0.0])
    t_5d = np.linspace(0, 10, 1000)

    response_5d = control.initial_response(sys_5d, T=t_5d, X0=x0_5d)
    x_states = response_5d.states.T  # Shape: (1000, 5)

    # Extract individual states
    x_pos = x_states[:, 0]
    x_dot = x_states[:, 1]
    theta = x_states[:, 2]
    theta_dot = x_states[:, 3]
    x_integral = x_states[:, 4]

    # Derived Inputs
    w_motor = x_dot / R_wheel
    u_accel = -(x_states @ K_5d.T).flatten()

    # Plotting 5D Results
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title("LQI Simulation Results")

    # Plot 1: System States
    axs[0].plot(t_5d, x_pos, 'b', label='Position x (m)', linewidth=1.5)
    axs[0].plot(t_5d, theta, 'g', label=r'Angle $\theta$ (rad)', linewidth=1.5)
    axs[0].plot(t_5d, x_dot, 'r', label=r'$\dot{x}$ (m/s)', linewidth=1.5)
    axs[0].plot(t_5d, theta_dot, 'y', label=r'$\dot{\theta}$ (rad/s)', linewidth=1.5)
    axs[0].plot(t_5d, x_integral, 'k', label='integral(x)', linewidth=1.5)
    axs[0].set_ylabel('States')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)
    axs[0].set_title('System Response to -0.1m Initial Offset')

    # Plot 2: Motor Angular Velocity w(t)
    axs[1].plot(t_5d, w_motor, 'm', label=r'Motor Velocity $w(t)$', linewidth=1.5)
    axs[1].set_ylabel('w (rad/s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True)
    axs[1].set_title(r'Motor Angular Velocity Input $w = \dot{x} / R_{wheel}$')

    # Plot 3: Control Acceleration u(t)
    axs[2].plot(t_5d, u_accel, 'r', label=r'Control Input $u(t)$', linewidth=1.5)
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel(r'$\alpha$ (rad/s$^2$)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)
    axs[2].set_title(r'Commanded Angular Acceleration $u = -K \cdot x$')

# =========================================================================
# 5. Main Execution
# =========================================================================
# compute_2d_controller()
compute_5d_controller()

plt.tight_layout()
plt.show()