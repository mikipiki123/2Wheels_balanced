# 2 Wheels self balancing robot

A self-balancing robot modeled using a linearized state-space representation and implemented on a Raspberry Pi Pico with an IMU sensor and NEMA 17 stepper motors. The project includes the physical system modeling and core real-time control algorithm, along with Python-based tools for experimental data analysis, signal processing, and real-time visualization of system states and control signals.

---
## Robot in action

https://github.com/user-attachments/assets/70e26cd5-e973-4f93-8034-186bfde12b96

---
## 📌 Features

* **5D LQI State Control:** Regulates Robot's linear position, linear velocity, body tilt angle, pitch angular velocity, and integrated position error to guarantee zero steady-state position drift.
* **2D LQR State Control:** Regulates only Robot's angle and angular velocity.


**Note:** You can easily change between those two algorithms by comment/uncomment instances on main.cpp


* **Real-Time plot:** Displays states of the system in real time and records to signal_records
* **Vibration analysis:** dsp.py introduces a signal proccess analysis of chosen state. I added a plot of filtered signal by 1st-order IIR with cutoff on frequency 10 (Hz), that also implemented on the IMU gyro output on hardware.cpp

---

## 🧮 Control Architecture & Physics

The robot is modeled as a non-minimum phase coupled inverted pendulum system operating at a $100\text{ Hz}$ control loop ($T_s = 10\text{ ms}$).

### 5D State Vector
$$\mathbf{x} = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} & x_{\text{int}} \end{bmatrix}^T$$

Where $x_{\text{int}} = \int (x - x_{\text{ref}}) \, dt$.

### State-Space Representation
$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u$$
$$u = -\mathbf{K}\mathbf{x}$$

$$\mathbf{A} = \begin{bmatrix}  0 & 1 & 0 & 0 & 0 \\  0 & 0 & 0 & 0 & 0 \\  0 & 0 & 0 & 1 & 0 \\  0 & 0 & \frac{M_{\text{tot}} g l_{\text{tot}}}{I_{\text{axle}}} & 0 & 0 \\  1 & 0 & 0 & 0 & 0  \end{bmatrix}, \quad  \mathbf{B} = \begin{bmatrix}  0 \\  R_{\text{wheel}} \\  0 \\  -\frac{M_{\text{tot}} l_{\text{tot}} R_{\text{wheel}}}{I_{\text{axle}}} \\  0  \end{bmatrix}$$

* **Note:** 2D States is a reduced representation that uses the state vector: $\mathbf{x} = \begin{bmatrix} \theta & \dot{\theta} \end{bmatrix}^T$

---

## DSP Analysis:

The next graph represents the frequency analysis of the angular velocity:


<img src="images/theta_dot_before_filtering.png" width="600">

The natural frequency of the robot is ~3-4 Hz, so considering the noise reduction and the potential phase-lag i decided to implement the 1st-order IIR filter with cutoff of 10 Hz

## 🚀 Quick Start

### 1. Embedded Firmware Setup
Build and flash the firmware using the Pico SDK from root:
```bash
mkdir build && cd build
cmake ..
make
cp 2Wheels_balanced.uf2 /Volumes/RPI-RP2
```

## Future updates:
* **Physical structure:** To reduce natural frequency the robot center of mass should be higher - increasing inertia.
* **Electronic scheme:** Desinging a desired PCB will reduce wiring cables and potential damage to the system. Also, adding a battery to the top could prevent external disturbance from the power cable (The current system really sensitive to it).
* **Algorithms:** Extending the robot capabilities to track references, to yaw in place, 2D space moving.
