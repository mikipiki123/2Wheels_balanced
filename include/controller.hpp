#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>

class SSController { //state-space controller base class
    
    public:
    // Given parameters
    double Ts = 0.005; // Sampling time (s)

    double mtot = 0.92 + 0.25;      // total mass (kg)
    double mp = 0.4 + 0.25;         // Pendulum mass (kg)
    double lp = 120.17 * 0.001;         // Distance from pivot to center of mass (m)
    // double I_cm_motor = (1.291*std::pow(10, 5)) * 1e-9; // each motor COM inertia (kg*m^2)
    // double I_cm_p = (1.749*std::pow(10, 5)) * 1e-9; // pendulum COM inertia (kg*m^2)
    double g = 9.81;
    double R_wheel = 0.065/2; // wheel radius (m)

    double mbat = 0.25;                          // Battery mass (kg)
    double lbat = 0.18;                          // Battery distance from pivot to center of mass (m)

    // Calculated parameters
    double l_tot = (mp/mtot)*lp; // pivot to COM (center of mass) (m)
    double Ibat = (1.0 / 12.0) * mbat * std::pow(lbat, 2); // Battery inertia (kg*m^2)
    double I_total = 0.00470 + (Ibat + mbat * std::pow(lbat, 2)); // Total inertia around axle
    double u = 0.0; // control input (rad/s^2)
    double w = 0.0; // Control input to motors (angular velocity in rad/s) - integrated acceleration

    // virtual double Controller(double theta, double theta_dot, double dt) = 0; // Pure virtual function for controller implementation

};

class FullController : public SSController {

    public:

    double x_dot = 0.0; // Linear velocity of the robot (m/s)
    double x = 0.0; // Position in meters
    double integral_action_pos = 0.0; // Integral action for position control

    //low pass filter parameters
    double x_dot_prev = 0.0; // Previous linear velocity for low-pass filter

    double target_x_prev = 0.0; // Previous target position for derivative action

    double Controller(double theta, double theta_dot, double dt, double target_x);
    void integrate_velocity(double dt);
    void tune_gains(double dt, double target_x);


    private:

    // // state-space representation of the system - [x, x_dot, theta, theta_dot], u = acceleration (rad/s^2) //todo - add integral action state!!!!!
    std::array<std::array<double, 5>, 5> Ad = {{
        {1, Ts, 0, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 1, Ts, 0},
        {0, 0, (mtot * g * l_tot* Ts) / I_total, 1, 0},
        {Ts, 0, 0, 0, 1}
    }};

    std::array<std::array<double, 1>, 5> Bd = {{
        {0},
        {R_wheel * Ts},
        {0},
        {-(mtot * l_tot * R_wheel * Ts) / I_total},
        {0}
    }};

    std::array<std::array<double, 5>, 5> Cd = {{
        {1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1}
    }};

// double K[5] = { -56.5440, -67.0199, -715.3253, -40.6780, -50.3607 }; // Controller gains (u = -K*x) - good gains
// double K[5] = { -50.5090, -60.3163, -350.2072, -30.7753, -20.7566 };
    double K[5] = { -30.2407f, -100.7864f, -400.2284f, -30.4034f, -6.9480f }; // Controller gains
};


class AngleController : public SSController {

    public:


    double Controller(double theta, double theta_dot, double dt);


    private:


    double theta_dot_prev = 0.0;

    std::array<std::array<double, 2>, 2> A_reduce = {{ {0, 1},
                                                    {(mtot*l_tot*g)/I_total, 0} }};

    std::array<std::array<double, 1>, 2> B_reduce = {{ {0}, 
                                                    {-(mtot*l_tot*R_wheel)/I_total} }}; // when u defined as acceleration (rad/s^2)

    std::array<std::array<double, 2>, 1> C_reduce = {{ {1, 0} }};

    double K[2] = { -603.7006, -54.3107 }; // Controller gains (u = -K*x)
    
};

