#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>

class SSController {
    
    public:
    // Given parameters
    double mtot = 0.85;      // total mass (kg)
    double mp = 0.3;         // Pendulum mass (kg)
    double lp = 51.2*0.001;         // Distance from pivot to center of mass (m)
    double I_cm_motor = (1.291*std::pow(10, 5)) * 1e-9; // each motor COM inertia (kg*m^2)
    double I_cm_p = (1.749*std::pow(10, 5)) * 1e-9; // pendulum COM inertia (kg*m^2)
    double g = 9.81;
    double R_wheel = 0.065/2; // wheel radius (m)

    // Calculated parameters
    double l_tot = (mp/mtot)*lp; // pivot to COM (center of mass) (m)
    double I_total = 2 * I_cm_motor + (I_cm_p + mp*std::pow(lp, 2)); // total inertia around rotation point (center of motors)
    double u = 0.0; // control input (rad/s^2)
    double w = 0.0; // Control input to motors (angular velocity in rad/s) - integrated acceleration

    // virtual double Controller(double theta, double theta_dot, double dt) = 0; // Pure virtual function for controller implementation




};

class FullController {

    public:

    double mtot = 0.85;      // total mass (kg)
    double mp = 0.3;         // Pendulum mass (kg)
    double lp = 51.2*0.001;         // Distance from pivot to center of mass (m)
    double l_tot = (mp/mtot)*lp; // pivot to COM (center of mass) (m)
    double I_cm_motor = (1.291*std::pow(10, 5)) * 1e-9; // each motor COM inertia (kg*m^2)
    double I_cm_p = (1.749*std::pow(10, 5)) * 1e-9; // pendulum COM inertia (kg*m^2)
    double I_total = 2 * I_cm_motor + (I_cm_p + mp*std::pow(lp, 2)); // total inertia around rotation point (center of motors)
    double g = 9.81;
    double R_wheel = 0.065/2; // wheel radius (m)

    double u = 0.0; // control input (rad/s^2)
    double w = 0.0; // Control input to motors (angular velocity in rad/s)
    double x_dot = 0.0; // Linear velocity of the robot (m/s)
    double x = -0.1; // Position in meters
    double integral_action_pos = 0.0; // Integral action for position control

    //low pass filter parameters
    double x_dot_prev = 0.0; // Previous linear velocity for low-pass filter

    double Controller(double theta, double theta_dot, double dt);
    void integrate_velocity(double dt);


    private:

    // state-space representation of the system - [x, x_dot, theta, theta_dot], u = acceleration (rad/s^2)
    std::array<std::array<double, 4>, 4> A = {{
        {0, 1, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, (mtot*g*l_tot)/I_total, 0}
    }};

    std::array<std::array<double, 1>, 4> B = {{
        {0},
        {R_wheel},
        {0},
        {-(mtot*l_tot*R_wheel)/I_total}
    }};

    std::array<std::array<double, 4>, 4> C = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};

// double K[5] = { -56.5440, -67.0199, -715.3253, -40.6780, -50.3607 }; // Controller gains (u = -K*x) - good gains
// double K[5] = { -56.5440, -67.0199, -715.3253, -40.6780, -50.3607 }; // Controller gains (u = -K*x)
double K[5] = { -53.5090, -62.3163, -677.2072, -61.7753, -80.7566 };

};


class AngleController {

    public:


    double Controller(double theta, double theta_dot, double dt);


    // private:

    // double J_axle = Jb + mb * (std::pow(Length, 2.0)); // inertia in motor's axis
    // double Jtot = Jb + mb * std::pow(Length, 2.0); 
    double Jtot = 0.005;

    // initial parameters
    double theta_hat = 0.0; // estimated angle
    double theta_dot_hat = 0.0; // estimated angular velocity
    double u = 0.0; // control input

    double theta_dot_prev = 0.0;

    std::array<std::array<double, 2>, 2> A_reduce = {{ {0, 1},
                                                    {123.56, 0} }};

    // std::array<std::array<double, 1>, 2> B_reduce = {{ {0}, 
    //                                                 {1 / Jtot} }}; // when u defined as torque

    std::array<std::array<double, 1>, 2> B_reduce = {{ {0}, 
                                                    {-0.409}}}; // when u defined as acceleration (rad/s^2)

    std::array<std::array<double, 2>, 1> C_reduce = {{ {1, 0} }};

    double K[2] = { -603.7006, -54.3107 }; // Controller gains (u = -K*x)
    // double L[2] = { 35, 250 }; // Luenberger observer gains
    
};

