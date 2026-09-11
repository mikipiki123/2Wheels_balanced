#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>

class FullController {

    public:

    double mtot = 0.85;      // total mass (kg)
    double mp = 0.3;         // Pendulum mass (kg)
    double lp = 51.2*0.001;         // Distance from pivot to center of mass (m)
    double l_tot = (mp/mtot)*lp; // pivot to center mass (m)
    double I_cm_motor = (1.291*std::pow(10, 5)) * 1e-9; // each motor COM inertia
    double I_cm_p = (1.749*std::pow(10, 5)) * 1e-9; // pendulum COM inertia
    double I_total = 2 * I_cm_motor + (I_cm_p + mp*std::pow(lp, 2)); // total inertia around rotation point (center of motors)
    double g = 9.81;
    double R_wheel = 0.065/2; // (m)

    double u = 0.0; // control input (rad/s^2)
    double w = 0.0; // Control input to motors (angular velocity in rad/s)
    double x_dot = 0.0; // Linear velocity of the robot (m/s)
    double x = 0.0; // Position in meters
    double integral_action_pos = 0.0; // Integral action for position control

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

double K[5] = { -24.6576, -42.2849, -674.7556, -60.7790, -7.0711 }; // Controller gains (u = -K*x)

};


class AngleController {

    public:

    // const double R_wheel = 0.0325; // [m]
    // const double mw = 0.039;          // Mass of ONE wheel [kg]
    const double Length = 0.1; // [m] - Length measured from the wheel axis and mass center
    const double mb = 0.5; // [kg] - mass of main body - 76(g) + PCB
    const double g = 9.81; // [m/s^2] - gravitational acceleration
    const double Jb = 0.005; // [kg*m^2] - moment of inertia of main system
    const double b_fric = 0.001; // [N*m*s] - Viscous friction/damping coefficient at the motor shaft


    // double v_max = 5.0; // [V] - maximum voltage applied to the motor
    // double v_deadzone = 0.0; // [V] - voltage below which the motor does not respond
    // double deadband = 0.02; // [rad] - angle range within which the controller does not act (2.86 degrees)

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

