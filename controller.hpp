#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>

class Controller {

    public:

    double R_wheel = 0.065/2; // [m]
    double mr = 0.18; // [kg] - mass of rotation system
    double Jr = 0.000025; // [kg*m^2] - moment of inertia of rotation system
    double Fr = 0.2; // [N/(rad*s)] - friction of rotation system
    double L = 0.0412; // [m] - Length measured from the wheel axis and mass center
    double mb = 0.3; // [kg] - mass of main body - 76(g) + PCB
    double Jb = 0.0000996377; // [kg*m^2] - moment of inertia of main system
    double Fb = 0.002; // [N/(rad*s)] - friction of main system
    double g = 9.81; // [m/s^2] - gravitational acceleration



    private:

    double p = Jb*(mb+mr) + mb*mr*(std::pow(L, 2.0)); // denominator - just for simplicity

    std::array<std::array<double, 4>, 4> A = {{
        {0, 1, 0, 0},
        {0, -Fr*(Jb+mb*std::pow(L, 2.0))/p, (std::pow(mb, 2.0) * std::pow(L, 2.0) * g)/p, 0},
        {0, 0, 0, 1},
        {0, -Fr*mb*L/p, mb*g*(mb+mr)/p, 0}
    }};

    std::array<std::array<double, 1>, 4> B = {{
        {0},
        {(Jb+mb*std::pow(L, 2.0))/p},
        {0},
        {mb*L/p}
    }};

    std::array<std::array<double, 4>, 2> C = {{
        {1, 0, 0, 0},
        {0, 0, 1, 0}
    }};



};


class AngleController {

    public:

    const double R_wheel = 0.0325; // [m]
    const double mw = 0.039;          // Mass of ONE wheel [kg]
    const double Length = 0.05; // [m] - Length measured from the wheel axis and mass center
    const double mb = 0.141; // [kg] - mass of main body - 76(g) + PCB
    const double g = 9.81; // [m/s^2] - gravitational acceleration
    const double Jb = 0.001135; // [kg*m^2] - moment of inertia of main system




    // --- Motor Parameters (JGB37-520 Typical Values) ---
    const double R_motor  = 6.5;     // [Ohm] Terminal resistance
    const double Kt       = 0.015;   // [Nm/A] Torque constant
    const double Ke       = 0.015;   // [V/(rad/s)] Back-EMF constant
    const double G_ratio  = 9.6;    // Gear ratio
    const double num_motors = 2;      // Total number of drive motors

    double v_max = 5.0; // [V] - maximum voltage applied to the motor
    double v_deadzone = 0.4; // [V] - voltage below which the motor does not respond
    double deadband = 0.01; // [rad] - angle range within which the controller does not act (2.86 degrees)

    double Controller(double theta, double dt);


    private:

    // double J_axle = Jb + mb * (std::pow(Length, 2.0)); // inertia in motor's axis
    double Jtot = Jb + mb * std::pow(Length, 2.0); 

    // initial parameters
    double theta_hat = 0.0; // estimated angle
    double theta_dot_hat = 0.0; // estimated angular velocity
    double u = 0.0; // control input
    double u_prev = 0.0; // previous control input

    std::array<std::array<double, 2>, 2> A_reduce = {{ {0, 1},
                                                    {(mb * g * Length) / Jtot, -((num_motors * Kt * Ke * std::pow(G_ratio, 2.0)) / (R_motor * Jtot))} }};

    std::array<std::array<double, 1>, 2> B_reduce = {{ {0}, num_motors * (Kt * G_ratio) / (R_motor * Jtot)}};

    std::array<std::array<double, 2>, 1> C_reduce = {{ {1, 0} }};

    double K[2] = { 4.2346, 0.2377 }; // Controller gains - to be tuned
    double L[2] = { 40.5156, 244.7479 }; // Observer gains - to be tuned
    
};

