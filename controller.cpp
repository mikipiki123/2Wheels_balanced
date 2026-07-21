#include "controller.hpp"

double AngleController::Controller(double theta, double dt) {

    // 1. Calculate observer angle error
    double angle_error = theta - this->theta_hat;

    // 2. Observer state derivatives (uses this->u from the PREVIOUS loop cycle)
    double d_theta_hat = this->theta_dot_hat + this->L[0] * angle_error;
    double d_theta_dot_hat = (this->A_reduce[1][0] * this->theta_hat) + 
                            (this->A_reduce[1][1] * this->theta_dot_hat) + 
                            (this->B_reduce[1][0] * this->u) + // Uses stored class variable
                            (this->L[1] * angle_error);

    // 3. Update estimates using Forward Euler integration
    this->theta_hat += d_theta_hat * dt;
    this->theta_dot_hat += d_theta_dot_hat * dt;

    // std::cout << "Estimated Angle: " << this->theta_hat << " rad | Estimated Angular Velocity: " << this->theta_dot_hat << " rad/s" << std::endl;
    // std::cout << "d_theta_hat: " << d_theta_hat << " | d_theta_dot_hat: " << d_theta_dot_hat << std::endl;


    // 4. Compute unconstrained control input from estimated states
    double u_unconstrained = -this->K[0] * this->theta_hat - this->K[1] * this->theta_dot_hat;

    // 5. Apply saturation [-v_max, +v_max]
    double u_saturated = std::clamp(u_unconstrained, -this->v_max, this->v_max);
    double abs_v = std::abs(u_saturated);

    // 6. UPDATE THE CLASS MEMBER VARIABLE directly (do not declare 'double u')
    if (std::abs(theta) > this->deadband) {
        // Map control demand [0V to 5V] into active window [4V to 5V]
        double u_compensated = this->v_deadzone + (abs_v / this->v_max) * (this->v_max - this->v_deadzone);

        // Save directly to class member variable this->u
        this->u = std::copysign(u_compensated, u_saturated);
    } else {
        this->u = 0.0; // Turn off motors when balanced within deadband
    }

    // 7. Return the updated class variable
    return this->u;
}
