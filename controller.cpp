#include "controller.hpp"

double AngleController::Controller(double theta, double dt) {

    // 1. Calculate observer angle error
    double angle_error = theta - this->theta_hat;

    // std::cout << "Angle Error: " << angle_error << " rad | Estimated Angle: " << this->theta_hat << " rad | Measured Angle: " << theta << " rad" << std::endl;

    // 2. Observer state derivatives
    double d_theta_hat = this->theta_dot_hat + this->L[0] * angle_error;
    double d_theta_dot_hat = (this->A_reduce[1][0] * this->theta_hat) + 
                             (this->A_reduce[1][1] * this->theta_dot_hat) + 
                             (this->B_reduce[1][0] * this->u) + 
                             (this->L[1] * angle_error);

    // 3. Update estimates using Forward Euler integration
    this->theta_hat += d_theta_hat * dt;
    this->theta_dot_hat += d_theta_dot_hat * dt;

    const double alpha = 0.3; // Low-pass filter coefficient for angular velocity
    this->theta_dot_hat = alpha * this->theta_dot_hat + (1 - alpha) * this->theta_dot_prev; // Apply low-pass filter to estimated angular velocity
    this->theta_dot_prev = this->theta_dot_hat; // Store current estimate for next iteration

    // 4. Compute control input using estimated angular velocity (FIXED)
    this->u = -this->K[0] * this->theta_hat - this->K[1] * this->theta_dot_prev;

    if (std::abs(this->theta_hat) < 0.02) { // Deadband threshold of 0.02 rad (~1.15 degrees)
        this->u = 0.0; // No control action within deadband
    }

    // // 5. Print format: estimated angle, estimated velocity, control input
    // printf("%.4f,%.4f,%.4f\n", this->theta_hat, this->theta_dot_hat, this->u);

    return this->u;
}

