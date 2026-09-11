#include "controller.hpp"

void FullController::integrate_velocity(double dt) {
    x_dot = w * R_wheel; // Update linear velocity based on current angular velocity
    // x = x*0.995 + x_dot * dt;     // Integrate to get position with leaky integrator (0.98) to prevent drift
    x += x_dot * dt; // Integrate to get position without leaky integrator
}

double FullController::Controller(double theta, double theta_dot, double dt) {

    integrate_velocity(dt);

    this->integral_action_pos += x * dt; // Integrate position for integral action

    // Compute the control input using state feedback
    this->u = -this->K[0] * this->x - this->K[1] * this->x_dot - this->K[2] * theta - this->K[3] * theta_dot - this->K[4] * integral_action_pos; // Added integral action for position control

    // Apply deadband to control input
    if (std::abs(theta) < 0.008) { // Deadband threshold of 0.008 rad (~0.46 degrees)
        this->u = 0.0; // No control action within deadband
    }

    return this->u;
}

double AngleController::Controller(double theta, double theta_dot, double dt){

    // Compute the control input using state feedback
    this->u = -this->K[0] * theta - this->K[1] * theta_dot;

    // Apply deadband to control input
    if (std::abs(theta) < 0.008) { // Deadband threshold of 0.008 rad (~0.46 degrees)
        this->u = 0.0; // No control action within deadband
    }

    return this->u;
}