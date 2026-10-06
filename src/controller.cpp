#include "controller.hpp"



void FullController::integrate_velocity(double dt) {

    if (std::abs(this->w) < 100.0) { // Only integrate if angular velocity is within safe limits
        x_dot = w * R_wheel; // Update linear velocity based on current angular velocity
        // x = x*0.995 + x_dot * dt;     // Integrate to get position with leaky integrator (0.98) to prevent drift
        x += x_dot * dt; // Integrate to get position without leaky integrator

        double alpha = 0.30; // Low-pass filter coefficient (0 < alpha < 1)
        x_dot = alpha * x_dot + (1.0 - alpha) * x_dot_prev; // Apply low-pass filter to linear velocity
        x_dot_prev = x_dot; // Update previous linear velocity for next iteration

    } else {
        u = 0.0; // If angular velocity exceeds threshold, set control input to zero
    }

}

double FullController::Controller(double theta, double theta_dot, double dt, double target_x) {

    double e_x = this->x - target_x; // Position error

    integrate_velocity(dt);

    this->integral_action_pos += e_x * dt; // Integrate position for integral action

    if (std::abs(e_x) < 0.005) { // Only integrate position when within 0.5 cm of the origin
        this->integral_action_pos *= 0.96; // Apply leaky integrator to prevent windup
    }

    this->integral_action_pos = std::clamp(this->integral_action_pos, -0.8, 0.8); // Clamp integral action to prevent windup

    this->u = -this->K[0] * (e_x) 
              -this->K[1] * this->x_dot 
              -this->K[2] * theta 
              -this->K[3] * theta_dot
              -this->K[4] * this->integral_action_pos //todo - try without integral.
            ;

    
    // // Apply deadband to control input
    // if (std::abs(theta) < 0.008) { // Deadband threshold of 0.008 rad (~0.46 degrees)
    //     this->u = 0.0; // No control action within deadband
    // }

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