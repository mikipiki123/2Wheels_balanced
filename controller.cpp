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

    // Simple Exponential Moving Average (Low-Pass Filter)
    static float theta_dot_prev = 0.0;
    float alpha_theta_dot = 0.15; // Filter strength (0.05 = heavy smoothing, 0.3 = light)

    theta_dot_prev = (alpha_theta_dot * theta_dot_hat) + ((1.0 - alpha_theta_dot) * theta_dot_prev);

    // std::cout << "Estimated Angle: " << this->theta_hat << " rad | Estimated Angular Velocity: " << this->theta_dot_hat << " rad/s" << std::endl;
    // std::cout << "d_theta_hat: " << d_theta_hat << " | d_theta_dot_hat: " << d_theta_dot_hat << std::endl;

    // Instead of just printing total u:
    // float u_angle = -this->K[0] * this->theta_hat;
    // float u_rate  = -this->K[1] * theta_dot_prev;
    // float u_total = u_angle + u_rate;

    // printf("Angle: %f | u_angle: %f | u_rate: %f | TOTAL: %f\n", this->theta_hat, u_angle, u_rate, u_total);


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

    
    // float alpha_u = 0.9; // Filter strength (0.05 = heavy smoothing, 0.3 = light)

    // this->u_prev = (alpha_u * this->u) + ((1.0 - alpha_u) * this->u_prev);

    // Print format: angle,angular_velocity,voltage
    printf("%.4f,%.4f,%.4f\n", theta, theta_dot_prev, this->u);

    // 7. Return the updated class variable
    return this->u;
}

// double AngleController::Controller(double theta, double dt) {

//     // 1. Observer innovation error
//     double angle_error = theta - this->theta_hat;

//     // 2. Observer state derivatives
//     double d_theta_hat = this->theta_dot_hat + this->L[0] * angle_error;
//     double d_theta_dot_hat = (this->A_reduce[1][0] * this->theta_hat) + 
//                              (this->A_reduce[1][1] * this->theta_dot_hat) + 
//                              (this->B_reduce[1][0] * this->u) + 
//                              (this->L[1] * angle_error);

//     // 3. Integration
//     this->theta_hat += d_theta_hat * dt;
//     this->theta_dot_hat += d_theta_dot_hat * dt;

//     // 4. Smooth estimated velocity
//     static float theta_dot_prev = 0.0;
//     float alpha_theta_dot = 0.20; 
//     theta_dot_prev = (alpha_theta_dot * this->theta_dot_hat) + ((1.0 - alpha_theta_dot) * theta_dot_prev);

//     // 5. Calculate control voltage using smoothed rate
//     double u_unconstrained = -this->K[0] * this->theta_hat - this->K[1] * theta_dot_prev;

//     // 6. Saturation and deadzone mapping
//     double u_saturated = std::clamp(u_unconstrained, -this->v_max, this->v_max);
//     double abs_v = std::abs(u_saturated);
//     double noise_threshold = 0.15; // [V]

//     if (abs_v > noise_threshold && std::abs(this->theta_hat) > this->deadband) {
//         // Smooth deadzone offset mapping
//         double u_compensated = this->v_deadzone + 
//             ((abs_v - noise_threshold) / (this->v_max - noise_threshold)) * (this->v_max - this->v_deadzone);
            
//         u_compensated = std::clamp(u_compensated, 0.0, this->v_max);
//         this->u = std::copysign(u_compensated, u_saturated);
//     } else {
//         this->u = 0.0;
//     }

//     printf("%.4f,%.4f,%.4f\n", this->theta_hat, theta_dot_prev, this->u);

//     return this->u;
// }
