#include "controller.hpp"

#include <cmath>
#include <complex>
#include <algorithm>

std::array<double, 5> compute_ackermann_5d(std::array<std::array<double, 5>, 5> Ad, std::array<std::array<double, 1>, 5> Bd, const std::complex<double> poles[5]) {
    // 1. Build Controllability Matrix C = [Bd, Ad*Bd, Ad^2*Bd, Ad^3*Bd, Ad^4*Bd]
    double C_mat[5][5];
    for (int i = 0; i < 5; i++) C_mat[i][0] = Bd[i][0];

    for (int col = 1; col < 5; col++) {
        for (int i = 0; i < 5; i++) {
            C_mat[i][col] = 0.0;
            for (int k = 0; k < 5; k++) {
                C_mat[i][col] += Ad[i][k] * C_mat[k][col - 1];
            }
        }
    }

    // 2. Solve C^T * q = [0, 0, 0, 0, 1]^T using Gaussian Elimination
    double M[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            M[i][j] = C_mat[j][i]; // Transpose C
        }
    }
    double b[5] = {0.0, 0.0, 0.0, 0.0, 1.0};

    for (int i = 0; i < 5; i++) {
        int max_row = i;
        for (int k = i + 1; k < 5; k++) {
            if (std::abs(M[k][i]) > std::abs(M[max_row][i])) max_row = k;
        }
        for (int k = 0; k < 5; k++) std::swap(M[i][k], M[max_row][k]);
        std::swap(b[i], b[max_row]);

        for (int k = i + 1; k < 5; k++) {
            double factor = M[k][i] / M[i][i];
            for (int j = i; j < 5; j++) {
                M[k][j] -= factor * M[i][j];
            }
            b[k] -= factor * b[i];
        }
    }

    // Back substitution to solve for vector q
    double q[5] = {0};
    for (int i = 4; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < 5; j++) sum += M[i][j] * q[j];
        q[i] = (b[i] - sum) / M[i][i];
    }

    // 3. Expand characteristic polynomial P(z) = (z - z1)(z - z2)...(z - z5)
    std::complex<double> poly[6] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int p = 0; p < 5; p++) {
        std::complex<double> next_poly[6] = {0.0};
        for (int i = 0; i <= p; i++) {
            next_poly[i + 1] += poly[i];
            next_poly[i]     -= poly[i] * poles[p];
        }
        for (int i = 0; i <= p + 1; i++) poly[i] = next_poly[i];
    }

    // 4. Compute phi(Ad) = Ad^5 + a4*Ad^4 + a3*Ad^3 + a2*Ad^2 + a1*Ad + a0*I
    double phi[5][5] = {0};
    for (int i = 0; i < 5; i++) phi[i][i] = poly[0].real(); // a0 * I

    double Ad_pow[5][5];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            Ad_pow[i][j] = Ad[i][j];

    for (int p = 1; p <= 5; p++) {
        double coeff = poly[p].real();
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                phi[i][j] += coeff * Ad_pow[i][j];
            }
        }
        if (p < 5) {
            double temp[5][5] = {0};
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    for (int k = 0; k < 5; k++) {
                        temp[i][j] += Ad_pow[i][k] * Ad[k][j];
                    }
                }
            }
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 5; j++)
                    Ad_pow[i][j] = temp[i][j];
        }
    }

    double K[5] = {0.0};

    // 5. Compute Gain Vector: K = q^T * phi(Ad)
    for (int j = 0; j < 5; j++) {
        K[j] = 0.0;
        for (int i = 0; i < 5; i++) {
            K[j] += q[i] * phi[i][j];
        }
    }
    return {K[0], K[1], K[2], K[3], K[4]};
}

void FullController::tune_gains(double dt, double target_x) {
    double Ts = dt; // Sampling time (5 ms)

    // 2. Compute s-plane poles matching control_analysis.py exactly
    double e_x = std::abs(target_x - this->target_x_prev);

    double s_pos1   = -2.0 / (std::pow(e_x, 4.0) + 0.8);
    double s_pos2   = -2.0 / (std::pow(e_x, 0.5) + 1.5);
    double s_tilt_r = -4.50;
    double s_tilt_i =  4.50;                     
    // double s_int    = -0.20/(std::pow(e_x, 1.2)+ 3.0);
    double s_int    = -0.20;                           

    std::complex<double> s_poles[5] = {
        std::complex<double>(s_pos1, 0),
        std::complex<double>(s_pos2, 0),
        std::complex<double>(s_tilt_r,  s_tilt_i),
        std::complex<double>(s_tilt_r, -s_tilt_i),
        std::complex<double>(s_int, 0)
    };

    // 3. Map s-domain poles to z-domain: z = exp(s * Ts)
    std::complex<double> z_poles[5];
    for (int i = 0; i < 5; i++) {
        z_poles[i] = std::exp(s_poles[i] * Ts);
    }

    // 4. Compute gains using discretized Ad_5d and Bd_5d
    auto new_K = compute_ackermann_5d(this->Ad, this->Bd, z_poles);
    
    // 5. Assign computed gains to Controller
    for (int i = 0; i < 5; ++i) {
        this->K[i] = new_K[i];
    }

    K[3] *= 0.60; // reduce the gain for theta_dot to avoid overshoot

    printf("Gains updated: K = [%.4f, %.4f, %.4f, %.4f, %.4f]\n\n", 
           this->K[0], this->K[1], this->K[2], this->K[3], this->K[4]);
}

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

    if (std::abs(e_x) < 0.02 && std::abs(this->x_dot) > 0.2) { // Only apply leaky integrator when close to target
        this->integral_action_pos *= 0.99; // Apply leaky integrator to prevent windup
    }

    if (target_x != this->target_x_prev) {
        this->integral_action_pos = 0.0; // Reset integral action when target position changes
        tune_gains(dt, target_x); // Re-tune gains based on new target position
        this->target_x_prev = target_x;   // Update previous target position
    }

    this->integral_action_pos = std::clamp(this->integral_action_pos, -50.0, 50.0); // Clamp integral action to prevent windup

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