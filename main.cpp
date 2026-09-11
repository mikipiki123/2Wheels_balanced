

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include <chrono>
#include <thread>

#include "controller.hpp"
#include "hardware.hpp"

#define PERIOD_MS 5 // 5 ms period for 200 Hz frequency

int main() {
    stdio_init_all();

    IMU_sensor imu; // Create an instance of the IMU_sensor class

    printf("Starting IMU read...\n");
    printf("--------------------------------------------\n");

    MotorController motorController; // Create an instance of the MotorController class

   
    printf("Initializing Motor Controller...\n");
    printf("--------------------------------------------\n");


    float theta_bias = 0.0;
    // Calibrate the IMU to find the bias for the angle measurement
    printf("Calibrating IMU...\n");
    printf("--------------------------------------------\n");

    sleep_ms(2000); // Wait for 2 seconds to allow the IMU to stabilize

    
    // for (int i = 0; i < 100; i++) {
    //     imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
    //     sleep_ms(10); // 10 ms delay for 100 Hz sampling during calibration
    // }

    // {
    //     double i = 0.0;

    //     while (std::abs(imu.angle_x) >= 0.02 || i < 10.0) { // Wait until the initial angle is within a reasonable range for calibration
    //     printf("Waiting for IMU to stabilize... Current angle: %.4f rad, i = %.1f\n", imu.angle_x, i);
    //     imu.read_sensor_fusion_x(PERIOD_MS * 1000);
    //     if (std::abs(imu.angle_x) < 0.02) i += 1.0; else i = 0.0;
    //     sleep_ms(10);
    // }

    // }


    // if (std::abs(imu.angle_x) < 0.02) { // Check if the initial angle is within a reasonable range for calibration
    //     float sum = 0.0;
    //     for (int i = 0; i < 500; i++) {
    //         imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
    //         float raw_angle = imu.angle_x; // Get the raw angle from the IMU
    //         if (!std::isnan(raw_angle)) {
    //             sum += raw_angle;
    //         } else {
    //             printf("Read failed during calibration! I2C bus error.\n");
    //         }
    //         sleep_ms(2); // 2 ms delay for 100 Hz sampling during calibration
    //     }
    //     theta_bias = sum / 500.0;
    // }
    // sleep_ms(2000); 
    
    // AngleController angleController;
    FullController fullController; // Create an instance of the FullController class

    absolute_time_t next = get_absolute_time();

    // double w = 0.0; // Control input (angular velocity in rad/s)
    // double x_dot
    // double x = 0.0; // Position in meters

    while (true) {
    
        // schedule next execution time
        next = delayed_by_us(next, PERIOD_MS * 1000);
        
        if (true) { // imu_read_accel(accel)) {

            auto start = std::chrono::high_resolution_clock::now();

            imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
            // imu.angle_x -= theta_bias; // Remove bias from calibration

            double u = fullController.Controller(imu.angle_x, imu.angular_velocity_x, PERIOD_MS/1000.0); // dt = 0.005 s (5 ms) = 200 Hz

            fullController.w = fullController.w*0.98 + u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command, with Leaky Integrator - 0.98

            printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",fullController.x, fullController.x_dot, imu.angle_x, imu.angular_velocity_x, fullController.w);

            motorController.set_motor_velocity(fullController.w); // Apply control input to motors

            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            // std::cout << "Execution time: " << elapsed_ms << " ms" << std::endl; // Ensure elapsed_ms is consistently <= 10.0 ms!

        

            // printf("Accel X: %5.3f g | Y: %5.3f g | Z: %5.3f g\n", accel[0], accel[1], accel[2]); // Assuming accel[0] is the angle in radians

            
            // std::cout << "Angle: " << angle_rad << "\t rad | Control Input (Voltage): " << u << "\tV" << std::endl;

        } else {
            printf("Read failed! I2C bus error.\n");
        }
        
        // sleep until the exact next time
        sleep_until(next);

    }

    return 0;
}
