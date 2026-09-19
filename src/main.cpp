

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

#define PERIOD_MS 10 // 10 ms period for 100 Hz frequency

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

    
    // AngleController angleController;
    FullController fullController; // Create an instance of the FullController class

    absolute_time_t next = get_absolute_time();


    while (true) {
    
        // schedule next execution time
        next = delayed_by_us(next, PERIOD_MS * 1000);
        
        if (true) { // imu_read_accel(accel)) {

            auto start = std::chrono::high_resolution_clock::now();

            imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
            // imu.angle_x -= theta_bias; // Remove bias from calibration

            double u = fullController.Controller(imu.angle_x, imu.angular_velocity_x, PERIOD_MS/1000.0); // dt = 0.005 s (5 ms) = 200 Hz

            // fullController.w = fullController.w*0.98 + u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command, with Leaky Integrator - 0.98
            // without leaky integrator:
            fullController.w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command

            printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                fullController.x, 
                fullController.x_dot, 
                imu.angle_x, 
                imu.angular_velocity_x, 
                fullController.integral_action_pos, 
                fullController.w); // for graph RTplot.py

            motorController.set_motor_velocity(fullController.w); // Apply control input to motors

            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            // std::cout << "Execution time: " << elapsed_ms << " ms" << std::endl; // Ensure elapsed_ms is consistently <= 10.0 ms!

        } else {
            printf("Read failed! I2C bus error.\n");
        }
        
        // sleep until the exact next time
        sleep_until(next);

    }

    return 0;
}
