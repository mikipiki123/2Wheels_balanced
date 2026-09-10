

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "controller.hpp"
#include <chrono>
#include <thread>
#include "hardware.hpp"

#define PERIOD_MS 10 // 10 ms period for 100 Hz frequency

// // Waveshare Pico 10-DOF I2C pins
// #define I2C_PORT i2c1
// #define I2C_SDA 6 
// #define I2C_SCL 7
// #define IMU_ADDR 0x68

// // --- ICM20948 Registers ---
// #define ICM_WHOAMI      0x00 // Expected ID: 0xEA
// #define ICM_PWR_MGMT_1  0x06
// #define ICM_ACCEL_START 0x2D

// // --- MPU9250 Registers ---
// #define MPU_WHOAMI      0x75 // Expected ID: 0x71 (or 0x73 for MPU9255)
// #define MPU_PWR_MGMT_1  0x6B
// #define MPU_ACCEL_START 0x3B

// // Global variable to hold the correct starting register for the accelerometer
// uint8_t accel_reg_start = 0; 

// bool imu_init() {
//     uint8_t chip_id = 0;
//     uint8_t reg;
    
//     // 1. Try to read the ICM20948 Identity Register
//     reg = ICM_WHOAMI;
//     i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &reg, 1, true, 100000);
//     i2c_read_timeout_us(I2C_PORT, IMU_ADDR, &chip_id, 1, false, 100000);
    
//     if (chip_id == 0xEA) {
//         printf("Hardware Detected: ICM20948 (Rev 1.0)\n");
//         accel_reg_start = ICM_ACCEL_START;
        
//         // Wake up ICM20948
//         uint8_t buf[2] = {ICM_PWR_MGMT_1, 0x01};
//         i2c_write_timeout_us(I2C_PORT, IMU_ADDR, buf, 2, false, 100000);
//         sleep_ms(100);
//         return true;
//     }
    
//     // 2. Try to read the MPU9250 Identity Register
//     reg = MPU_WHOAMI;
//     i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &reg, 1, true, 100000);
//     i2c_read_timeout_us(I2C_PORT, IMU_ADDR, &chip_id, 1, false, 100000);
    
//     if (chip_id == 0x71 || chip_id == 0x73) {
//         printf("Hardware Detected: MPU9250/9255 (Rev 2.1)\n");
//         accel_reg_start = MPU_ACCEL_START;
        
//         // Wake up MPU9250
//         uint8_t buf[2] = {MPU_PWR_MGMT_1, 0x01};
//         i2c_write_timeout_us(I2C_PORT, IMU_ADDR, buf, 2, false, 100000);
//         sleep_ms(100);
//         return true;
//     }

//     printf("ERROR: Could not verify chip identity.\n");
//     return false;
// }

// bool imu_read_accel(float *accel_g) {

//     if (accel_reg_start == 0) return false; // Safety check
    
//     uint8_t data[6];
    
//     // Point to whichever register we dynamically assigned during init
//     int ret = i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &accel_reg_start, 1, true, 100000);
//     if (ret < 0) return false;
    
//     // Read 6 consecutive bytes 
//     ret = i2c_read_timeout_us(I2C_PORT, IMU_ADDR, data, 6, false, 100000);
//     if (ret < 0) return false;
    
//     // Combine high and low bytes (16-bit, big-endian)
//     int16_t accel_x = (data[0] << 8) | data[1];
//     int16_t accel_y = (data[2] << 8) | data[3];
//     int16_t accel_z = (data[4] << 8) | data[5];
    
//     // Convert to g (Both chips use a default scale factor of 16384 LSB/g)
//     accel_g[0] = (float)accel_x * (M_PI_2)/ 16384.0f;
//     accel_g[1] = (float)accel_y * (M_PI_2) / 16384.0f;
//     accel_g[2] = (float)accel_z * (M_PI_2) / 16384.0f;
    
//     return true;
// }

// GPIO Pin Definitions
    const uint STEP_PIN = 2; // GP2 (Pico Pin 4)
    const uint DIR_PIN  = 3; // GP3 (Pico Pin 5)
    const uint EN_PIN   = 4; // GP4 (Pico Pin 6)

    // Configuration Parameters
    // Set to 3200 for 1/16 microstepping (200 steps * 16)
    // Set to 1600 for 1/8 microstepping (MS1/MS2 left floating)
    const float STEPS_PER_REV = 200.0f; // 1/16 microstepping
    const float MICROSTEPS    = 16.0f; 
    const float RAD_TO_FREQ   = (STEPS_PER_REV * MICROSTEPS) / (2.0f * M_PI); // Conversion factor from rad/s to step frequency (Hz)
   

void set_motor_velocity(float rad_sec) {
    uint slice_num = pwm_gpio_to_slice_num(STEP_PIN);
    uint chan = pwm_gpio_to_channel(STEP_PIN);

    // Stop output if requested speed is near zero
    if (std::abs(rad_sec) < 0.001f || std::abs(rad_sec) > 30.0f) { // Safety limit to prevent excessive speed
        pwm_set_enabled(slice_num, false);
        return;
    }

    // Set Direction Pin (High = CW, Low = CCW)
    gpio_put(DIR_PIN, rad_sec > 0.0f);

    // Calculate step frequency in Hz
    float total_steps_per_rev = STEPS_PER_REV * MICROSTEPS;
    float step_freq = (std::abs(rad_sec) / (2.0f * M_PI)) * total_steps_per_rev;

    // RP2040 PWM frequency math: f_pwm = f_sys / (clkdiv * (wrap + 1))
    uint32_t sys_clk = clock_get_hz(clk_sys);
    
    // Dynamically calculate divider to maximize 16-bit wrap precision
    float divider = (float)sys_clk / (step_freq * 65536.0f);
    if (divider < 1.0f) divider = 1.0f;
    if (divider > 255.0f) divider = 255.0f; // Hardware limit for 8-bit integer component

    uint32_t wrap = (uint32_t)((float)sys_clk / (divider * step_freq)) - 1;
    if (wrap > 65535) wrap = 65535;

    // Apply hardware registers
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, (uint16_t)wrap);
    
    // 50% duty cycle provides clean square wave pulses for the TMC2209
    pwm_set_chan_level(slice_num, chan, (uint16_t)(wrap / 2));
    pwm_set_enabled(slice_num, true);
}


int main() {
    stdio_init_all();

    IMU_sensor imu; // Create an instance of the IMU_sensor class

    // // Initialize I2C for IMU
    // i2c_init(I2C_PORT, 400 * 1000);
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    
    sleep_ms(2000); 
    printf("Starting Auto-Detect IMU Accelerometer Read...\n");
    printf("--------------------------------------------\n");
    
    // if (!imu_init()) {
    //     printf("Halting. Check wiring.\n");
    //     while (1) { sleep_ms(1000); }
    // }

    // Initialize Pins
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    // Enable TMC2209 (Active LOW: LOW = Driver ON, HIGH = Motors Free-Wheeling)
    gpio_put(EN_PIN, 0);
    // Setup STEP Pins as PWM function outputs
    gpio_set_function(STEP_PIN, GPIO_FUNC_PWM);
    gpio_set_function(STEP_PIN, GPIO_FUNC_PWM);


    float theta_bias = 0.0;

    // {
    //     float sum = 0.0;
    //     for (int i = 0; i < 500; i++) {
    //         if (imu_read_accel(accel)) {
    //             float raw_angle = -atan2(accel[0], accel[2]); // negative sign to correct for mounting orientation
    //             sum += raw_angle;
    //         } else {
    //             printf("Read failed during calibration! I2C bus error.\n");
    //         }
    //         sleep_ms(2); // 2 ms delay for 100 Hz sampling during calibration
    //     }
    //     theta_bias = sum / 500.0;
    // }
    
    AngleController angleController;

    absolute_time_t next = get_absolute_time();

    double w = 0.0; // Control input (angular velocity in rad/s)

    while (true) {
    
        // schedule next execution time
        next = delayed_by_us(next, PERIOD_MS * 1000);
        
        if (true) { // imu_read_accel(accel)) {

            auto start = std::chrono::high_resolution_clock::now();

            // static double angle_filtered = 0.0;
            // double raw_angle = -atan2(accel[0], accel[2]); // negative sign to correct for mounting orientation
            // raw_angle -= theta_bias; // Remove bias from calibration
            
            // angle_filtered = 0.8 * angle_filtered + 0.2 * raw_angle; // Smooth raw accelerometer noise

            imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU

            double u = angleController.Controller(imu.angle_x, PERIOD_MS/1000.0); // dt = 0.01 s (10 ms) = 100 Hz

            w = w*0.98 + u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command, with Leaky Integrator - 0.98

            printf("%.4f,%.4f,%.4f\n", imu.angle_x, imu.angular_velocity_x, w);

            set_motor_velocity(w); // Apply control input to motors

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
        // sleep_ms(100);

    }

    return 0;
}
