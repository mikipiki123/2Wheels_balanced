// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"

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
//     accel_g[0] = (float)accel_x / 16384.0f;
//     accel_g[1] = (float)accel_y / 16384.0f;
//     accel_g[2] = (float)accel_z / 16384.0f;
    
//     return true;
// }

// // Define your Encoder Pins
// #define ENCODER_PIN_A 28 // Often labeled "CLK"
// #define ENCODER_PIN_B 27 // Often labeled "DT"

// // Use volatile for variables modified inside an interrupt!
// volatile int encoder_position = 0;
// volatile uint32_t last_interrupt_time = 0;

// // This is the Interrupt Service Routine (ISR)
// void encoder_callback(uint gpio, uint32_t events) {
//     // 1. Debounce: Physical metal contacts in the encoder "bounce" and can trigger 
//     // multiple rapid micro-clicks. We ignore any signals faster than 2 milliseconds.
//     uint32_t current_time = to_ms_since_boot(get_absolute_time());
//     if ((current_time - last_interrupt_time) < 2) {
//         return; 
//     }
    
//     // 2. Read direction: If Pin A triggered the interrupt, we check Pin B
//     if (gpio == ENCODER_PIN_A) {
//         if (gpio_get(ENCODER_PIN_B)) {
//             // Turning Counter-Clockwise
//             encoder_position--;
//         } else {
//             // Turning Clockwise
//             encoder_position++;
//         }
//     }
    
//     // Record the time of this valid click
//     last_interrupt_time = current_time;
// }

// int main() {
//     stdio_init_all();
    
    // i2c_init(I2C_PORT, 400 * 1000);
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    
    // sleep_ms(2000); 
    // printf("Starting Auto-Detect IMU Accelerometer Read...\n");
    // printf("--------------------------------------------\n");
    
    // if (!imu_init()) {
    //     printf("Halting. Check wiring.\n");
    //     while (1) { sleep_ms(1000); }
    // }
    
    // float accel[3];

//     // Initialize the GPIO pins
//     gpio_init(ENCODER_PIN_A);
//     gpio_init(ENCODER_PIN_B);
    
//     // Set them as inputs
//     gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
//     gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
    
//     // Enable pull-up resistors (keeps the signal HIGH when resting)
//     gpio_pull_up(ENCODER_PIN_A);
//     gpio_pull_up(ENCODER_PIN_B);

//     // Setup the interrupt on Pin A
//     // We only want it to trigger on a GPIO_IRQ_EDGE_RISE (signal going from 0 to 1)
//     gpio_set_irq_enabled_with_callback(
//         ENCODER_PIN_A, 
//         GPIO_IRQ_EDGE_RISE, 
//         true, 
//         &encoder_callback
//     );

//     // Track the last printed position to avoid spamming the console
//     int last_printed_position = 0;
    
//     while (true) {
//         if (imu_read_accel(accel)) {
//             printf("Accel X: %5.3f g | Y: %5.3f g | Z: %5.3f g\n", accel[0], accel[1], accel[2]);
//         } else {
//             printf("Read failed! I2C bus error.\n");
//         }
        
//         // Only print if the value has actually changed
//         if (encoder_position != last_printed_position) {
//             printf("Encoder Position: %d\n", encoder_position);
//             last_printed_position = encoder_position;
//         }

//         sleep_ms(100); 
//     }
    
//     return 0;
// }

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "controller.hpp"
#include <chrono>
#include <thread>

#define PERIOD_MS 6 // 6 ms period for 167 Hz frequency

// Waveshare Pico 10-DOF I2C pins
#define I2C_PORT i2c1
#define I2C_SDA 6 
#define I2C_SCL 7
#define IMU_ADDR 0x68

// --- ICM20948 Registers ---
#define ICM_WHOAMI      0x00 // Expected ID: 0xEA
#define ICM_PWR_MGMT_1  0x06
#define ICM_ACCEL_START 0x2D

// --- MPU9250 Registers ---
#define MPU_WHOAMI      0x75 // Expected ID: 0x71 (or 0x73 for MPU9255)
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_START 0x3B

// Global variable to hold the correct starting register for the accelerometer
uint8_t accel_reg_start = 0; 

bool imu_init() {
    uint8_t chip_id = 0;
    uint8_t reg;
    
    // 1. Try to read the ICM20948 Identity Register
    reg = ICM_WHOAMI;
    i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &reg, 1, true, 100000);
    i2c_read_timeout_us(I2C_PORT, IMU_ADDR, &chip_id, 1, false, 100000);
    
    if (chip_id == 0xEA) {
        printf("Hardware Detected: ICM20948 (Rev 1.0)\n");
        accel_reg_start = ICM_ACCEL_START;
        
        // Wake up ICM20948
        uint8_t buf[2] = {ICM_PWR_MGMT_1, 0x01};
        i2c_write_timeout_us(I2C_PORT, IMU_ADDR, buf, 2, false, 100000);
        sleep_ms(100);
        return true;
    }
    
    // 2. Try to read the MPU9250 Identity Register
    reg = MPU_WHOAMI;
    i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &reg, 1, true, 100000);
    i2c_read_timeout_us(I2C_PORT, IMU_ADDR, &chip_id, 1, false, 100000);
    
    if (chip_id == 0x71 || chip_id == 0x73) {
        printf("Hardware Detected: MPU9250/9255 (Rev 2.1)\n");
        accel_reg_start = MPU_ACCEL_START;
        
        // Wake up MPU9250
        uint8_t buf[2] = {MPU_PWR_MGMT_1, 0x01};
        i2c_write_timeout_us(I2C_PORT, IMU_ADDR, buf, 2, false, 100000);
        sleep_ms(100);
        return true;
    }

    printf("ERROR: Could not verify chip identity.\n");
    return false;
}

bool imu_read_accel(float *accel_g) {

    if (accel_reg_start == 0) return false; // Safety check
    
    uint8_t data[6];
    
    // Point to whichever register we dynamically assigned during init
    int ret = i2c_write_timeout_us(I2C_PORT, IMU_ADDR, &accel_reg_start, 1, true, 100000);
    if (ret < 0) return false;
    
    // Read 6 consecutive bytes 
    ret = i2c_read_timeout_us(I2C_PORT, IMU_ADDR, data, 6, false, 100000);
    if (ret < 0) return false;
    
    // Combine high and low bytes (16-bit, big-endian)
    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];
    
    // Convert to g (Both chips use a default scale factor of 16384 LSB/g)
    accel_g[0] = (float)accel_x * (M_PI_2)/ 16384.0f;
    accel_g[1] = (float)accel_y * (M_PI_2) / 16384.0f;
    accel_g[2] = (float)accel_z * (M_PI_2) / 16384.0f;
    
    return true;
}


// Define GPIO Pins
#define AIN1_PIN 14
#define AIN2_PIN 15
#define PWMA_PIN 2

#define BIN1_PIN 12
#define BIN2_PIN 13
#define PWMB_PIN 2

// Pin Definitions
#define PWM_PIN     2
#define ENCODER_A   28
#define ENCODER_B   27

void set_motors_voltage(double u, double v_max = 5.0) {
    // 1. Calculate absolute voltage and map to PWM level [0 to 6250]
    double abs_v = std::abs(u);
    uint16_t pwm_level = (uint16_t)((abs_v / v_max) * 5000.0); // Scale to 0-5000 for 20kHz PWM
    pwm_level = std::clamp(pwm_level, (uint16_t)0, (uint16_t)5000);

    // 2. Set direction logic based on sign of u
    if (u > 0.0) {
        // --- FORWARD (+V) ---
        gpio_put(AIN1_PIN, 1); gpio_put(AIN2_PIN, 0);
        gpio_put(BIN1_PIN, 1); gpio_put(BIN2_PIN, 0);
    } 
    else if (u < 0.0) {
        // --- REVERSE (-V) ---
        gpio_put(AIN1_PIN, 0); gpio_put(AIN2_PIN, 1);
        gpio_put(BIN1_PIN, 0); gpio_put(BIN2_PIN, 1);
    } 
    else {
        // --- STOP (0V) ---
        gpio_put(AIN1_PIN, 0); gpio_put(AIN2_PIN, 0);
        gpio_put(BIN1_PIN, 0); gpio_put(BIN2_PIN, 0);
        pwm_level = 0;
    }

    // 3. Apply PWM duty cycle
    pwm_set_gpio_level(PWMA_PIN, pwm_level);
    // pwm_set_gpio_level(PWMB_PIN, pwm_level);
}

// Volatile variable since it is modified inside an ISR
volatile int32_t encoder_count = 0;

// Interrupt Service Routine (ISR) for Encoder
void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_A) {
        // Check state of Channel B to determine direction
        if (gpio_get(ENCODER_B)) {
            encoder_count++;
        } else {
            encoder_count--;
        }
    }
}

int main() {
    stdio_init_all();

    // Initialize I2C for IMU
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(2000); 
    printf("Starting Auto-Detect IMU Accelerometer Read...\n");
    printf("--------------------------------------------\n");
    
    if (!imu_init()) {
        printf("Halting. Check wiring.\n");
        while (1) { sleep_ms(1000); }
    }
    
    float accel[3];

    // ---------------------------------------------------------
    // 1. PWM Setup (Motor Speed Control)
    // ---------------------------------------------------------
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint channel = pwm_gpio_to_channel(PWM_PIN);

    // Set PWM frequency to ~20kHz (outside human hearing range)
    // System clock (125MHz) / 20,000Hz = 6250 cycles
    uint32_t pwm_wrap = 5000;
    pwm_set_wrap(slice_num, pwm_wrap);
    
    // Start at 0% duty cycle (stopped)
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);

    // ---------------------------------------------------------
    // 2. Encoder GPIO Setup
    // ---------------------------------------------------------
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A); // Ensure stable signals

    gpio_init(ENCODER_B);
    gpio_set_dir(ENCODER_B, GPIO_IN);
    gpio_pull_up(ENCODER_B);

    // Enable falling/rising edge interrupt on Encoder A
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);

    // ---------------------------------------------------------
    // 3. Motor Direction GPIO Setup
    // ---------------------------------------------------------
    // Explicitly initialize AND set direction
    gpio_init(AIN1_PIN);
    gpio_set_dir(AIN1_PIN, GPIO_OUT);
    gpio_init(AIN2_PIN);
    gpio_set_dir(AIN2_PIN, GPIO_OUT);

    gpio_init(BIN1_PIN);
    gpio_set_dir(BIN1_PIN, GPIO_OUT);
    gpio_init(BIN2_PIN);
    gpio_set_dir(BIN2_PIN, GPIO_OUT);

    // ---------------------------------------------------------
    // 3. Main Control Loop
    // ---------------------------------------------------------
    uint32_t speed_step = pwm_wrap / 4; // 25% increments

     // Step 1: Drive motor at 75% speed
        std::cout << "Setting start speed to 78%" << std::endl;
        // printf("Setting speed to 78%%\n");
        pwm_set_chan_level(slice_num, channel, 4000); // 75% of 6250
        sleep_ms(2000);
        // printf("Current Encoder Count: %ld\n", encoder_count);

    // int i = 4937.5 ; // 78% of 6250
    float i = 4000; // 75% of 6250

    float onePresent = 62.5;

    AngleController angleController;

    absolute_time_t next = get_absolute_time();

    while (true) {
        // // Step 1: Drive motor at 75% speed
        // // printf("Setting speed to 79%%\n");
        // std::cout << "\nSetting speed to " << (i / 5000.0) * 100 << "%\n" << std::endl;
        // pwm_set_chan_level(slice_num, channel, i); // 75% of 6250
        // sleep_ms(1000);
        // // // printf("Current Encoder Count: %ld\n", encoder_count);
        // // std::cout << "Current Encoder Count: " << encoder_count << std::endl;
        
        // if (i >= 5000) {
        //     i = 4000; // Cap at 75%
        // } else {
        //     i += onePresent/2.0; // Increment by 1%
        // }
    
        // schedule next execution time
        next = delayed_by_us(next, PERIOD_MS * 1000);
        
        if (imu_read_accel(accel)) {

            auto start = std::chrono::high_resolution_clock::now();

            static double angle_filtered = 0.0;
            double raw_angle = atan2(accel[0], accel[2]);
            std::cout << "Raw Angle: " << raw_angle << " rad | " << (raw_angle * 180.0 / M_PI) << " deg" << std::endl;
            // angle_filtered = 0.8 * angle_filtered + 0.2 * raw_angle; // Smooth raw accelerometer noise

            double u = angleController.Controller(raw_angle, PERIOD_MS/1000.0); // dt = 0.006s for 167Hz

            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            // std::cout << "Execution time: " << elapsed_ms << " ms" << std::endl; // Ensure elapsed_ms is consistently <= 10.0 ms!

            // printf("Accel X: %5.3f g | Y: %5.3f g | Z: %5.3f g\n", accel[0], accel[1], accel[2]); // Assuming accel[0] is the angle in radians

            
            // std::cout << "Angle: " << angle_rad << "\t rad | Control Input (Voltage): " << u << "\tV" << std::endl;

            // pwm_set_chan_level(slice_num, channel, u * 1000.0); // max = 5000
            set_motors_voltage(u, angleController.v_max);
        } else {
            printf("Read failed! I2C bus error.\n");
        }
        
        // sleep until the exact next time

        sleep_until(next);
        // sleep_ms(100);

    }

    return 0;
}

