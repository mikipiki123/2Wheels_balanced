#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include <cmath>
#include <algorithm>


// Waveshare Pico 10-DOF I2C pins
#define I2C_PORT i2c1
#define I2C_SDA 6 
#define I2C_SCL 7
#define IMU_ADDR 0x68

// Sensor sensitivity factors (±2g range, ±250 deg/s range)
constexpr float ACCEL_SCALE = 16384.0f;                     // LSB / g
constexpr float GYRO_SCALE_RAD = 131.0f * (180.0f / M_PI);   // LSB / (rad/s)
constexpr float ALPHA = 0.98f;                               // Complementary filter weight
#define MOTORS_RAD_S_MAX 100.0f // Maximum motor speed in rad/s


class IMU_sensor {

    private:
    double gyro_bias_y = 0.0f; // Gyroscope bias for Y-axis (pitch rate)
    void calibrate_gyro();


    public:

    float angle_x = 0.0f; // Estimated angle from sensor fusion
    float angular_velocity_x = 0.0f; // Estimated angular velocity from sensor fusion

    IMU_sensor();
    bool read_sensor_fusion_x(uint64_t delta_us);
};



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

class MotorController {

    public:
    MotorController();
    void set_motor_velocity(float rad_sec);


};