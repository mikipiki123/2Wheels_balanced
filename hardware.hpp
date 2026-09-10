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


class IMU_sensor {

    public:

    float angle_x = 0.0f; // Estimated angle from sensor fusion
    float angular_velocity_x = 0.0f; // Estimated angular velocity from sensor fusion

    IMU_sensor();
    bool read_sensor_fusion_x(uint64_t delta_us);
};