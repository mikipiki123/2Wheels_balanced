#include "hardware.hpp"


IMU_sensor::IMU_sensor() { //initialize the IMU sensor object

    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz fast mode
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Reset/Wake-up register (PWR_MGMT_1 register 0x6B = 0x00)
    uint8_t wake_cmd[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, IMU_ADDR, wake_cmd, 2, false);
    
}

bool IMU_sensor::read_sensor_fusion_x(uint64_t delta_us) {
    uint8_t reg = 0x3B; // ACCEL_XOUT_H register start
    uint8_t buffer[14];

    // Burst read 14 registers (Accel X/Y/Z, Temp, Gyro X/Y/Z)
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, buffer, 14, false);

    // Parse 16-bit signed integers from buffer
    // accel[0] = Accel X (buffer 0-1), accel[2] = Accel Z (buffer 4-5)
    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Pitch rate in X-Z plane aligns with Gyro Y (buffer 10-11).
    // Swap to Gyro X (buffer 8-9) if your sensor orientation requires it.
    int16_t raw_gyro_pitch = (int16_t)((buffer[10] << 8) | buffer[11]); 

    // Convert to physical units
    float ax = (float)raw_ax / ACCEL_SCALE;
    float az = (float)raw_az / ACCEL_SCALE;
    float gyro_pitch_rad = (float)raw_gyro_pitch / GYRO_SCALE_RAD;

    // Calculate raw angle using X and Z axes with corrected mounting orientation
    float accel_angle = -std::atan2(ax, az);

    // Apply complementary filter
    static float angle = 0.0f;
    float dt = (float)delta_us / 1000000.0f;

    angle = ALPHA * (angle + gyro_pitch_rad * dt) + (1.0f - ALPHA) * accel_angle;

    this->angle_x = angle;
    this->angular_velocity_x = gyro_pitch_rad;

    return true;
}
