#include "hardware.hpp"

#include <cmath>

#define INT_PIN_CFG     0x37
#define MAG_ADDR        0x0C  // AK8963 address (Change to 0x0D for QMC5883L or 0x1E for HMC5883L)
#define MAG_CNTL1       0x0A

void IMU_sensor::init_magnetometer() {
    // 1. Enable I2C Bypass Mode on MPU6050/9250 so Pico can talk to magnetometer directly
    uint8_t bypass_cmd[] = {INT_PIN_CFG, 0x02};
    i2c_write_blocking(I2C_PORT, IMU_ADDR, bypass_cmd, 2, false);
    sleep_ms(10);

    // 2. Configure AK8963 Magnetometer to Continuous Measurement Mode 2 (100 Hz, 16-bit)
    uint8_t mag_cmd[] = {MAG_CNTL1, 0x16}; 
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cmd, 2, false);
    sleep_ms(10);
}

void IMU_sensor::calibrate_magnetometer() {
    printf("[IMU] Calibrating Magnetometer... Rotate robot 360 degrees on flat surface!\n");

    int16_t mag_min[3] = {32767, 32767, 32767};
    int16_t mag_max[3] = {-32768, -32768, -32768};

    // Sample magnetometer for 10 seconds while spinning robot manually
    for (int i = 0; i < 500; i++) {
        uint8_t reg = 0x03; // ST1/HXL register start
        uint8_t buf[7];
        
        i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 7, false);

        int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);

        mag_min[0] = std::min(mag_min[0], mx); mag_max[0] = std::max(mag_max[0], mx);
        mag_min[1] = std::min(mag_min[1], my); mag_max[1] = std::max(mag_max[1], my);
        mag_min[2] = std::min(mag_min[2], mz); mag_max[2] = std::max(mag_max[2], mz);

        sleep_ms(20);
    }

    // Compute Hard-Iron Offset (Center of bounding sphere)
    this->mag_bias_x = (float)(mag_max[0] + mag_min[0]) / 2.0f;
    this->mag_bias_y = (float)(mag_max[1] + mag_min[1]) / 2.0f;
    this->mag_bias_z = (float)(mag_max[2] + mag_min[2]) / 2.0f;

    printf("[IMU] Mag Calibrated! Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", mag_bias_x, mag_bias_y, mag_bias_z);
}

void IMU_sensor::calibrate_gyro() {
    uint8_t reg = 0x43; // GYRO_XOUT_H register start[cite: 5]
    uint8_t buffer[6];
    int32_t sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < 500; i++) {
        i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);[cite: 5]
        i2c_read_blocking(I2C_PORT, IMU_ADDR, buffer, 6, false);[cite: 5]

        int16_t raw_gyro_y = (int16_t)((buffer[2] << 8) | buffer[3]);[cite: 5]
        int16_t raw_gyro_z = (int16_t)((buffer[4] << 8) | buffer[5]);

        sum_y += raw_gyro_y;[cite: 5]
        sum_z += raw_gyro_z;

        sleep_ms(2);[cite: 5]
    }

    this->gyro_bias_y = (float)sum_y / 500.0f;[cite: 5]
    this->gyro_bias_z = (float)sum_z / 500.0f;
}


IMU_sensor::IMU_sensor() { //initialize the IMU sensor object

    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz fast mode
    // gpio_init(I2C_SDA);
    // gpio_init(I2C_SCL);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Reset/Wake-up register (PWR_MGMT_1 register 0x6B = 0x00)
    uint8_t wake_cmd[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, IMU_ADDR, wake_cmd, 2, false);
    
    calibrate_gyro();
}

IMU_sensor::IMU_sensor() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    uint8_t wake_cmd[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, IMU_ADDR, wake_cmd, 2, false);

    calibrate_gyro();
    init_magnetometer();
    calibrate_magnetometer();
}

// bool IMU_sensor::read_sensor_fusion_x(uint64_t delta_us) {
//     uint8_t reg = 0x3B; // ACCEL_XOUT_H register start
//     uint8_t buffer[14];

//     // Burst read 14 registers (Accel X/Y/Z, Temp, Gyro X/Y/Z)
//     i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);
//     i2c_read_blocking(I2C_PORT, IMU_ADDR, buffer, 14, false);

//     // Parse 16-bit signed integers from buffer
//     // accel[0] = Accel X (buffer 0-1), accel[2] = Accel Z (buffer 4-5)
//     int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
//     int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
//     // Pitch rate in X-Z plane aligns with Gyro Y (buffer 10-11).
//     // Swap to Gyro X (buffer 8-9) if your sensor orientation requires it.
//     int16_t raw_gyro_pitch = (int16_t)((buffer[10] << 8) | buffer[11]); 

//     // Convert to physical units
//     float ax = (float)raw_ax / ACCEL_SCALE;
//     float az = (float)raw_az / ACCEL_SCALE;
//     float gyro_pitch_rad = (float)(raw_gyro_pitch - gyro_bias_y) / GYRO_SCALE_RAD;

//     // Fast 1st-Order Low-Pass Filter at 8 Hz (100 Hz sample rate)
//     static float gyro_filtered = 0.0f;
//     float alpha_gyro = 0.335f;
//     gyro_filtered = alpha_gyro * gyro_pitch_rad + (1.0f - alpha_gyro) * gyro_filtered;

//     // Calculate raw angle using X and Z axes with corrected mounting orientation
//     float accel_angle = -std::atan2(ax, az);

//     // Apply complementary filter
//     static float angle = 0.0f;
//     float dt = (float)delta_us / 1000000.0f;

//     angle = ALPHA * (angle + gyro_filtered * dt) + (1.0f - ALPHA) * accel_angle;

//     this->angle_x = angle;
//     this->angular_velocity_x = gyro_filtered;

//     return true;
// }

bool IMU_sensor::read_sensor_fusion_x(uint64_t delta_us) {
    uint8_t reg = 0x3B; 
    uint8_t buffer[14];

    // 1. Read 14 bytes from MPU6050/9250[cite: 5]
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);[cite: 5]
    i2c_read_blocking(I2C_PORT, IMU_ADDR, buffer, 14, false);[cite: 5]

    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);[cite: 5]
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);[cite: 5]
    
    int16_t raw_gyro_pitch = (int16_t)((buffer[10] << 8) | buffer[11]);[cite: 5]
    int16_t raw_gyro_yaw   = (int16_t)((buffer[12] << 8) | buffer[13]); 

    float ax = (float)raw_ax / ACCEL_SCALE;[cite: 5]
    float ay = (float)raw_ay / ACCEL_SCALE;
    float az = (float)raw_az / ACCEL_SCALE;[cite: 5]
    
    float gyro_pitch_rad = (float)(raw_gyro_pitch - gyro_bias_y) / GYRO_SCALE_RAD;[cite: 5]
    float gyro_yaw_rad   = (float)(raw_gyro_yaw - gyro_bias_z) / GYRO_SCALE_RAD;

    // --- PITCH ESTIMATION (Existing Code) ---
    static float gyro_filtered = 0.0f;[cite: 5]
    float alpha_gyro = 0.335f;[cite: 5]
    gyro_filtered = alpha_gyro * gyro_pitch_rad + (1.0f - alpha_gyro) * gyro_filtered;[cite: 5]

    float accel_pitch = -std::atan2(ax, az);[cite: 5]
    float dt = (float)delta_us / 1000000.0f;[cite: 5]

    static float pitch = 0.0f;
    pitch = ALPHA * (pitch + gyro_filtered * dt) + (1.0f - ALPHA) * accel_pitch;[cite: 5]

    this->angle_x = pitch;[cite: 5]
    this->angular_velocity_x = gyro_filtered;[cite: 5]

    // --- YAW ESTIMATION WITH TILT-COMPENSATED MAGNETOMETER ---
    uint8_t mag_reg = 0x03;
    uint8_t mag_buf[7];
    
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &mag_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, mag_buf, 7, false);

    // Apply Hard-Iron Offsets
    float mx = (float)((int16_t)((mag_buf[1] << 8) | mag_buf[0])) - mag_bias_x;
    float my = (float)((int16_t)((mag_buf[3] << 8) | mag_buf[2])) - mag_bias_y;
    float mz = (float)((int16_t)((mag_buf[5] << 8) | mag_buf[0])) - mag_bias_z;

    // Roll angle (needed for tilt compensation)
    float roll = std::atan2(ay, az); 

    // Tilt Compensation Equations
    float Mx = mx * std::cos(pitch) + my * std::sin(roll) * std::sin(pitch) + mz * std::cos(roll) * std::sin(pitch);
    float My = my * std::cos(roll) - mz * std::sin(roll);

    // Raw Magnetic Yaw Angle
    float raw_mag_yaw = std::atan2(-My, Mx);

    // Complementary Filter for Yaw
    static float yaw = 0.0f;
    const float ALPHA_YAW = 0.98f; // Trust gyro 98%, Mag 2%

    yaw = ALPHA_YAW * (yaw + gyro_yaw_rad * dt) + (1.0f - ALPHA_YAW) * raw_mag_yaw;

    this->angle_z = yaw;
    this->angular_velocity_z = gyro_yaw_rad;

    return true;
}


MotorController::MotorController() {
    // Initialize the motor controller
    gpio_init(STEP_PIN1);
    gpio_set_dir(STEP_PIN1, GPIO_OUT);

    gpio_init(DIR_PIN1);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);

    gpio_init(STEP_PIN2);
    gpio_set_dir(STEP_PIN2, GPIO_OUT);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);

    // gpio_init(EN_PIN);
    // gpio_set_dir(EN_PIN, GPIO_OUT);
    // // Enable TMC2209 (Active LOW: LOW = Driver ON, HIGH = Motors Free-Wheeling)
    // gpio_put(EN_PIN, 0);
    // Setup STEP Pins as PWM function outputs
    gpio_set_function(STEP_PIN2, GPIO_FUNC_PWM);
    gpio_set_function(STEP_PIN1, GPIO_FUNC_PWM);
}

void MotorController::set_motor_velocity(float rad_sec, int motor_id) {

    uint STEP_PIN, DIR_PIN;
    if (motor_id == 1) {
        STEP_PIN = STEP_PIN1;
        DIR_PIN = DIR_PIN1;
    } else if (motor_id == 2) {
        STEP_PIN = STEP_PIN2;
        DIR_PIN = DIR_PIN2;
    }

    uint slice_num = pwm_gpio_to_slice_num(STEP_PIN);
    uint chan = pwm_gpio_to_channel(STEP_PIN);

    // Stop output if requested speed is near zero
    if (std::abs(rad_sec) < 0.001f || std::abs(rad_sec) > MOTORS_RAD_S_MAX) { // Safety limit to prevent excessive speed
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
