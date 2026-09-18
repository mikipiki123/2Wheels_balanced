#include "hardware.hpp"

void IMU_sensor::calibrate_gyro() {
    uint8_t reg = 0x45; // GYRO_YOUT_H register (buffer[10-11] in your burst read)
    uint8_t buffer[2];
    int32_t sum = 0;

    for (uint16_t i = 0; i < 500; i++) {
        // Read 2 bytes starting at 0x45 (Gyro Y High/Low)
        i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, IMU_ADDR, buffer, 2, false);

        int16_t raw_gyro_y = (int16_t)((buffer[0] << 8) | buffer[1]);
        sum += raw_gyro_y;

        sleep_ms(2); // 500 samples @ 2ms = 1 second calibration
    }

    // Compute average raw LSB offset
    this->gyro_bias_y = (float)sum / (float)500.0f;
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
    float gyro_pitch_rad = (float)(raw_gyro_pitch - gyro_bias_y) / GYRO_SCALE_RAD;

    // Fast 1st-Order Low-Pass Filter at 8 Hz (100 Hz sample rate)
    static float gyro_filtered = 0.0f;
    float alpha_gyro = 0.335f;
    gyro_filtered = alpha_gyro * gyro_pitch_rad + (1.0f - alpha_gyro) * gyro_filtered;

    // Calculate raw angle using X and Z axes with corrected mounting orientation
    float accel_angle = -std::atan2(ax, az);

    // Apply complementary filter
    static float angle = 0.0f;
    float dt = (float)delta_us / 1000000.0f;

    angle = ALPHA * (angle + gyro_filtered * dt) + (1.0f - ALPHA) * accel_angle;

    this->angle_x = angle;
    this->angular_velocity_x = gyro_filtered;

    return true;
}


MotorController::MotorController() {
    // Initialize the motor controller
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
}

void MotorController::set_motor_velocity(float rad_sec) {
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
