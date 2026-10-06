

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include <chrono>
#include <thread>

// #include "pico/cyw43_arch.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Local
#include "controller.hpp"
#include "hardware.hpp"

#define PERIOD_MS 10 // 10 ms period for 100 Hz frequency.


// Task Handles
TaskHandle_t xControlLoopTaskHandle = NULL;

typedef struct {
    IMU_sensor* imu;
    FullController* fullController;
    MotorController* motorController;
} ControlLoopParams;

void Control_Loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Exactly 10 ms (100 Hz)

    float x_ref = 0.0f; // todo // Local copy of the target position

    while (1) {

        if (true) { // imu_read_accel(accel)) {

            ControlLoopParams* params = (ControlLoopParams*)pvParameters;
            params->imu->read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
            // imu.angle_x -= theta_bias; // Remove bias from calibration

            double u = params->fullController->Controller(params->imu->angle_x, params->imu->angular_velocity_x, PERIOD_MS/1000.0, x_ref); // dt = 0.005 s (5 ms) = 200 Hz, target_x = 0.1 (m)

            // double u = angleController.Controller(imu.angle_x, imu.angular_velocity_x, PERIOD_MS/1000.0); // dt = 0.01 s (10 ms) = 100 Hz
        
            // fullController.w = fullController.w*0.98 + u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command, with Leaky Integrator - 0.98
            // without leaky integrator:
            params->fullController->w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command
            // angleController.w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command

            // printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            //     fullController.x, 
            //     fullController.x_dot, 
            //     imu.angle_x, 
            //     imu.angular_velocity_x, 
            //     fullController.integral_action_pos, 
            //     fullController.w); // for graph RTplot.py


            params->motorController->set_motor_velocity(params->fullController->w, 2); // Apply control input to motors
            params->motorController->set_motor_velocity(-params->fullController->w, 1); // Apply control input to motors
            

        } else {
            printf("Read failed! I2C bus error.\n");
        }



        

        // F. Wait until precisely 10 ms has elapsed since last cycle execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int main() {
    stdio_init_all();

    IMU_sensor imu; // Create an instance of the IMU_sensor class

    printf("Starting IMU read...\n");
    printf("--------------------------------------------\n");

    MotorController motorController; // Create an instance of the MotorController class

   
    printf("Initializing Motor Controller...\n");
    printf("--------------------------------------------\n");

    sleep_ms(5000); // Wait for 5 seconds to allow the IMU to stabilize

    
    // AngleController angleController;
    FullController fullController; // Create an instance of the FullController class

    ControlLoopParams params = {&imu, &fullController, &motorController}; // Create a struct to hold the parameters for the control loop task

    // absolute_time_t next = get_absolute_time();

    // Create Balance Task (Priority 3 - Highest)
    xTaskCreate(
        Control_Loop,
        "Control_Loop",
        1024,               // Stack size (words)
        &params, // Pass parameters to the task
        configMAX_PRIORITIES - 1, // Highest priority
        &xControlLoopTaskHandle
    );

    // Pin BalanceTask to Core 1 using RP2040 FreeRTOS SMP affinity mask
    vTaskCoreAffinitySet(xControlLoopTaskHandle, (1 << 1)); // Bit 1 = Core 1

    // Start the FreeRTOS Scheduler
    vTaskStartScheduler();




    while (true) {
        

        // // schedule next execution time
        // next = delayed_by_us(next, PERIOD_MS * 1000);

        
        // if (true) { // imu_read_accel(accel)) {

        //     auto start = std::chrono::high_resolution_clock::now();

        //     imu.read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU
        //     // imu.angle_x -= theta_bias; // Remove bias from calibration

        //     double u = fullController.Controller(imu.angle_x, imu.angular_velocity_x, PERIOD_MS/1000.0, 0.1); // dt = 0.005 s (5 ms) = 200 Hz, target_x = 0.1 (m)

        //     // double u = angleController.Controller(imu.angle_x, imu.angular_velocity_x, PERIOD_MS/1000.0); // dt = 0.01 s (10 ms) = 100 Hz
        
        //     // fullController.w = fullController.w*0.98 + u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command, with Leaky Integrator - 0.98
        //     // without leaky integrator:
        //     fullController.w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command
        //     // angleController.w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command

        //     printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        //         fullController.x, 
        //         fullController.x_dot, 
        //         imu.angle_x, 
        //         imu.angular_velocity_x, 
        //         fullController.integral_action_pos, 
        //         fullController.w); // for graph RTplot.py

        //     // printf("%.4f,%.4f,%.4f\n",
        //     //     imu.angle_x, 
        //     //     imu.angular_velocity_x, 
        //     //     angleController.w); // for graph RTplot.py


            
        //     motorController.set_motor_velocity(fullController.w, 2); // Apply control input to motors
        //     motorController.set_motor_velocity(-fullController.w, 1); // Apply control input to motors
            

        //     auto end = std::chrono::high_resolution_clock::now();
        //     double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        //     std::cout << "Execution time: " << elapsed_ms << " ms" << std::endl; // Ensure elapsed_ms is consistently <= 10.0 ms!

        // } else {
        //     printf("Read failed! I2C bus error.\n");
        // }
        
        // sleep until the exact next time
        // sleep_until(next);


    }

    return 0;
}
