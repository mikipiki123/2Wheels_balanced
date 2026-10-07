

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include <chrono>
#include <thread>

#include "pico/cyw43_arch.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Local
#include "controller.hpp"
#include "hardware.hpp"

// lwIP Sockets API for your communication channels
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/apps/mdns.h"

#define WIFI_SSID "bernerm"
#define WIFI_PASSWORD "mikipiki12"
#define SERVER_PORT    5000
#define HOSTNAME    "robot" // Will resolve to robot.local

#define PERIOD_MS 10 // 10 ms period for 100 Hz frequency.


// Task Handles
TaskHandle_t xControlLoopTaskHandle = NULL;
TaskHandle_t xCommandRecieveTaskHandle = NULL;

SemaphoreHandle_t xConfigMutex = NULL;

typedef struct {
    IMU_sensor* imu;
    FullController* fullController;
    MotorController* motorController;
} ControlLoopParams;

double x_reference = 0.0f; // Global target position for the robot


void Control_Loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Exactly 10 ms (100 Hz)

    double x_ref = 0.0; // Local copy of the target position

    while (1) {

        if (true) { // imu_read_accel(accel)) {

            if (xSemaphoreTake(xConfigMutex, 0) == pdTRUE) {
                // Read the global target position for the robot
                x_ref = x_reference;
                xSemaphoreGive(xConfigMutex);
            }

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

void vCommandRecieveTask(void *pvParameters) {
    // 2. Initialize Wi-Fi hardware
    if (cyw43_arch_init()) {
        printf("[NET] CYW43 Init failed!\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    printf("[NET] Connecting to Wi-Fi: %s...\n", WIFI_SSID);

    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000) != 0) {
        printf("[NET] Wi-Fi connection failed. Retrying in 2s...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // 3. Wait for valid DHCP IP address assignment
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    printf("[NET] Wi-Fi Associated. Waiting for IP address via DHCP...\n");

    while (ip4_addr_isany(netif_ip4_addr(netif)) || !netif_is_up(netif)) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    printf("\n=========================================\n");
    printf("[NET] Connected! Robot IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
    printf("[NET] Reachable at: %s.local\n", HOSTNAME);
    printf("=========================================\n\n");

    // 4. Safe mDNS Initialization after IP is valid
    mdns_resp_init();
    mdns_resp_add_netif(netif, HOSTNAME);

    // 5. Create Listening TCP Socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        printf("[NET] Failed to create socket.\n");
        vTaskDelete(NULL);
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("[NET] Socket bind failed.\n");
        lwip_close(listen_sock);
        vTaskDelete(NULL);
    }

    if (listen(listen_sock, 1) < 0) {
        printf("[NET] Socket listen failed.\n");
        lwip_close(listen_sock);
        vTaskDelete(NULL);
    }

    printf("[NET] TCP Server listening on port %d...\n", SERVER_PORT);

    // 6. Server Accept Loop
    for (;;) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        // Blocks safely on Core 0
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("[NET] PC Connected from %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        char rx_buf[128];
        for (;;) {
            int bytes_received = recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0);
            if (bytes_received <= 0) {
                printf("[NET] PC Client disconnected.\n");
                break;
            }

            rx_buf[bytes_received] = '\0';

            

            std::string rx = std::string(rx_buf);


            // Strip trailing newlines (\n, \r) or spaces sent by TCP clients
            while (!rx.empty() && (rx.back() == '\r' || rx.back() == '\n' || rx.back() == ' ')) {
                rx.pop_back();
            }

            printf("[CMD] Received: %s", rx.c_str());

            if (rx == "sleep"){
                send(client_sock, "Robot go to sleep\n", 17, 0);

            } else if (rx.rfind("X=", 0) == 0 || rx.rfind("x=", 0) == 0) { // Check if the command starts with "X="
                std::string value_str = rx.substr(2); // Extract the value after "X="
                double new_x_ref = std::stod(value_str); // Convert to double
                if (xSemaphoreTake(xConfigMutex, portMAX_DELAY) == pdTRUE) {
                    x_reference = new_x_ref; // Update the global target position
                    xSemaphoreGive(xConfigMutex);

                    std::string ack = "Target position updated to " + std::to_string(new_x_ref) + "\n";
                    send(client_sock, ack.c_str(), ack.length(), 0);
                }
            } else {
                send(client_sock, "What?\n", 6, 0);
            }


        }

        lwip_close(client_sock);
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

    xConfigMutex = xSemaphoreCreateMutex(); // Create a mutex for shared configuration access

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

    xTaskCreate(
        vCommandRecieveTask,
        "CommandRecieveTask",
        2048,
        NULL,
        2,
        &xCommandRecieveTaskHandle
    );
    vTaskCoreAffinitySet(xCommandRecieveTaskHandle, (1 << 0)); // Pin NetworkTask to Core 0


    // Start the FreeRTOS Scheduler
    vTaskStartScheduler();

    return 0;
}
