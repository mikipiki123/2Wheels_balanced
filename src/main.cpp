

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

#define WIFI_SSID "WIFI"
#define WIFI_PASSWORD "PASSWORD"
#define SERVER_PORT    5000
#define HOSTNAME    "robot" // Will resolve to robot.local

#define UDP_TELEMETRY_PORT 5001

#define PERIOD_MS 5 // 5 ms period for 200 Hz frequency.


// Task Handles
TaskHandle_t xControlLoopTaskHandle = NULL;
TaskHandle_t xCommandRecieveTaskHandle = NULL;
TaskHandle_t xTelemetryTaskHandle = NULL;

SemaphoreHandle_t xConfigMutex = NULL;

typedef struct {
    IMU_sensor* imu;
    FullController* fullController;
    MotorController* motorController;
} ControlLoopParams;

// Packed telemetry struct (24 bytes total)
#pragma pack(push, 1)
typedef struct { 
    float x;           // Wheel position (m)
    float x_dot;       // Wheel velocity (m/s)
    float theta;       // Tilt angle (rad)
    float theta_dot;   // Angular velocity (rad/s)
    float x_integral;  // Position error integral
    float w_motor;     // Control output (rad/s^2)
} TelemetryPacket;
#pragma pack(pop)

double x_reference = 0.0f; // Global target position for the robot

void vTelemetryTask(void *pvParameters) {

ControlLoopParams* params = (ControlLoopParams*)pvParameters;

    // 1. Create UDP Socket
    int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        printf("[TELEM] Socket creation failed!\n");
        vTaskDelete(NULL);
    }

    // 2. Enable Broadcast Option
    int broadcast_enable = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    // 3. Configure Broadcast Destination Address
    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_TELEMETRY_PORT);
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); // 255.255.255.255

    printf("[TELEM] UDP Broadcaster starting on port %d...\n", UDP_TELEMETRY_PORT);

    TelemetryPacket packet;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz (50 ms)

        // Read sensor values safely
        packet.x          = (float)params->fullController->x;
        packet.x_dot      = (float)params->fullController->x_dot;
        packet.theta      = (float)params->imu->angle_x;
        packet.theta_dot  = (float)params->imu->angular_velocity_x;
        packet.x_integral = (float)params->fullController->integral_action_pos;
        packet.w_motor    = (float)params->fullController->w;

        // Send binary telemetry struct
        int bytes_sent = sendto(
            udp_sock, 
            &packet, 
            sizeof(TelemetryPacket), 
            0, 
            (struct sockaddr *)&broadcast_addr, 
            sizeof(broadcast_addr)
        );

        if (bytes_sent < 0) {
            printf("[TELEM] sendto failed!\n");
        }
    }
}

void Control_Loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_MS); // Exactly 5 ms (200 Hz)


    // // Profiling variables
    // uint64_t total_exec_time_us = 0;
    // uint64_t max_exec_time_us = 0;
    // uint64_t min_exec_time_us = UINT64_MAX;
    // uint32_t sample_count = 0;


    double x_ref = 0.0; // Local copy of the target position

    while (1) {

        

        if (true) {

            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // // 1. Record start timestamp
            // absolute_time_t t_start = get_absolute_time();


            //------------- control loop code here -------------//

            if (xSemaphoreTake(xConfigMutex, 0) == pdTRUE) {
                // Read the global target position for the robot
                x_ref = x_reference;
                xSemaphoreGive(xConfigMutex);
            }

            ControlLoopParams* params = (ControlLoopParams*)pvParameters;
            params->imu->read_sensor_fusion_x(PERIOD_MS * 1000); // Read sensor fusion data from IMU

            double u = params->fullController->Controller(params->imu->angle_x, params->imu->angular_velocity_x, PERIOD_MS/1000.0, x_ref); // dt = 0.005 s (5 ms) = 200 Hz, target_x = 0.1 (m)

            params->fullController->w += u*(PERIOD_MS/1000.0); // Integrate control input to get angular velocity command



            params->motorController->set_motor_velocity(params->fullController->w, 2); // Apply control input to motors
            params->motorController->set_motor_velocity(-params->fullController->w, 1); // Apply control input to motors

        //------------- end of control loop code -------------//

        //     // 2. Record end timestamp
        // absolute_time_t t_end = get_absolute_time();

        // // 3. Compute execution time for this single iteration
        // int64_t exec_duration_us = absolute_time_diff_us(t_start, t_end);

        // // Update statistics
        // total_exec_time_us += exec_duration_us;
        // if ((uint64_t)exec_duration_us > max_exec_time_us) {
        //     max_exec_time_us = exec_duration_us;
        // }
        // if ((uint64_t)exec_duration_us < min_exec_time_us) {
        //     min_exec_time_us = exec_duration_us;
        // }
        // sample_count++;

        // // 4. Print aggregated metrics every 1000 ms (200 samples)
        // if (sample_count >= 100) {
        //     uint64_t avg_exec_us = total_exec_time_us / sample_count;
            
        //     // Calculate Core 1 CPU Utilization Percentage
        //     float core1_cpu_load = ((float)avg_exec_us / (float)10000) * 100.0f;

        //     printf("[CORE 1 PROFILE] Loop Target: %d us | Exec Avg: %llu us | Min: %llu us | Max: %llu us | Core 1 CPU Load: %.2f%%\n",
        //            10000,
        //            avg_exec_us,
        //            min_exec_time_us,
        //            max_exec_time_us,
        //            core1_cpu_load);

        //     // Reset accumulation metrics
        //     total_exec_time_us = 0;
        //     max_exec_time_us = 0;
        //     min_exec_time_us = UINT64_MAX;
        //     sample_count = 0;
        // }
            

        } else {
            printf("Read failed! I2C bus error.\n");
        }



        

        // // F. Wait until precisely 10 ms has elapsed since last cycle execution
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

    xTaskCreate(
        vTelemetryTask,
        "TelemetryTask",
        2048,
        pvParameters,
        2,
        &xTelemetryTaskHandle
    );
    vTaskCoreAffinitySet(xTelemetryTaskHandle, (1 << 0)); // Pin TelemetryTask to Core 0

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
        &params,
        2,
        &xCommandRecieveTaskHandle
    );
    vTaskCoreAffinitySet(xCommandRecieveTaskHandle, (1 << 0)); // Pin NetworkTask to Core 0



    // Start the FreeRTOS Scheduler
    vTaskStartScheduler();

    return 0;
}
