/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 1A of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		task1A.c
*  Created:		    10/10/2025
*  Last Modified:	15/10/2025
*  Author:		    e-Yantra Team
*  Team ID:		    [ CB_2202 ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
*/


#include "coppeliasim_client.h"  // Include our header

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations (these will move to header gradually)
// ----------------------
void* control_loop(void* arg);          // Only control_loop remains

/**
 * @brief Establishes connection to the CoppeliaSim server
 * @param c Pointer to SocketClient structure
 * @param ip IP address of the server (typically "127.0.0.1" for localhost)
 * @param port Port number of the server (typically 50002)
 * @return 1 if connection successful, 0 if failed
 */

int connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        printf("Socket creation failed\n");
        return 0;
    }

    // Setup server address structure
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    // Attempt to connect to server
    if (connect(c->sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

    // Start the receive thread to handle incoming sensor data
#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}
/*===============================================================================================*/
/**
 * @brief Main control loop thread for robot behavior
 * @param arg Pointer to SocketClient structure (cast from void*)
 * @return NULL when thread exits
 * 
 * This is where you should implement your robot's control logic.
 * The function runs continuously while the client is connected.
 * 
 * Available functions for control:
 * - set_motor(c, left_speed, right_speed): Control motor speeds
 * - Access sensor data via: c->sensor_values[index] and c->sensor_count
 */

float compute_line_position(SocketClient * c){
    int weight[5]= {-2,-1,0,1,2};
    float numerator = 0.0f;
    float denominator = 0.0f;
    for(int i = 0; i<5 ;i++){
        float value = c->sensor_values[i];  // keep raw float
        numerator += value * weight[i];
        denominator += value;
    }
    if (denominator == 0){
        return 0.0f;  // no line detected
    }
    return numerator / denominator;
}

void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    float kp = 0.95f;
    float kd = 0.6f;
    float ki = 0.00002f;

    float last_error = 0.0f;
    float integral  = 0.0f;
    float base_speed = 3.0f;

    while (c->running) {
        // Sharp turns
        if ((c->sensor_values[0] > 0.4595f) && ( c->sensor_values[4]<0.4595f)) {
            
            set_motor(c, -1.5f, 2.0f);
            //printf("[TURN] Hard LEFT | L=%.2f R=%.2f\n", left, right);  // hard left
        }
        else if ((c->sensor_values[0] < 0.4595f ) && ( c->sensor_values[4]>0.4595f)) {
            
            set_motor(c, 2.0f, -1.5f); 
            //printf("[TURN] Hard RIGHT | L=%.2f R=%.2f\n", left, right); // hard right
        }
        else {
            // PID logic
            float position  = compute_line_position(c);
            float error = -position;  // target = 0
            integral += error;
            float derivative = error - last_error;

            float correction  = kp*error + kd*derivative + ki*integral;

            float left_speed = base_speed - correction;
            float right_speed = base_speed + correction;

            // Clamp speeds
            if (left_speed > 5.0f) left_speed = 5.0f;
            if (right_speed > 5.0f) right_speed = 5.0f;
            if (left_speed < -5.0f) left_speed = -5.0f;
            if (right_speed < -5.0f) right_speed = -5.0f;

            set_motor(c, left_speed, right_speed);

            //  printf("[PID] Pos=%.3f Err=%.3f Corr=%.3f | L=%.2f R=%.2f\n",
            //        position, error, correction, left_speed, right_speed);

            last_error = error;
        }

        SLEEP(50);  // 50 ms
    }
    return NULL;
}

/*===============================================================================================*/

/**
 * @brief Main function - Entry point of the program
 * @return 0 if successful, -1 if connection failed
 * 
 * This function:
 * 1. Connects to the CoppeliaSim server
 * 2. Starts the control thread for robot behavior
 * 3. Continuously displays sensor data
 * 4. Handles cleanup when program exits
 */
int main() {
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }
    
    printf("Successfully connected to CoppeliaSim server!\n");
    printf("Starting control thread...\n");
    
    // Start the control thread for robot behavior
#ifdef _WIN32
    client.control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_create(&client.control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        if (client.sensor_count > 0) {
            printf("Sensors (%d): ", client.sensor_count);
            for (int i = 0; i < client.sensor_count; i++) {
                printf("%.3f ", client.sensor_values[i]);
            }
            printf("\n");
        } else {
            printf("Waiting for sensor data...\n");
        }
        
        SLEEP(200);  // Update display every 200ms
    }
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}