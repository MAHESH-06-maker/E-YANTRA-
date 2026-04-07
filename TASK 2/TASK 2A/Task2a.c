/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 2a of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Task2a.c
*  Created:		    19/08/2025
*  Last Modified:	12/09/2025
*  Author:		    Team members Name
*  Team ID:		    [ CB_xxxx ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
**********************************************

*/
// Platform-specific includes for Windows compatibility
#ifdef _WIN32
    #define WINVER 0x0600
    #define _WIN32_WINNT 0x0600
    #include <winsock2.h>
    #include <ws2tcpip.h>
#endif

#include "coppeliasim_client.h"  // Include our header
#include <sys/time.h>
#include <math.h>
#include <string.h>

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations
// ----------------------
void* control_loop(void* arg);

/**
 * @brief Establishes connection to the CoppeliaSim server
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

/**
 * @brief Get current time in seconds
 */
double get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}
/*******************************
  COMPUTE LINE POSITION FUNCTION
********************************/
float compute_line_position(SocketClient * c){
    int weight[5] = {-2, -1, 0, 1, 2};
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int i = 0; i < 5; i++) {
        // Invert the raw value so black → 1.0, white → 0.0
        float value = 1.0f - c->line_sensors[i];
        numerator += value * weight[i];
        denominator += value;
    }

    if (denominator == 0.0f)
        return 0.0f;

    return numerator / denominator;
}
// ----------------------------
// Global robot state variables
// ----------------------------
int box_picked = 0;           // 0 = not yet picked, 1 = picked
char box_color[10] = "none";  // store color string: "red", "green", "blue"
int at_junction = 0;          // flag to mark if junction detected
int drop_done = 0;            // 1 when drop is completed

// Identify box color based on RGB readings
const char* detect_box_color(float r, float g, float b) {
    if (g > 0.4f && b < 0.1f) return "red";      // Red box
    else if (b > 0.4f && g < 0.1f) return "green"; // Green box
    else if (r > 0.1f && g < 0.05f && b < 0.05f) return "blue"; // Blue box
    else return "unknown";
}

/**
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    
    while (c->running) {
        float threshold = 0.43f;
        int valsensor[5];//binary vlaues 0 for black and 1 for white 
        float kp = 0.4f;
        float kd = 0.65f;
        float ki = 0.00002f;
        float last_error = 0.0f;
        float integral  = 0.0f;
        float base_speed = 3.0f; //speed of bot

        // ========================================
        // LINE FOLLOWING SENSORS (IR SENSORS)
        // ========================================
        // These sensors detect black lines on white surface
        // Values typically range from 0.0 to 1.0
        // Lower values indicate darker surface (line detected)
        // Higher values indicate lighter surface (no line)
        float ir1 = c->line_sensors[0];  // left_corner sensor
        float ir2 = c->line_sensors[1];  // left sensor
        float ir3 = c->line_sensors[2];  // middle sensor (center)
        float ir4 = c->line_sensors[3];  // right sensor
        float ir5 = c->line_sensors[4];  // right_corner sensor
        // printf("Readings: [%.2f, %.2f, %.2f, %.2f, %.2f]-->", ir1, ir2, ir3, ir4, ir5);

        for(int i =0 ; i<5; i++){
            if(c->line_sensors[i]<threshold){
                valsensor[i]=0;//0 for black 
            }
            else{
                valsensor[i]=1;//1 for white
            }
        }

        // Print binary values for debugging
        printf("Binary:[");
        for (int i = 0; i < 5; i++) {
            printf("%d", valsensor[i]);
            if (i < 4) printf(" ");
        }
        printf("]-->");
        
        if (ir1 < threshold &&  ir5 > threshold){
            //LEFT TURN 
            set_motor(c,-1.0f,2.0f);
        }   
        else if (ir1 > threshold &&  ir5 < threshold) {
            //RIGHT TURN 
            set_motor(c, 2.0f, -1.0f);
        } 
        else{
            //PID LOGIC
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

            //PRINT VALUES FOR DEBUGGING
            //  printf("[PID] Pos=%.3f Err=%.3f Corr=%.3f | L=%.2f R=%.2f\n",
            //        position, error, correction, left_speed, right_speed);

            last_error = error;
           }
        
        
        // ========================================
        // PROXIMITY SENSOR (ULTRASONIC/DISTANCE)
        // ========================================
        // Detects obstacles or objects in front of the robot
        // Value represents distance in meters
        // Smaller values indicate closer obstacles
        // Use this for obstacle avoidance or box detection
        float proximity = c->proximity_distance;
        printf("Proximity sensor distance: %.3f meters\n", proximity);


        // ========================================
        // COLOR SENSOR (RGB VALUES)
        // ========================================
        // Detects color of surface beneath the sensor
        // RGB values range from 0.0 to 1.0
        // Use these values to identify colored boxes or markers
        float r = c->color_r, g = c->color_g, b = c->color_b;
        printf("Color RGB raw values: (%.3f, %.3f, %.3f)\n", r, g, b);


        // ========================================
        // BOX DETECTION AND PICKING LOGIC
        // ========================================
        if (!box_picked  && proximity <=0.144f) {
            const char* color = detect_box_color(r, g, b);
            if (strcmp(color, "unknown") != 0) {
                strcpy(box_color, color);
                printf("\n🎯 Box detected! Color: %s | Distance: %.3f\n", box_color, proximity);

                // Stop for 1 second
                set_motor(c, 0, 0);
                SLEEP(1000);

                // Pick up the box
                pick_box(c);
                box_picked = 1;

                printf("✅ Box picked successfully!\n");
            }
        }

        // ========================================
        // PLUS JUNCTION DETECTION & TURN DECISION
        // ========================================
        if (box_picked && !at_junction) {
            // Plus junction detected when all sensors see black line
            if (ir1 < threshold && ir2 < threshold && ir3 < threshold && ir4 < threshold && ir5 < threshold) {
                at_junction = 1;
                SLEEP(500);
                set_motor(c, 0, 0);
                printf("\n🧭 Plus junction detected!\n");
                SLEEP(1000);

                if (strcmp(box_color, "red") == 0) {
                    printf("🔴 Turning LEFT for red box...\n");
                    set_motor(c, -2.0f, 2.5f);  // Rotate left
                    SLEEP(900);
                    
                }
                else if (strcmp(box_color, "green") == 0) {
                    printf("🟢 Turning RIGHT for green box...\n");
                    set_motor(c, 2.5f, -2.0f);  // Rotate right
                    SLEEP(900);
                } 
                else if (strcmp(box_color, "blue") == 0) {
                    printf("🔵 Going STRAIGHT for blue box...\n");
                    set_motor(c, 2.0f, 2.0f);
                    SLEEP(600);
                }

                // Resume normal line following
                set_motor(c, 0, 0);
                SLEEP(300);
            }
        }

        // ========================================
        // DROP ZONE DETECTION LOGIC
        // ========================================
        if (box_picked && !drop_done) {
            // Drop zone detected by Ir Values when all Sensor see white 
            if ((strcmp(box_color, "blue") == 0 &&(ir1>threshold && ir2 >threshold && ir3 >threshold && ir4 >threshold && ir5 >threshold)) ||
                (strcmp(box_color, "red") == 0 && (ir1>threshold && ir2 >threshold && ir3 >threshold && ir4 >threshold && ir5 >threshold)) ||
                (strcmp(box_color, "green") == 0 &&(ir1>threshold && ir2 >threshold && ir3 >threshold && ir4 >threshold && ir5 >threshold))) {

                printf("\n📦 Drop zone detected for %s box!\n", box_color);
                set_motor(c, 0, 0);
                SLEEP(1000);

                drop_box(c);
                drop_done = 1;

                printf("✅ Box dropped successfully!\n");

                // Stop robot at final zone
                set_motor(c, 0, 0);
                break;
                
            }
        }

        
        SLEEP(50);  // Wait 5ms before next iteration
    }
    return NULL;
}



/**
 * @brief Main function - Entry point of the program
 */
int main() {
    // Attempt to connect to CoppeliaSim server
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
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        SLEEP(100);  // Update display every 100ms
    }

    // Cleanup
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}