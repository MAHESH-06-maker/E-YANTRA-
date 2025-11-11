/*
 *
 *   ===================================================
 *       CropDrop Bot (CB) Theme [eYRC 2025-26]
 *   ===================================================
 *
 *  This script is intended to be an Boilerplate for
 *  Task 2b of CropDrop Bot (CB) Theme [eYRC 2025-26].
 *
 *  Filename:		Task2b.c
 *  Created:		    19/08/2025
 *  Last Modified:	12/09/2025
 *  Author:		    Team members Name
 *  Team ID:		    [ CB_xxxx ]
 *
 **********************************************
 */

// Platform-specific includes for Windows compatibility
#ifdef _WIN32
#define WINVER 0x0600
#define _WIN32_WINNT 0x0600
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
typedef SOCKET SocketType;
#define CLOSESOCKET closesocket
#define READ(s, buf, len) recv(s, buf, len, 0)
#define SLEEP(ms) Sleep(ms)
typedef HANDLE THREAD_TYPE;
#else
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <sys/time.h>
typedef int SocketType;
#define CLOSESOCKET close
#define READ(s, buf, len) read(s, buf, len)
#define SLEEP(ms) usleep((ms) * 1000)
typedef pthread_t THREAD_TYPE;
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// ========================================
// STRUCTURE DEFINITIONS
// ========================================

// Structure to hold socket client data and sensor information
typedef struct
{
    SocketType sock;
    bool running;

    // Line sensors (5 sensors)
    float line_sensors[5]; // left_corner, left, middle, right, right_corner

    // Proximity sensor
    float proximity_distance; // Proximity sensor raw distance in meters

    // Color sensor (RGB values)
    float color_r, color_g, color_b; // RGB color raw values (0.0-1.0)

    // Color detection result storage
    char detected_color[32]; // Store detected color name

    THREAD_TYPE recv_thread;
} SocketClient;

// Robot states
typedef enum
{
    STATE_LINE_FOLLOW,
    STATE_APPROACHING_BOX,
    STATE_PICKING_BOX,
    STATE_CARRYING_TO_DROP,
    STATE_DROPPING_BOX,
    STATE_RESUMING,
    STATE_COMPLETED
} RobotState;

// ========================================
// GLOBAL VARIABLES
// ========================================

// Global client instance for socket communication
SocketClient client;

// ========================================
// FUNCTION DECLARATIONS
// ========================================

void *receive_loop(void *arg);
void *control_loop(void *arg);
int connect_to_server(SocketClient *c, const char *ip, int port);
void set_motor(SocketClient *c, float left, float right);
void disconnect(SocketClient *c);
int pick_box(SocketClient *c);
int drop_box(SocketClient *c);
int send_color(SocketClient *c, const char *color);
double get_current_time();

// ========================================
// FUNCTION IMPLEMENTATIONS
// ========================================

/**
 * @brief Sends motor control commands to the robot
 */
void set_motor(SocketClient *c, float left, float right)
{
    if (c->sock != -1)
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "L:%.2f;R:%.2f\n", left, right);
        send(c->sock, cmd, strlen(cmd), 0);
    }
}

/**
 * @brief Send pick command to the robot
 */
int pick_box(SocketClient *c)
{
    if (!c->running || c->sock == -1)
        return 0;

    char message[] = "PICK\n";
    int bytes_sent = send(c->sock, message, strlen(message), 0);
    return (bytes_sent > 0) ? 1 : 0;
}

/**
 * @brief Send drop command to the robot
 */
int drop_box(SocketClient *c)
{
    if (!c->running || c->sock == -1)
        return 0;

    char message[] = "DROP\n";
    int bytes_sent = send(c->sock, message, strlen(message), 0);
    return (bytes_sent > 0) ? 1 : 0;
}

/**
 * @brief Send color data to the server
 */
int send_color(SocketClient *c, const char *color)
{
    if (!c->running || c->sock == -1 || !color)
        return 0;

    char message[64];
    snprintf(message, sizeof(message), "COLOR:%s\n", color);
    int bytes_sent = send(c->sock, message, strlen(message), 0);

    if (bytes_sent > 0)
    {
        printf("Sent color data to server: %s\n", color);
        return 1;
    }
    return 0;
}

/**
 * @brief Cleanly disconnects from the server and cleans up resources
 */
void disconnect(SocketClient *c)
{
    c->running = false;

#ifdef _WIN32
    WaitForSingleObject(c->recv_thread, INFINITE);
#else
    pthread_join(c->recv_thread, NULL);
#endif

    if (c->sock != -1)
    {
        CLOSESOCKET(c->sock);
        c->sock = -1;
    }

#ifdef _WIN32
    WSACleanup();
#endif
}

/**
 * @brief Thread function that continuously receives sensor data from the server
 */
void *receive_loop(void *arg)
{
    SocketClient *c = (SocketClient *)arg;
    char buffer[2048];
    char line_buffer[2048] = {0};
    int line_pos = 0;

    while (c->running)
    {
        int n = READ(c->sock, buffer, sizeof(buffer) - 1);
        if (n > 0)
        {
            buffer[n] = '\0';

            for (int i = 0; i < n; i++)
            {
                if (buffer[i] == '\n')
                {
                    line_buffer[line_pos] = '\0';

                    if (strncmp(line_buffer, "PICK:", 5) == 0)
                    {
                        printf("Pick response: %s\n", line_buffer + 5);
                    }
                    else if (strncmp(line_buffer, "DROP:", 5) == 0)
                    {
                        printf("Drop response: %s\n", line_buffer + 5);
                    }
                    else if (strncmp(line_buffer, "COLOR:", 6) == 0)
                    {
                        printf("Color response: %s\n", line_buffer + 6);
                    }
                    else
                    {
                        char line_copy[2048];
                        strcpy(line_copy, line_buffer);

                        char *segment = line_copy;
                        char *next_segment = NULL;

                        do
                        {
                            next_segment = strchr(segment, ';');
                            if (next_segment)
                            {
                                *next_segment = '\0';
                                next_segment++;
                            }

                            if (strncmp(segment, "S:", 2) == 0)
                            {
                                char *values = segment + 2;
                                char values_copy[256];
                                strcpy(values_copy, values);

                                char *token = values_copy;
                                char *next_token = NULL;
                                int idx = 0;

                                do
                                {
                                    next_token = strchr(token, ',');
                                    if (next_token)
                                    {
                                        *next_token = '\0';
                                        next_token++;
                                    }

                                    if (idx < 5)
                                    {
                                        c->line_sensors[idx++] = (float)atof(token);
                                    }
                                    token = next_token;
                                } while (token && idx < 5);
                            }
                            else if (strncmp(segment, "P:", 2) == 0)
                            {
                                c->proximity_distance = (float)atof(segment + 2);
                            }
                            else if (strncmp(segment, "C:", 2) == 0)
                            {
                                char *values = segment + 2;
                                char values_copy[256];
                                strcpy(values_copy, values);

                                char *r_str = values_copy;
                                char *g_str = strchr(r_str, ',');
                                char *b_str = NULL;

                                if (g_str)
                                {
                                    *g_str = '\0';
                                    g_str++;
                                    b_str = strchr(g_str, ',');
                                    if (b_str)
                                    {
                                        *b_str = '\0';
                                        b_str++;
                                    }
                                }

                                c->color_r = (float)atof(r_str);
                                if (g_str)
                                    c->color_g = (float)atof(g_str);
                                if (b_str)
                                    c->color_b = (float)atof(b_str);
                            }

                            segment = next_segment;
                        } while (segment);
                    }

                    line_pos = 0;
                }
                else
                {
                    if (line_pos < sizeof(line_buffer) - 1)
                    {
                        line_buffer[line_pos++] = buffer[i];
                    }
                }
            }
        }
        SLEEP(1);
    }
    return NULL;
}

/**
 * @brief Establishes connection to the CoppeliaSim server
 */
int connect_to_server(SocketClient *c, const char *ip, int port)
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif

    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0)
    {
        printf("Socket creation failed\n");
        return 0;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

#ifdef _WIN32
    serv_addr.sin_addr.s_addr = inet_addr(ip);
#else
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);
#endif

    if (connect(c->sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

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
double get_current_time()
{
#ifdef _WIN32
    return (double)GetTickCount() / 1000.0;
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
#endif
}

/**
 * @brief Main control loop thread for robot behavior
 */
void *control_loop(void *arg)
{
    SocketClient *c = (SocketClient *)arg;

    // ========= PID parameters (tune these for your robot) =========
    float Kp = 1.7f;
    float Ki = 0.0004f;
    float Kd = 0.35f;
    float base_speed = 0.23f;
    // slower = better control
    float base_speed_carrying = 0.20f;
    float max_correction = 1.2f;
    float derivative_smoothing = 0.15f;
    float integral_limit = 2.0f; // limit for integral accumulation

    // ========= Line detection parameters =========
    float black_threshold = 0.4f;
    float white_threshold = 0.6f;

    // ========= Runtime state variables =========
    float last_error = 0.0f;
    float integral = 0.0f;
    float last_derivative = 0.0f;

    int boxes_detected = 0;
    int is_carrying_box = 0;
    RobotState robot_state = STATE_LINE_FOLLOW;

    double last_box_time = 0.0;
    double box_cooldown = 1.5;
    double carry_start_time = 0.0;

    /* Navigation helpers for drop-zone navigation */
    int drop_nav_phase = 0; /* 0=not started, 1=driving to drop */
    char target_color[32] = "";
    float drop_drive_time = 2.0f;   /* seconds to drive into drop zone */
    float drop_rotate_time = 0.7f;  /* seconds to rotate to drop direction */

    char last_detected_colors[3][20] = {"", "", ""};

    printf("\n");
    printf("==================================================\n");
    printf("       Task 2b - Control Loop Started\n");
    printf("==================================================\n");
    printf("Objective: Follow line, detect 3 boxes, pick 3rd box\n");
    printf("--------------------------------------------------\n\n");

    while (c->running)
    {
        double now = get_current_time();

        // ========= Read all sensors =========
        float ir[5];
        for (int i = 0; i < 5; ++i)
        {
            ir[i] = c->line_sensors[i];
        }

        float proximity = c->proximity_distance;
        float r = c->color_r;
        float g = c->color_g;
        float b = c->color_b;

        // ========= Determine line type (black or white) =========
        float avg_sensor = 0.0f;
        for (int i = 0; i < 5; ++i)
            avg_sensor += ir[i];
        avg_sensor /= 5.0f;

        // --- Detect which region we are in (dark or light floor)
        static int follow_black = 1; // start assuming black floor with white line
        static int last_zone = 0;    // 0 = left/dark zone, 1 = right/bright zone

        // Compute overall brightness
        float brightness = avg_sensor;

        // Switch zones based on threshold
        // Left zone has avg < 0.4 (dark floor), right zone avg > 0.6 (white floor)
        if (brightness < 0.4 && last_zone != 0)
        {
            follow_black = 1; // follow white line
            last_zone = 0;
            printf("Switched to DARK zone → following WHITE line\n");
        }
        else if (brightness > 0.6 && last_zone != 1)
        {
            follow_black = 0; // follow black line
            last_zone = 1;
            printf("Switched to BRIGHT zone → following BLACK line\n");
        }

        // ========= Compute line position error =========
        float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
        float line_values[5];

        for (int i = 0; i < 5; ++i)
        {
            if (follow_black)
            {
                line_values[i] = 1.0f - ir[i];
            }
            else
            {
                line_values[i] = ir[i];
            }
        }

        float numer = 0.0f, denom = 0.0f;
        for (int i = 0; i < 5; ++i)
        {
            numer += weights[i] * line_values[i];
            denom += line_values[i];
        }

        int line_detected = (denom > 0.05f);
        float position = line_detected ? (numer / denom) : 0.0f;

        // ========= State Machine =========
        float left_speed = 0.0f, right_speed = 0.0f;

        switch (robot_state)
        {
        case STATE_LINE_FOLLOW:
        {
            float error = position;

            integral += error;
            if (integral > integral_limit)
                integral = integral_limit;
            if (integral < -integral_limit)
                integral = -integral_limit;

            float derivative = error - last_error;
            derivative = derivative_smoothing * last_derivative +
                         (1.0f - derivative_smoothing) * derivative;

            float correction = Kp * error + Ki * integral + Kd * derivative;
            if (correction > max_correction)
                correction = max_correction;
            if (correction < -max_correction)
                correction = -max_correction;

            last_error = error;
            last_derivative = derivative;

            float current_base = is_carrying_box ? base_speed_carrying : base_speed;
            left_speed = current_base - correction;
            right_speed = current_base + correction;

            if (!line_detected)
            {
                if (last_error >= 0.0f)
                {
                    left_speed = 0.2f;
                    right_speed = -0.2f;
                }
                else
                {
                    left_speed = -0.2f;
                    right_speed = 0.2f;
                }
            }

            float abs_correction = (correction < 0) ? -correction : correction;
            if (abs_correction > 0.6f)
            {
                left_speed *= 0.6f;
                right_speed *= 0.6f;
            }

            if (proximity > 0.0f && proximity < 0.11f &&
                (now - last_box_time) > box_cooldown &&
                boxes_detected < 3 && !is_carrying_box)
            {

                // Make sure it's a new box (ignore reflections)
                SLEEP(200);
                if (c->proximity_distance < 0.11f)
                {
                    printf("\n[NEW BOX DETECTED]\n");
                    robot_state = STATE_APPROACHING_BOX;
                    set_motor(c, 0.0f, 0.0f);
                }
            }
            /*
             * Only transition to APPROACHING_BOX when proximity
             * conditions are met (above). The stray unconditional
             * block was removed to prevent false positive detections.
             */
            break;
        }

        case STATE_APPROACHING_BOX:
        {
            /* If this is the 3rd box, stabilize and average several color samples
             * to make a robust decision, then mark it as the target */
            int samples = 5;
            float sr = 0.0f, sg = 0.0f, sb = 0.0f;

            for (int i = 0; i < samples; ++i)
            {
                /* small delay to let receive_loop update color values */
                SLEEP(80);
                sr += c->color_r;
                sg += c->color_g;
                sb += c->color_b;
            }

            sr /= samples;
            sg /= samples;
            sb /= samples;

            float max_channel = sr;
            if (sg > max_channel)
                max_channel = sg;
            if (sb > max_channel)
                max_channel = sb;

            float color_threshold = 0.12f;
            const char *detected_color = "unknown";

            if (sr == max_channel && sr > sg + color_threshold && sr > sb + color_threshold)
            {
                detected_color = "red";
            }
            else if (sg == max_channel && sg > sr + color_threshold && sg > sb + color_threshold)
            {
                detected_color = "green";
            }
            else if (sb == max_channel && sb > sr + color_threshold && sb > sg + color_threshold)
            {
                detected_color = "blue";
            }
            else
            {
                if (sr >= sg && sr >= sb)
                    detected_color = "red";
                else if (sg >= sr && sg >= sb)
                    detected_color = "green";
                else
                    detected_color = "blue";
            }

            /* Only increment box count after stable detection */
            strcpy(last_detected_colors[boxes_detected], detected_color);
            boxes_detected++;
            last_box_time = now;

            /* If this is the 3rd box, set it as the target and send color */
            send_color(c, detected_color);

            printf("----------------------------------------\n");
            printf("Box #%d Identified\n", boxes_detected);
            printf("  Color: %s\n", detected_color);
            printf("  RGB(avg): (%.2f, %.2f, %.2f)\n", sr, sg, sb);
            printf("  Proximity: %.3f m\n", proximity);
            printf("----------------------------------------\n");

            if (boxes_detected == 3)
            {
                printf("\n>>> TARGET BOX FOUND (3rd box) <<<\n");
                /* store the target color for drop navigation */
                strncpy(target_color, detected_color, sizeof(target_color) - 1);
                target_color[sizeof(target_color) - 1] = '\0';

                robot_state = STATE_PICKING_BOX;
            }
            else
            {
                printf("Skipping this box (not the 3rd)...\n\n");
                robot_state = STATE_LINE_FOLLOW;
                SLEEP(300);
            }
            break;
        }

        case STATE_PICKING_BOX:
        {
            printf("\n========================================\n");
            printf("  PICKING 3RD BOX (%s)\n", last_detected_colors[2]);
            printf("========================================\n");

            printf("Aligning with box...\n");
            set_motor(c, 0.12f, 0.12f);
            SLEEP(400);
            set_motor(c, 0.0f, 0.0f);
            SLEEP(150);

            printf("Executing pick operation...\n");
            pick_box(c);
            is_carrying_box = 1;
            printf("Box picked successfully!\n");

            /* small backup to clear the spawn area */
            printf("Backing up...\n");
            set_motor(c, -0.18f, -0.18f);
            SLEEP(350);
            set_motor(c, 0.0f, 0.0f);
            SLEEP(150);

            /* Prepare navigation to drop zone based on detected color */
            printf("Preparing to navigate to drop zone for color: %s\n", target_color);
            drop_nav_phase = 0; /* will begin rotation in carrying state */

            robot_state = STATE_CARRYING_TO_DROP;
            carry_start_time = now;
            break;
        }

        case STATE_CARRYING_TO_DROP:
        {
            /* Navigate to the drop zone based on the target color.
             * Phase 0: rotate towards the target direction.
             * Phase 1: drive forward for a fixed time into the drop zone.
             */
            if (!is_carrying_box)
            {
                /* If we lost the box somehow, resume line following */
                robot_state = STATE_LINE_FOLLOW;
                break;
            }

            if (drop_nav_phase == 0)
            {
                float rot_speed = 0.28f;

                printf("Starting rotation towards drop zone for color: %s\n", target_color);

                if (strncmp(target_color, "red", 3) == 0)
                {
                    /* rotate left */
                    set_motor(c, -rot_speed, rot_speed);
                    SLEEP((int)(drop_rotate_time * 1000.0f));
                }
                else if (strncmp(target_color, "blue", 4) == 0)
                {
                    /* rotate right */
                    set_motor(c, rot_speed, -rot_speed);
                    SLEEP((int)(drop_rotate_time * 1000.0f));
                }
                else
                {
                    /* green or unknown: go straight (no rotation) */
                    /* small pause to stabilize */
                    SLEEP(120);
                }

                set_motor(c, 0.0f, 0.0f);
                SLEEP(120);

                drop_nav_phase = 1;
                carry_start_time = now;
            }
            else if (drop_nav_phase == 1)
            {
                /* Drive forward into the drop zone */
                set_motor(c, base_speed_carrying, base_speed_carrying);

                if ((now - carry_start_time) > drop_drive_time)
                {
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(80);
                    printf("\n>>> DROP ZONE REACHED (by timed drive) <<<\n");
                    robot_state = STATE_DROPPING_BOX;
                    SLEEP(200);
                }
            }

            break;
        }

        case STATE_DROPPING_BOX:
        {
            printf("========================================\n");
            printf("  DROPPING BOX IN %s ZONE\n", last_detected_colors[2]);
            printf("========================================\n");

            drop_box(c);
            is_carrying_box = 0;

            printf("Box dropped successfully!\n");
            printf("========================================\n");
            printf("  TASK COMPLETED!\n");
            printf("========================================\n\n");

            robot_state = STATE_COMPLETED;
            SLEEP(500);
            break;
        }

        case STATE_COMPLETED:
        {
            set_motor(c, 0.0f, 0.0f);
            break;
        }

        case STATE_RESUMING:
        {
            printf("Resuming line following...\n");
            robot_state = STATE_LINE_FOLLOW;
            break;
        }
        }

        if (left_speed > 1.0f)
            left_speed = 1.0f;
        if (left_speed < -1.0f)
            left_speed = -1.0f;
        if (right_speed > 1.0f)
            right_speed = 1.0f;
        if (right_speed < -1.0f)
            right_speed = -1.0f;

        if (robot_state == STATE_LINE_FOLLOW || robot_state == STATE_CARRYING_TO_DROP)
        {
            set_motor(c, left_speed, right_speed);
        }

        SLEEP(25);
    }

    printf("\nControl loop terminated.\n");
    return NULL;
}

/**
 * @brief Main function - Entry point of the program
 */
int main()
{
    printf("\n");
    printf("==================================================\n");
    printf("     CropDrop Bot - Task 2b\n");
    printf("     eYantra Robotics Competition 2025-26\n");
    printf("==================================================\n\n");

    printf("Connecting to CoppeliaSim server...\n");
    if (!connect_to_server(&client, "127.0.0.1", 50002))
    {
        printf("\n[ERROR] Failed to connect to CoppeliaSim server\n");
        printf("Please ensure:\n");
        printf("  1. CoppeliaSim is running\n");
        printf("  2. The simulation scene is loaded\n");
        printf("  3. The ZMQ remote API is enabled on port 50002\n\n");
        return -1;
    }

    printf("Successfully connected to CoppeliaSim!\n");
    printf("Starting control thread...\n");

#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
    if (control_thread == NULL)
    {
        printf("[ERROR] Failed to create control thread\n");
        disconnect(&client);
        return -1;
    }
#else
    pthread_t control_thread;
    if (pthread_create(&control_thread, NULL, control_loop, &client) != 0)
    {
        printf("[ERROR] Failed to create control thread\n");
        disconnect(&client);
        return -1;
    }
#endif

    printf("Control thread started successfully\n\n");

    printf("Robot is now active. Press Ctrl+C to exit.\n");
    printf("--------------------------------------------------\n");

    while (1)
    {
        SLEEP(1000);
    }

    printf("\nShutting down...\n");
    client.running = false;

#ifdef _WIN32
    WaitForSingleObject(control_thread, INFINITE);
    CloseHandle(control_thread);
#else
    pthread_join(control_thread, NULL);
#endif

    disconnect(&client);
    printf("Disconnected. Goodbye!\n");

    return 0;
}