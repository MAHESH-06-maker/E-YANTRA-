#ifndef COPPELIASIM_CLIENT_H
#define COPPELIASIM_CLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    typedef SOCKET SocketType;
    #define CLOSESOCKET closesocket
    #define READ(s, buf, len) recv(s, buf, len, 0)
    #define SLEEP(ms) Sleep(ms)
    #pragma comment(lib, "Ws2_32.lib")
#else
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <pthread.h>
    typedef int SocketType;
    #define CLOSESOCKET close
    #define READ(s, buf, len) read(s, buf, len)
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

// Structure to hold socket client data and sensor information
typedef struct {
    SocketType sock;                    
    bool running;                       
    
    // Line sensors (5 sensors)
    float line_sensors[5];              // left_corner, left, middle, right, right_corner
    
    // Proximity sensor
    float proximity_distance;           // Proximity sensor raw distance in meters
    
    // Color sensor (RGB values)
    float color_r, color_g, color_b;    // RGB color raw values (0.0-1.0)
    
#ifdef _WIN32
    HANDLE recv_thread;                 
#else
    pthread_t recv_thread;              
#endif
} SocketClient;

// Function declarations
void set_motor(SocketClient* c, float left, float right);
void disconnect(SocketClient* c);
void* receive_loop(void* arg);

// Pick and place function declarations
int pick_box(SocketClient* c);
int drop_box(SocketClient* c);

// Function implementations
/**
 * @brief Sends motor control commands to the robot
 */
void set_motor(SocketClient* c, float left, float right) {
    if (c->sock != -1) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "L:%.2f;R:%.2f\n", left, right);
        send(c->sock, cmd, strlen(cmd), 0);
    }
}

/**
 * @brief Send pick command to the robot
 * @param c Pointer to SocketClient structure
 * @return 1 if command sent successfully, 0 if failed
 */
int pick_box(SocketClient* c) {
    if (!c->running || c->sock == -1) return 0;
    
    char message[] = "PICK\n";
    int bytes_sent = send(c->sock, message, strlen(message), 0);
    return (bytes_sent > 0) ? 1 : 0;
}

/**
 * @brief Send drop command to the robot
 * @param c Pointer to SocketClient structure
 * @return 1 if command sent successfully, 0 if failed
 */
int drop_box(SocketClient* c) {
    if (!c->running || c->sock == -1) return 0;
    
    char message[] = "DROP\n";
    int bytes_sent = send(c->sock, message, strlen(message), 0);
    return (bytes_sent > 0) ? 1 : 0;
}

/**
 * @brief Cleanly disconnects from the server and cleans up resources
 */
void disconnect(SocketClient* c) {
    c->running = false;  // Signal threads to stop
    
    // Wait for receive thread to finish
#ifdef _WIN32
    WaitForSingleObject(c->recv_thread, INFINITE);
#else
    pthread_join(c->recv_thread, NULL);
#endif
    
    // Close socket if open
    if (c->sock != -1) {
        CLOSESOCKET(c->sock);
        c->sock = -1;
    }
    
    // Cleanup Windows socket library
#ifdef _WIN32
    WSACleanup();
#endif
}

/**
 * @brief Thread function that continuously receives sensor data from the server
 * @param arg Pointer to SocketClient structure (cast from void*)
 * @return NULL when thread exits
 * 
 * This function runs in a separate thread and parses incoming sensor data.
 * Expected data formats: 
 * - "S:val1,val2,val3,val4,val5;P:distance;C:r,g,b\n"
 */
void* receive_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    char buffer[2048];
    char line_buffer[2048] = {0};
    int line_pos = 0;
    
    while (c->running) {
        // Read data from socket
        int n = READ(c->sock, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';  // Null-terminate the received string
            
            // Process character by character to handle complete lines
            for (int i = 0; i < n; i++) {
                if (buffer[i] == '\n') {
                    // Complete line received
                    line_buffer[line_pos] = '\0';
                    
                    // Parse the line: "S:val1,val2,val3,val4,val5;P:distance;C:r,g,b"
                    char line_copy[2048];
                    strcpy(line_copy, line_buffer);
                    
                    char* segment = line_copy;
                    char* next_segment = NULL;
                    
                    // Parse each segment separated by semicolon
                    do {
                        next_segment = strchr(segment, ';');
                        if (next_segment) {
                            *next_segment = '\0';
                            next_segment++;
                        }
                        
                        if (strncmp(segment, "S:", 2) == 0) {
                            // Line sensor data: "S:val1,val2,val3,val4,val5"
                            char* values = segment + 2;
                            char values_copy[256];
                            strcpy(values_copy, values);
                            
                            char* token = values_copy;
                            char* next_token = NULL;
                            int idx = 0;
                            
                            do {
                                next_token = strchr(token, ',');
                                if (next_token) {
                                    *next_token = '\0';
                                    next_token++;
                                }
                                
                                if (idx < 5) {
                                    c->line_sensors[idx++] = (float)atof(token);
                                }
                                token = next_token;
                            } while (token && idx < 5);
                            
                        } else if (strncmp(segment, "P:", 2) == 0) {
                            // Proximity sensor: "P:distance"
                            c->proximity_distance = (float)atof(segment + 2);
                            
                        } else if (strncmp(segment, "C:", 2) == 0) {
                            // Color sensor: "C:r,g,b"
                            char* values = segment + 2;
                            char values_copy[256];
                            strcpy(values_copy, values);
                            
                            char* r_str = values_copy;
                            char* g_str = strchr(r_str, ',');
                            char* b_str = NULL;
                            
                            if (g_str) {
                                *g_str = '\0';
                                g_str++;
                                b_str = strchr(g_str, ',');
                                if (b_str) {
                                    *b_str = '\0';
                                    b_str++;
                                }
                            }
                            
                            c->color_r = (float)atof(r_str);
                            if (g_str) c->color_g = (float)atof(g_str);
                            if (b_str) c->color_b = (float)atof(b_str);
                        }
                        
                        segment = next_segment;
                    } while (segment);
                    
                    line_pos = 0;  // Reset line buffer
                } else {
                    // Add character to line buffer
                    if (line_pos < sizeof(line_buffer) - 1) {
                        line_buffer[line_pos++] = buffer[i];
                    }
                }
            }
        }
        SLEEP(1);  // Small delay to prevent excessive CPU usage
    }
    return NULL;
}

#endif // COPPELIASIM_CLIENT_H