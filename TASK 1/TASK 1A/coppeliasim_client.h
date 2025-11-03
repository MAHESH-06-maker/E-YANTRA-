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
    float sensor_values[32];            
    int sensor_count;                   
#ifdef _WIN32
    HANDLE recv_thread;                 
    HANDLE control_thread;              
#else
    pthread_t recv_thread;              
    pthread_t control_thread;           
#endif
} SocketClient;

// Function declarations
void set_motor(SocketClient* c, float left, float right);
void disconnect(SocketClient* c);
void* receive_loop(void* arg);

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
 * Expected data format: "S:<sensor1>,<sensor2>,<sensor3>,...\n"
 * Example: "S:0.125,0.0,1.0,0.5\n" represents 4 sensor values
 */
void* receive_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    char buffer[2048];
    
    while (c->running) {
        // Read data from socket
        int n = READ(c->sock, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';  // Null-terminate the received string
            
            // Check if this is sensor data (starts with "S:")
            if (strncmp(buffer, "S:", 2) == 0) {
                char* values = buffer + 2;  // Skip the "S:" prefix
                char* token = strtok(values, ",");  // Split by commas
                int idx = 0;
                
                // Parse each sensor value
                while (token && idx < 32) {
                    c->sensor_values[idx++] = (float)atof(token);
                    token = strtok(NULL, ",");
                }
                c->sensor_count = idx;  // Store the number of sensors received
            }
        }
        SLEEP(50);  // Small delay to prevent excessive CPU usage
    }
    return NULL;
}

#endif // COPPELIASIM_CLIENT_H