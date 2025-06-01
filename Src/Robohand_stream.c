/*!
* @file Robohand_stream.c
* @brief USB streaming interface for robotic hand control system.
* @details Provides optimized data streaming for PC-based GUI interaction.
* @author Robert Fudge <rnfudge@mun.ca>
* @date 2025
* @copyright Apache 2.0 License
*/

#include "Robohand.h"
#include "Robohand_i2c.h"
#include "Robohand_rgb.h"
#include "Robohand_servos.h"
#include "Robohand_struct.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#define CMD_BUFFER_SIZE 128
#define STREAM_INTERVAL_MS 50  // Default 20Hz update rate
#define JSON_BUFFER_SIZE 512

// Protocol commands
#define CMD_SERVO "SERVO"
#define CMD_SENSORS "SENSORS"
#define CMD_STATUS "STATUS"
#define CMD_RGB "RGB"
#define CMD_STREAM "STREAM"
#define CMD_VERSION "VERSION"
#define CMD_RESET "RESET"

// Function prototypes
static void process_command(char* cmd_buffer);
static void stream_sensor_data(void);
static void stream_status_data(void);
static bool streaming_timer_callback(struct repeating_timer *t);
static void handle_servo_command(char* params);
static void handle_rgb_command(char* params);
static void handle_stream_command(char* params);
static void format_json_response(const char* type, const char* data, char* output_buffer, size_t buffer_size);

// Global variables
static struct repeating_timer streaming_timer;
static bool streaming_enabled = false;
static uint32_t streaming_interval_ms = STREAM_INTERVAL_MS;
static bool stream_sensors = true;
static bool stream_status = false;
static uint32_t stream_counter = 0;
static char json_buffer[JSON_BUFFER_SIZE];

// Version info
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

/*!
* @brief Main entry point for the streaming interface.
* @return Should never return in normal operation.
*/
int main() {
    // Initialize standard I/O
    stdio_init_all();
    
    // Small delay to allow for USB connection
    sleep_ms(2000);
    
    // Initialize the robotic hand system
    init_robohand_system();
    
    if (HAS_RGB) {
        init_rgb();
        rgb_set_color(0, 0, 255); // Blue for streaming mode
    }
    
    char cmd_buffer[CMD_BUFFER_SIZE];
    int cmd_pos = 0;
    
    // Send version info on startup
    printf("{\"type\":\"version\",\"data\":{\"major\":%d,\"minor\":%d,\"patch\":%d}}\n", 
           VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    
    // Command processing loop
    while (true) {
        // Read incoming data with timeout
        int c = getchar_timeout_us(1000); // 1ms timeout
        
        if (c != PICO_ERROR_TIMEOUT) {
            // Process character
            if (c == '\r' || c == '\n') {
                // Process Enter key
                cmd_buffer[cmd_pos] = '\0';
                
                // Process command if not empty
                if (cmd_pos > 0) {
                    process_command(cmd_buffer);
                }
                
                // Reset buffer
                cmd_pos = 0;
            } 
            else if (c == 127 || c == '\b') {
                // Process Backspace key
                if (cmd_pos > 0) {
                    cmd_pos--;
                }
            }
            else if (isprint(c) && cmd_pos < (CMD_BUFFER_SIZE - 1)) {
                // Process printable characters
                cmd_buffer[cmd_pos++] = (char)c;
            }
        }
        
        // Update system counters
        mutex_enter_blocking(&sys_status.status_mutex);
        sys_status.core0_loops++;
        mutex_exit(&sys_status.status_mutex);
        
        // Small sleep to prevent tight looping
        sleep_us(100);
    }
}

/*!
* @brief Process a command from the serial interface.
* @param cmd_buffer The null-terminated command string.
*/
static void process_command(char *cmd_buffer) {
    // Skip leading whitespace
    char *cmd = cmd_buffer;
    while (isspace(*cmd)) cmd++;
    
    // Find the command and parameters
    char *params = cmd;
    while (*params && !isspace(*params)) params++;
    
    if (*params) {
        *params = '\0'; // Null-terminate the command
        params++; // Move to parameters
        
        // Skip leading whitespace in parameters
        while (isspace(*params)) params++;
    }
    
    // Convert command to uppercase for case-insensitive comparison
    char cmd_upper[32] = {0};
    for (int i = 0; cmd[i] && i < 31; i++) {
        cmd_upper[i] = (char) toupper(cmd[i]);
    }
    
    // Handle commands
    if (strcmp(cmd_upper, CMD_SERVO) == 0) {
        handle_servo_command(params);
    }
    else if (strcmp(cmd_upper, CMD_SENSORS) == 0) {
        stream_sensor_data();
    }
    else if (strcmp(cmd_upper, CMD_STATUS) == 0) {
        stream_status_data();
    }
    else if (strcmp(cmd_upper, CMD_RGB) == 0) {
        handle_rgb_command(params);
    }
    else if (strcmp(cmd_upper, CMD_STREAM) == 0) {
        handle_stream_command(params);
    }
    else if (strcmp(cmd_upper, CMD_VERSION) == 0) {
        sprintf(json_buffer, "{\"major\":%d,\"minor\":%d,\"patch\":%d}", 
                VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
        format_json_response("version", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
    }
    else if (strcmp(cmd_upper, CMD_RESET) == 0) {
        format_json_response("reset", "{\"status\":\"ok\"}", json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        sleep_ms(100); // Give time for transmission
        watchdog_enable(1, false);  // Enable watchdog with 1ms timeout
        while (1) tight_loop_contents();  // Wait for watchdog to reset
    }
    else {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Unknown command: %s\"}", cmd);
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
    }
}

/*!
* @brief Format a JSON response.
* @param type The type of response.
* @param data The JSON data string (without enclosing braces).
* @param output_buffer Buffer to store the formatted response.
* @param buffer_size Size of the output buffer.
*/
static void format_json_response(const char* type, const char* data, char* output_buffer, size_t buffer_size) {
    snprintf(output_buffer, buffer_size, "{\"type\":\"%s\",\"data\":%s}", type, data);
}

/*!
* @brief Stream sensor data in JSON format.
*/
static void stream_sensor_data(void) {
    sensor_data sensor_values;
    sensor_data_physical physical_values;
    
    if (get_sensor_data(&sensor_values)) {
        convert_sensor_data(&sensor_values, &physical_values);
        
        // Format JSON response
        snprintf(json_buffer, JSON_BUFFER_SIZE, 
                 "{\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                 "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                 "\"mag\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                 "\"adc\":[%.3f,%.3f,%.3f,%.3f,%.3f],"
                 "\"temp\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f,\"altitude\":%.2f}",
                 physical_values.accel[0], physical_values.accel[1], physical_values.accel[2],
                 physical_values.gyro[0], physical_values.gyro[1], physical_values.gyro[2],
                 physical_values.mag[0], physical_values.mag[1], physical_values.mag[2],
                 physical_values.adc_values[0], physical_values.adc_values[1], 
                 physical_values.adc_values[2], physical_values.adc_values[3], 
                 physical_values.adc_values[4],
                 sensor_values.temperature, sensor_values.pressure, 
                 sensor_values.humidity, physical_values.altitude);
        
        format_json_response("sensors", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
    }
}

/*!
* @brief Stream system status data in JSON format.
*/
static void stream_status_data(void) {
    system_status status;
    get_system_status(&status);
    
    // Start JSON string with system info
    snprintf(json_buffer, JSON_BUFFER_SIZE, 
             "{\"core0_load\":%.2f,\"core1_load\":%.2f,"
             "\"system_ok\":%s,\"emergency_stop\":%s,"
             "\"watchdog_time\":%.1f",
             status.core0_load, status.core1_load,
             status.system_ok ? "true" : "false",
             status.emergency_stop ? "true" : "false",
             (float)(time_us_32() - status.last_watchdog) / 1000000.0f);
    
    // Add servo information if available
    if (HAS_SERVOS) {
        strcat(json_buffer, ",\"servos\":[");
        
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            servo_motion_profile profile;
            char servo_json[64];
            
            if (get_servo_status(i, &profile)) {
                snprintf(servo_json, sizeof(servo_json),
                         "%s{\"id\":%d,\"current\":%d,\"target\":%d,\"moving\":%s}",
                         (i > 0) ? "," : "",
                         i, profile.current_pw, profile.target_pw,
                         profile.is_moving ? "true" : "false");
                strcat(json_buffer, servo_json);
            }
        }
        
        strcat(json_buffer, "]");
    }
    
    // Close JSON object
    strcat(json_buffer, "}");
    
    format_json_response("status", json_buffer, json_buffer, JSON_BUFFER_SIZE);
    printf("%s\n", json_buffer);
}

/*!
* @brief Handle servo movement command.
* @param params Command parameters string.
*/
static void handle_servo_command(char* params) {
    if (!HAS_SERVOS) {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Servo support is disabled\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Parse parameters: <servo_id> <position> <duration>
    uint16_t servo_id;
    uint16_t position;
    uint16_t duration;

    if (sscanf(params, "%hu %hu %hu", &servo_id, &position, &duration) != 3) {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Invalid servo parameters\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Validate parameters
    if (servo_id >= NUM_SERVOS) {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Invalid servo ID\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Constrain values
    position = constrain_u16(position, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    duration = constrain_u16(duration, 100, MAX_MOVE_DURATION_MS);
    
    // Move the servo
    actuate_servo((uint8_t) servo_id, position, duration);
    
    // Send success response
    sprintf(json_buffer, "{\"id\":%d,\"position\":%d,\"duration\":%d,\"status\":\"ok\"}", 
            servo_id, position, duration);
    format_json_response("servo", json_buffer, json_buffer, JSON_BUFFER_SIZE);
    printf("%s\n", json_buffer);
}

/*!
* @brief Handle RGB LED color command.
* @param params Command parameters string.
*/
static void handle_rgb_command(char* params) {
    if (!HAS_RGB) {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"RGB support is disabled\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Parse parameters: <r> <g> <b>
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (sscanf(params, "%c %c %c", &r, &g, &b) != 3) {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Invalid RGB parameters\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Constrain values
    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);
    
    // Set the color
    rgb_set_color(r, g, b);
    
    // Send success response
    sprintf(json_buffer, "{\"r\":%d,\"g\":%d,\"b\":%d,\"status\":\"ok\"}", r, g, b);
    format_json_response("rgb", json_buffer, json_buffer, JSON_BUFFER_SIZE);
    printf("%s\n", json_buffer);
}

/*!
* @brief Handle streaming configuration command.
* @param params Command parameters string.
*/
static void handle_stream_command(char* params) {
    const char* token;
    char* rest = params;
    bool valid_command = false;
    
    // Get first token (on/off/configure)
    token = strtok_r(rest, " ", &rest);
    if (!token) {
        // No parameters, report current status
        sprintf(json_buffer, "{\"enabled\":%s,\"interval\":%lu,\"sensors\":%s,\"status\":%s}",
                streaming_enabled ? "true" : "false",
                streaming_interval_ms,
                stream_sensors ? "true" : "false", 
                stream_status ? "true" : "false");
        format_json_response("stream", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
        return;
    }
    
    // Process based on first token
    if (strcasecmp(token, "on") == 0) {
        if (streaming_enabled) {
            cancel_repeating_timer(&streaming_timer);
        }
        
        add_repeating_timer_ms(-streaming_interval_ms, streaming_timer_callback, NULL, &streaming_timer);
        streaming_enabled = true;
        valid_command = true;
    }
    else if (strcasecmp(token, "off") == 0) {
        if (streaming_enabled) {
            cancel_repeating_timer(&streaming_timer);
            streaming_enabled = false;
        }
        valid_command = true;
    }

    else if (strcasecmp(token, "interval") == 0) {
        // Get interval parameter
        token = strtok_r(rest, " ", &rest);
        if (token) {
            int interval = atoi(token);
            if (interval >= 10) { // Minimum 10ms (100Hz)
                streaming_interval_ms = interval;
                
                // Restart timer if streaming is enabled
                if (streaming_enabled) {
                    cancel_repeating_timer(&streaming_timer);
                    add_repeating_timer_ms(-streaming_interval_ms, streaming_timer_callback, NULL, &streaming_timer);
                }
                valid_command = true;
            }
        }
    }
    else if (strcasecmp(token, "sensors") == 0) {
        // Get enable/disable parameter
        token = strtok_r(rest, " ", &rest);
        if (token) {
            if (strcasecmp(token, "on") == 0) {
                stream_sensors = true;
                valid_command = true;
            }
            else if (strcasecmp(token, "off") == 0) {
                stream_sensors = false;
                valid_command = true;
            }
        }
    }
    else if (strcasecmp(token, "status") == 0) {
        // Get enable/disable parameter
        token = strtok_r(rest, " ", &rest);
        if (token) {
            if (strcasecmp(token, "on") == 0) {
                stream_status = true;
                valid_command = true;
            }
            else if (strcasecmp(token, "off") == 0) {
                stream_status = false;
                valid_command = true;
            }
        }
    }
    
    // Send response
    if (valid_command) {
        sprintf(json_buffer, "{\"enabled\":%s,\"interval\":%lu,\"sensors\":%s,\"status\":%s}",
                streaming_enabled ? "true" : "false",
                streaming_interval_ms,
                stream_sensors ? "true" : "false", 
                stream_status ? "true" : "false");
        format_json_response("stream", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
    }
    else {
        sprintf(json_buffer, "{\"status\":\"error\",\"message\":\"Invalid stream parameters\"}");
        format_json_response("error", json_buffer, json_buffer, JSON_BUFFER_SIZE);
        printf("%s\n", json_buffer);
    }
}

/*!
* @brief Timer callback for periodic data streaming.
* @param t Timer structure pointer.
* @return true to continue timer, false to stop.
*/
static bool streaming_timer_callback(struct repeating_timer *t) {
    (void)t; // Unused parameter
    
    stream_counter++;
    
    // Send sensor data if enabled
    if (stream_sensors) {
        stream_sensor_data();
    }
    
    // Send status data if enabled (but less frequently)
    if (stream_status && (stream_counter % 5 == 0)) {
        stream_status_data();
    }
    
    return true; // Continue timer
}
