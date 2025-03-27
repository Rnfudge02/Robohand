/*!
 * \file Robohand_usb.c
 * \brief USB interface for robotic hand control.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

//Include headers
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "Robohand.h"

#include "pico/stdlib.h"

#define STATUS_UPDATE_MS 1000               ///< Millisecond timer for status retrieval, soon to be deprecated
#define CMD_BUFFER_SIZE 64                  ///< Size of the command buffer, exceeding this will cause overflow

/*!
 * \brief Handles commands over USB.
 * \param cmd Pointer to command array for reading.
 * \pre The cmd array should be null-terminated.
 */
void process_command(const char* cmd);

/*!
 * \brief Core 0 main loop routine - USB Variation.
 */
int main(void) {

    //Configure RGB light for debugging purposes
    init_rgb();
    rgb_set_color(255, 0, 0); //Red
    rgb_set_brightness(1.0); //100% brightness
    
    stdio_init_all();
    while(!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("STDIO initialized\r\nRobohand-Pico: program developed by Robert Fudge 2025.\r\n");

    rgb_set_color(255, 255, 0); //Yellow

    //Set up LED for heartbeat
    gpio_init(ROBOHAND_LED_PIN);
    gpio_set_dir(ROBOHAND_LED_PIN, GPIO_OUT);

    //Initialize the system, sync with Core 1
    init_robohand_system();

    rgb_set_color(0, 255, 0); //Green

    printf("System initialized!, both cores online, entering main loop.\r\n");

    printf("\r\n> ");

    //Routine main loop
    while(true) {
        sys_status.core0_loops++;
        
        //Non-blocking command input
        int c = getchar_timeout_us(0);
        if(c != PICO_ERROR_TIMEOUT) {
            static char cmd_buf[64];
            static int buf_idx = 0;

            printf("%c", c);

            //If either newline or cr is detected, null-terminate the array, and send to processing
            if(c == '\n' || c == '\r') {
                cmd_buf[buf_idx] = '\0';
                process_command(cmd_buf);
                buf_idx = 0;
                printf("\r\n> ");
            }

            //Add character to buffer
            else if(buf_idx < sizeof(cmd_buf)-1) {
                cmd_buf[buf_idx++] = c;
            }
        }

        tight_loop_contents();
    }
}

//! Command processing function
void process_command(const char* cmd) {
    int servo, pos, duration;
    sensor_data raw_data;
    sensor_data_physical converted;

    //If the sent command was 'help' or "HELP"
    if (strcasecmp(cmd, "HELP") == 0) {
        printf("Commands:\r\n"
               "SERVO [0-4] [500-2500] [duration_ms]\r\n"
               "VIEW - Show sensor data\r\n"
               "STATUS - System status\r\n"
               "HELP - This message\r\n");
    }
    
    //If the sent command wanted servo actuation
    else if ((sscanf(cmd, "SERVO %d %d %d", &servo, &pos, &duration) >= 2) ||
    (sscanf(cmd, "servo %d %d %d", &servo, &pos, &duration) >= 2)) {
        if (servo < 0 || servo >= NUM_SERVOS) {
            printf("Invalid servo\r\n");
            return;
        }

        if (duration <= 0) duration = 1000; //Default duration
        actuate_servo(servo, pos, duration);
        printf("Moving servo %d to %dμs in %dms\r\n", servo, pos, duration);
    }

    //If the sent command wishes to view sensor output
    else if (strcasecmp(cmd, "VIEW") == 0) {
        if (get_sensor_data(&raw_data) && convert_sensor_data(&raw_data, &converted)) {
            printf("Accel: %.4fg, %.2fg, %.4fg\r\n", 
                converted.accel[0], converted.accel[1], converted.accel[2]);
            printf("Gyro: %.4f°/s, %.2f°/s, %.4f°/s\r\n", 
                converted.gyro[0], converted.gyro[1], converted.gyro[2]);
            printf("Mag: %.4fμT, %.4fμT, %.4fμT\r\n", 
                converted.mag[0], converted.mag[1], converted.mag[2]);
            printf("Finger sensors: %.2fV, %.2fV, %.2fV, %.2fV, %.2fV\r\n",
                converted.adc_values[0], converted.adc_values[1],
                converted.adc_values[2], converted.adc_values[3],
                converted.adc_values[4]);
        }
        
        else {
            printf("Sensor data unavailable\r\n");
        }
    }

    //If the sent command wishes to view the systems status
    else if (strcasecmp(cmd, "STATUS") == 0) {
        system_status status;
        sensor_data sensors;
        sensor_data_physical converted;
        servo_motion_profile profile;

        get_system_status(&status);

        //Print load statistics
        printf("=== Robohand System Status ===\r\n");
        printf("Cores: 0(%s) | 1(%s)\r\n", 
            status.system_ok ? "OK" : "ERR",
            multicore_fifo_rvalid() ? "OK" : "ERR");

        printf("Load: Core0 %luHz | Core1 %luHz\r\n",
            status.core0_loops / (STATUS_UPDATE_MS/1000),
            status.core1_loops / (STATUS_UPDATE_MS/1000));

        //Print sensor data
        if(get_sensor_data(&sensors) && convert_sensor_data(&sensors, &converted)) {
            printf("\r\nSensors:\r\n");
            printf(" Accel: X%.2fg Y%.2fg Z%.2fg\r\n", 
                converted.accel[0], converted.accel[1], converted.accel[2]);
            printf(" Gyro:  X%.2f°/s Y%.2f°/s Z%.2f°/s\r\n",
                converted.gyro[0], converted.gyro[1], converted.gyro[2]);
        }

        //Retrieve and print servo data
        printf("\r\nServos:\r\n");
        for(int i=0; i<NUM_SERVOS; i++) {
            get_servo_status(i, &profile);
            printf(" %d: %4dμs %s\r\n", i, profile.current_pw, profile.is_moving ? "MOVING" : "READY");
        }
    }

    //Unrecognized command
    else {
        printf("Unknown command\r\n");
    }
}