/*!
* @file Robohand_usb.c
* @brief USB terminal interface for robotic hand control system.
* @details Provides an interactive terminal for monitoring and controlling the hand.
* @author Robert Fudge <rnfudge@mun.ca>
* @date 2025
* @copyright Apache 2.0 License
*/

#include "Robohand.h"
#include "Robohand_dma.h"
#include "Robohand_callbacks.h"
#include "Robohand_i2c.h"
#include "Robohand_interrupts.h"
#include "Robohand_struct.h"
#include "Robohand_servos.h"
#include "Robohand_rgb.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/multicore.h"
#include "hardware/watchdog.h"

#define CMD_BUFFER_SIZE 256
#define MAX_ARGS 10
#define TELEMETRY_INTERVAL_MS 1000  //Default telemetry update interval

//Function prototypes
static void process_command(char* cmd_buffer);
static void print_help(void);
static void handle_servo_command(int argc, char** argv);
static void handle_sensor_command(void);
static void handle_status_command(void);
static void handle_rgb_command(int argc, char** argv);
static void handle_blink_command(int argc, char** argv);
static void handle_core_debug(int argc, char** argv);
static void handle_debug_command(int argc, char** argv);
static bool timer_callback(struct repeating_timer *t);

static void handle_input_char(int c, char *cmd_buffer, int *cmd_pos);
static void handle_telemetry_command(int argc, char **argv);

//Global variables
static struct repeating_timer telemetry_timer;
static bool telemetry_enabled = false;
static uint32_t loop_counter = 0;
static absolute_time_t last_time;

/*!
* @brief Main entry point for the USB interface.
* @return Should never return in normal operation.
*/
static void handle_input_char(int c, char *cmd_buffer, int *cmd_pos) {
    if (c == '\r' || c == '\n') {
        //Process Enter key
        printf("\r\n");  // Echo newline
        
        //Null-terminate the command
        cmd_buffer[*cmd_pos] = '\0';
        
        //Process command if not empty
        if (*cmd_pos > 0) {
            process_command(cmd_buffer);
        }
        
        //Reset buffer
        *cmd_pos = 0;
        return;
    } 
    
    if (c == 127 || c == '\b') {
        //Process Backspace key
        if (*cmd_pos > 0) {
            //Move cursor back, print space, move cursor back again
            printf("\b \b");
            (*cmd_pos)--;
        }
        return;
    }
    
    if (c == 3) {
        //Process Ctrl+C - clear current line
        printf("^C\r\n");
        *cmd_pos = 0;
        return;
    }
    
    if (c >= 32 && c <= 126 && *cmd_pos < (CMD_BUFFER_SIZE - 1)) {
        //Process printable characters
        putchar(c);  //Echo character
        cmd_buffer[(*cmd_pos)++] = (char)c;
    }
}

/*!
* @brief Main entry point for the USB interface.
* @return Should never return in normal operation.
*/
int main() {
    //Initialize standard I/O
    stdio_init_all();
    
    //Small delay to allow for USB connection
    sleep_ms(2000);
    
    //Clear the terminal screen 
    printf("\033[2J\033[H");  //ANSI escape sequence to clear screen
    
    printf("RoboHand USB Interface v1.0\r\n");
    printf("Type 'help' for available commands\r\n\n");
    
    //Initialize the robotic hand system
    init_robohand_system();
    
    if (HAS_RGB) {
        init_rgb();
        rgb_set_color(0, 100, 0);  //Set initial color to green
    }
    
    char cmd_buffer[CMD_BUFFER_SIZE];
    int cmd_pos = 0;
    last_time = get_absolute_time();
    
    //Command processing loop
    while (true) {
		static bool printed = false;
        //Print prompt if buffer is empty
        if (cmd_pos == 0 && printed == false) {
            printf("robohand> ");
            fflush(stdout);
			printed = true;
        }
        
        //Check for incoming characters
        int c = getchar_timeout_us(10000);  //10ms timeout
        
        if (c != PICO_ERROR_TIMEOUT) {
            handle_input_char(c, cmd_buffer, &cmd_pos);
        }

		if (c == '\r' || c == '\n') {
			printed = false;
		}
        
        //Update system counters
        mutex_enter_blocking(&sys_status.status_mutex);
        sys_status.core0_loops++;
        mutex_exit(&sys_status.status_mutex);
        loop_counter++;
        
        //Calculate load every second
        if (absolute_time_diff_us(last_time, get_absolute_time()) >= 1000000) {
            last_time = get_absolute_time();
            loop_counter = 0;
        }
    }
}

/*!
* @brief Process a command string from the terminal.
* @param cmd_buffer The null-terminated command string.
*/
static void process_command(char *cmd_buffer) {
    //Skip leading whitespace
    while (isspace(*cmd_buffer)) cmd_buffer++;
    
    //Check for empty command
    if (*cmd_buffer == '\0') return;
    
    //Split command into tokens
    char *argv[MAX_ARGS];
    int argc = 0;
    char *saveptr; //Save pointer for strtok_r
    
    char *token = strtok_r(cmd_buffer, " \t", &saveptr);
    while (token != NULL && argc < MAX_ARGS) {
        argv[argc++] = token;
        token = strtok_r(NULL, " \t", &saveptr);
    }
    
    //Exit early if no arguments
    if (argc == 0) return;
    
    //Handle commands with function calls to reduce nesting
    if (strcmp(argv[0], "help") == 0) {
        print_help();
        return;
    }
    
    if (strcmp(argv[0], "servo") == 0) {
        handle_servo_command(argc, argv);
        return;
    }
    
    if (strcmp(argv[0], "sensors") == 0) {
        handle_sensor_command();
        return;
    }
    
    if (strcmp(argv[0], "status") == 0) {
        handle_status_command();
        return;
    }
    
    if (strcmp(argv[0], "rgb") == 0) {
        handle_rgb_command(argc, argv);
        return;
    }
    
    if (strcmp(argv[0], "blink") == 0) {
        handle_blink_command(argc, argv);
        return;
    }

	if (strcmp(argv[0], "coredebug") == 0) {
		handle_core_debug(argc, argv);
		return;
	}

	if (strcmp(argv[0], "debug") == 0) {
		handle_debug_command(argc, argv);
		return;
	}
    
    if (strcmp(argv[0], "telemetry") == 0) {
        handle_telemetry_command(argc, argv);
        return;
    }
    
    if (strcmp(argv[0], "reset") == 0) {
        printf("Resetting system...\r\n");
        watchdog_enable(1, false);  //Enable watchdog with 1ms timeout
        while (1) tight_loop_contents();  //Wait for watchdog to reset
    }
    
    //If we get here, command wasn't recognized
    printf("Unknown command: %s\r\n", argv[0]);
    printf("Type 'help' for available commands\r\n");
}

/*!
* @brief Main entry point for the USB interface.
* @return Should never return in normal operation.
*/
static void handle_telemetry_command(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: telemetry <on|off> [interval_ms]\r\n");
        return;
    }
    
    if (strcmp(argv[1], "on") == 0) {
        int interval = TELEMETRY_INTERVAL_MS;
        if (argc > 2) {
            interval = atoi(argv[2]);
            if (interval < 100) interval = 100;  //Minimum 100ms
        }
        
        if (telemetry_enabled) {
            cancel_repeating_timer(&telemetry_timer);
        }
        
        add_repeating_timer_ms(-interval, timer_callback, NULL, &telemetry_timer);
        telemetry_enabled = true;
        printf("Telemetry enabled (%dms interval)\r\n", interval);
        return;
    }
    
    if (strcmp(argv[1], "off") == 0) {
        if (telemetry_enabled) {
            cancel_repeating_timer(&telemetry_timer);
            telemetry_enabled = false;
            printf("Telemetry disabled\r\n");
        }
        return;
    }
    
    printf("Usage: telemetry <on|off> [interval_ms]\r\n");
}

/*!
* @brief Print help information.
*/
static void print_help(void) {
	printf("Available commands:\r\n");
	printf("  help                     - Show this help message\r\n");
	printf("  servo <id> <pos> <time>  - Move servo (id:0-4, pos:500-2500µs, time:ms)\r\n");
	printf("  sensors                  - Show current sensor readings\r\n");
	printf("  status                   - Show system status\r\n");
	printf("  rgb <r> <g> <b>          - Set RGB LED color (0-255)\r\n");
	printf("  blink <on|off> <time>    - Control LED blinking (time:ms)\r\n");
	printf("  telemetry <on|off> [ms]  - Enable/disable periodic updates\r\n");
	printf("  reset                    - Reset the system\r\n");
}

/*!
* @brief Handle servo movement command.
* @param argc Argument count.
* @param argv Argument values.
*/
static void handle_servo_command(int argc, char **argv) {
	if (argc < 4) {
		printf("Usage: servo <id> <position> <time>\r\n");
		printf("  id: 0-%d (servo index)\r\n", NUM_SERVOS-1);
		printf("  position: %d-%d (pulse width in µs)\r\n", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
		printf("  time: milliseconds for movement\r\n");
		return;
	}
	
	if (!HAS_SERVOS) {
		printf("Servo support is disabled in configuration\r\n");
		return;
	}
	
	uint8_t servo = (uint8_t) atoi(argv[1]);
	uint16_t position = (uint16_t) atoi(argv[2]);
	uint16_t duration = (uint16_t) atoi(argv[3]);
	
	if (servo >= NUM_SERVOS) {
		printf("Error: Servo index must be 0-%d\r\n", NUM_SERVOS-1);
		return;
	}
	
	//Constrain values
	position = constrain_u16(position, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
	duration = constrain_u16(duration, 100, MAX_MOVE_DURATION_MS);
	
	printf("Moving servo %d to position %d over %d ms\r\n", servo, position, duration);
	actuate_servo(servo, position, duration);
}

/*!
* @brief Handle sensor reading command.
*/
static void handle_sensor_command(void) {
	sensor_data sensor_values;
	sensor_data_physical physical_values;
	
	if (get_sensor_data(&sensor_values)) {
		convert_sensor_data(&sensor_values, &physical_values);
		
		printf("Sensor Readings:\r\n");
		printf("Accelerometer (g):  X: %.2f  Y: %.2f  Z: %.2f\r\n", 
			physical_values.accel[0], physical_values.accel[1], physical_values.accel[2]);
		
		printf("Gyroscope (°/s):    X: %.2f  Y: %.2f  Z: %.2f\r\n", 
			physical_values.gyro[0], physical_values.gyro[1], physical_values.gyro[2]);
		
		printf("Magnetometer (µT):  X: %.2f  Y: %.2f  Z: %.2f\r\n", 
			physical_values.mag[0], physical_values.mag[1], physical_values.mag[2]);
		
		printf("ADC Voltages (V):   ");
		for (int i = 0; i < 5; i++) {
			printf("CH%d: %.3f  ", i, physical_values.adc_values[i]);
		}
		printf("\r\n");
		
		printf("Altitude: %.2f m\r\n", physical_values.altitude);
	} else {
		printf("Error: Failed to read sensor data\r\n");
	}
}

/*!
* @brief Handle system status command.
*/
static void handle_status_command(void) {
    system_status status;
    get_system_status(&status);
    
    printf("System Status:\r\n");
    printf("Core 0 Load: %.2f loops/ms\r\n", status.core0_load);
    printf("Core 1 Load: %.2f loops/ms\r\n", status.core1_load);
    printf("System OK: %s\r\n", status.system_ok ? "Yes" : "No");
    printf("Emergency Stop: %s\r\n", status.emergency_stop ? "Active" : "Inactive");
    
    //Convert to seconds for more readable output
    float seconds_since_watchdog = (float) (time_us_32() - status.last_watchdog) / 1000000.0f;
    printf("Last Watchdog: %.1f seconds ago\r\n", seconds_since_watchdog);
    
    //Add configuration info
    printf("\nSystem Configuration:\r\n");
    printf("USE_INTERRUPTS: %s\r\n", USE_INTERRUPTS ? "Yes" : "No");
    printf("USE_DMA: %s\r\n", USE_DMA ? "Yes" : "No");
    printf("USE_CALLBACKS: %s\r\n", USE_CALLBACKS ? "Yes" : "No");
    
    printf("\nHardware Configuration:\r\n");
    printf("HAS_ADS1115: %s\r\n", HAS_ADS1115 ? "Yes" : "No");
    printf("HAS_BME280: %s\r\n", HAS_BME280 ? "Yes" : "No");
    printf("HAS_QMC5883L: %s\r\n", HAS_QMC5883L ? "Yes" : "No");
    printf("HAS_MPU6050: %s\r\n", HAS_MPU6050 ? "Yes" : "No");
    printf("HAS_PI_ADC: %s\r\n", HAS_PI_ADC ? "Yes" : "No");
    printf("HAS_RGB: %s\r\n", HAS_RGB ? "Yes" : "No");
    printf("HAS_SERVOS: %s\r\n", HAS_SERVOS ? "Yes" : "No");
    
    //Show servo status if enabled
    if (HAS_SERVOS) {
        printf("\nServo Status:\r\n");
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            servo_motion_profile profile;
            if (get_servo_status(i, &profile)) {
                printf("Servo %d: Pos=%d µs  Target=%d µs  Moving=%s\r\n",
                       i, profile.current_pw, profile.target_pw,
                       profile.is_moving ? "Yes" : "No");
            } else {
                printf("Servo %d: Status unavailable\r\n", i);
            }
        }
    }
}

/*!
* @brief Handle RGB LED color command.
* @param argc Argument count.
* @param argv Argument values.
*/
static void handle_rgb_command(int argc, char **argv) {
	if (!HAS_RGB) {
		printf("RGB LED support is disabled in configuration\r\n");
		return;
	}
	
	if (argc < 4) {
		printf("Usage: rgb <r> <g> <b>\r\n");
		printf("  r,g,b: 0-255 (color values)\r\n");
		return;
	}
	
	uint8_t r = (uint8_t) abs(atoi(argv[1]));
	uint8_t g = (uint8_t) abs(atoi(argv[2]));
	uint8_t b = (uint8_t) abs(atoi(argv[3]));
	
	rgb_set_color(r, g, b);
	printf("RGB color set to (%d, %d, %d)\r\n", r, g, b);
}

/*!
* @brief Handle LED blinking command.
* @param argc Argument count.
* @param argv Argument values.
*/
static void handle_blink_command(int argc, char **argv) {
	if (!HAS_RGB) {
		printf("RGB LED support is disabled in configuration\r\n");
		return;
	}
	
	if (argc < 2) {
		printf("Usage: blink <on|off> [interval_ms]\r\n");
		return;
	}
	
	if (strcmp(argv[1], "on") == 0) {
		uint32_t interval = 500;  //Default 500ms
		if (argc > 2) {
			interval = (uint32_t) abs(atoi(argv[2]));
			if (interval < 50) interval = 50;  //Minimum 50ms
		}
		
		rgb_blink(true, interval);
		printf("LED blinking enabled (%lu ms interval)\r\n", interval);
	}
	else if (strcmp(argv[1], "off") == 0) {
		rgb_blink(false, 0);
		printf("LED blinking disabled\r\n");
	}
	else {
		printf("Usage: blink <on|off> [interval_ms]\r\n");
	}
}
static void handle_debug_command(int argc, char** argv) {
	(void) argc;
	(void) argv;
	
	get_debug_info();
}

static void handle_core_debug(int argc, char** argv) {
	(void) argc;
	(void) argv;
	
	printf("Core Debug Information:\r\n");
		
	//Force synchronization between cores
	printf("Sending ping to Core 1...\r\n");
		
	uint32_t start_time = time_us_32();
		
	//Send ping signal to core 1
	multicore_fifo_push_blocking(0xDEAD);
		
	//Wait for response with timeout
	bool response = false;
	for (int i = 0; i < 10; i++) {  // Try for ~1 second
		if (multicore_fifo_rvalid()) {
			uint32_t response_val = multicore_fifo_pop_blocking();
			printf("Core 1 responded with: 0x%08lX\r\n", response_val);
			printf("Round-trip time: %lu microseconds\r\n", time_us_32() - start_time);
			response = true;
			break;
		}
		sleep_ms(100);
	}
		
	if (!response) {
		printf("ERROR: Core 1 did not respond within timeout!\r\n");
		printf("This suggests Core 1 might be locked up or not running properly.\r\n");
	}
}


/*!
* @brief Timer callback for telemetry updates.
* @param t Timer structure.
* @return Always returns true to continue timer.
*/
static bool timer_callback(struct repeating_timer *t) {
	(void)t;  // Unused parameter
	
	// Clear line
	printf("\033[2K\r");  // ANSI escape to clear line
	
	// Show condensed status
	system_status status;
	get_system_status(&status);
	
	printf("C0:%.1f C1:%.1f | ", status.core0_load, status.core1_load);
	
	// Show servo positions
	if (HAS_SERVOS) {
		printf("Servos: ");
		for (int i = 0; i < NUM_SERVOS; i++) {
			servo_motion_profile profile;
			if (get_servo_status((uint8_t) i, &profile)) {
				printf("%d:%d%s ", i, profile.current_pw, 
					profile.is_moving ? "*" : "");
			} else {
				printf("%d:? ", i);
			}
		}
	}
	
	// Add sensor data if available
	sensor_data sensor_values;
	sensor_data_physical physical_values;
	
	if (get_sensor_data(&sensor_values) && 
		convert_sensor_data(&sensor_values, &physical_values)) {
		printf("| ACC:%.1f,%.1f,%.1f", 
			physical_values.accel[0], 
			physical_values.accel[1], 
			physical_values.accel[2]);
	}
	
	// Keep cursor at start of line for next update
	fflush(stdout);
	return true;
}