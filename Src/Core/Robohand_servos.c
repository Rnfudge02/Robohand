#include "Robohand.h"
#include "Robohand_struct.h"
#include "Robohand_servos.h"

#include <stdio.h>
#include <string.h>

#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Dedicated hardware mutexes
mutex_t servo_mutex;                                         ///< Mutex protecting Servo access
const uint SERVO_PINS[NUM_SERVOS] = {11, 12, 13, 14, 15};           ///< GPIO pins to use for mechanical actuation
static const float PWM_US_TO_LEVEL = (39062.0f / 20000.0f);         ///< Constant value used for PWM conversion for servos
static absolute_time_t prev_update_time;                            ///< Used to track the time since last update

static void update_servo_position(int servo_index, uint32_t elapsed, uint32_t duration_us);

 /** @defgroup servo_control_impl Servo Control Implementation
  *  @brief Implementation of servo control functions.
  *  @{
  */
 
 /*!
  * @brief Actuates a servo to a specified position over a given duration.
  * @param servo Servo index (0 to NUM_SERVOS-1).
  * @param pulse_width Target pulse width in microseconds (500-2500µs).
  * @param duration_ms Movement duration in milliseconds.
  */
 void actuate_servo(uint8_t servo, uint16_t pulse_width, uint16_t duration_ms) {
    if (!HAS_SERVOS) return;
    
    if (servo >= NUM_SERVOS) {
        if (DEBUG > 0) {
            printf("Error: Invalid servo index %d\r\n", servo);
        }
        return;
    }
    
    // Validate inputs with proper bounds
    pulse_width = constrain_u16(pulse_width, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    duration_ms = constrain_u16(duration_ms, 100, MAX_MOVE_DURATION_MS); // Enforce minimum duration
    
    // Pack command - ensure bit shifting is correct
    uint32_t command = ((uint32_t)servo << 28) | 
                      ((uint32_t)(pulse_width & 0xFFF) << 16) | 
                       (uint32_t)(duration_ms & 0xFFFF);
    
    // Send command to other core
    multicore_fifo_push_blocking(command);
}
 /*!
  * @brief Retrieves the current status of a servo.
  * @param[in] servo Servo index (0 to NUM_SERVOS-1).
  * @param[out] dest Pointer to servo_motion_profile to populate.
  * @return True if successfully acquired, false if error.
  * @note An error in this case could simply be that the mutexes could not be acquired
  */
 bool get_servo_status(uint8_t servo, servo_motion_profile* dest) {
    if (HAS_SERVOS) {
        if(servo >= NUM_SERVOS) {
            return false;
        }

        // If user hasn't allocated the array, do it
        if (dest == NULL) {
            init_servo_motion_profile_struct(dest, 0);
        }

        if(!mutex_enter_timeout_ms(&servo_profiles[servo].profile_mutex, 25)) { // 25ms
            memcpy(dest, &servo_profiles[servo], sizeof(servo_motion_profile));
            mutex_exit(&servo_profiles[servo].profile_mutex);
            return true;
        }
    }

    return false;
}

/*!
 * @brief Initializes PWM for servo control.
 * @details Configures PWM frequency to 50Hz (20ms period) for standard servos.
 * @post All servo pins are configured with correct PWM settings.
 */
void init_servo_pwm(void) {
    if (HAS_SERVOS) {
        for(int i = 0; i < NUM_SERVOS; i++) {
            gpio_set_function(SERVO_PINS[i], GPIO_FUNC_PWM);
            uint slice_num = pwm_gpio_to_slice_num(SERVO_PINS[i]);
            pwm_config config = pwm_get_default_config();
            pwm_config_set_clkdiv(&config, 64.0f);  // 125MHz / 64 = 1.953125MHz
            pwm_config_set_wrap(&config, 39062);    // 1.953125MHz / 39062 ≈ 50Hz
            pwm_init(slice_num, &config, true);
        }
    }
}

/*!
 * @brief Function that unpacks commands sent from other modules.
 * @details Reads from the FIFO and parses servo commands.
 * @param cmd The FIFO command to process.
 * @post The appropriate servo profile is updated.
 */
void handle_servo_commands(uint32_t cmd) {
    if (!HAS_SERVOS) return;
    
    // Command format: [4-bit servo index][12-bit pulse width][16-bit duration]
    uint8_t servo = (cmd >> 28) & 0x0F;         // Extract servo index (top 4 bits)
    uint16_t pulse_width = (cmd >> 16) & 0xFFF; // Extract pulse width (next 12 bits)
    uint16_t duration = cmd & 0xFFFF;           // Extract duration (lower 16 bits)
    
    // Validate servo index
    if (servo >= NUM_SERVOS) {
        if (DEBUG > 0) {
            printf("Error: Invalid servo index %d\r\n", servo);
        }
        return;
    }
    
    // Constrain pulse width and duration to valid ranges
    pulse_width = constrain_u16(pulse_width, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    duration = constrain_u16(duration, 100, MAX_MOVE_DURATION_MS);
    
    // Try to update the servo profile with a timeout
    if (mutex_enter_timeout_ms(&servo_profiles[servo].profile_mutex, 50)) {
        servo_profiles[servo].target_pw = pulse_width;
        servo_profiles[servo].duration_ms = duration;
        servo_profiles[servo].start_time = time_us_32();
        servo_profiles[servo].is_moving = true;
        mutex_exit(&servo_profiles[servo].profile_mutex);
        
        if (DEBUG > 1) {
            printf("Servo %d command: pw=%d, dur=%d\r\n", servo, pulse_width, duration);
        }
    } else if (DEBUG > 0) {
        printf("Error: Failed to acquire servo %d mutex\r\n", servo);
    }
}

/*!
 * @brief Update servo positions using motion profiles.
 * @details Calculates smooth transitions between current and target positions.
 * @post Servo PWM outputs are updated with new calculated positions.
 */
void update_servo_positions(void) {
    if (!HAS_SERVOS) return;
    
    absolute_time_t now = get_absolute_time();
    uint64_t dt_us = absolute_time_diff_us(prev_update_time, now);
    
    if (dt_us <= 0) {
        return;  // Prevent time travel or division by zero
    }
    
    prev_update_time = now;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Try to get mutex for this servo
        if (!mutex_try_enter(&servo_mutex, NULL)) {
            continue;  // Skip this servo if we can't get the mutex
        }
        
        // Skip non-moving servos
        if (!servo_profiles[i].is_moving) {
            mutex_exit(&servo_mutex);
            continue;
        }

        // Calculate movement progress
        uint32_t elapsed = time_us_32() - servo_profiles[i].start_time;
        uint32_t duration_us = servo_profiles[i].duration_ms * 1000;
        
        // Check if movement is complete
        if (elapsed >= duration_us) {
            // Movement complete, set final position
            servo_profiles[i].current_pw = servo_profiles[i].target_pw;
            servo_profiles[i].is_moving = false;
        } else {
            // Movement in progress, calculate new position
            update_servo_position(i, elapsed, duration_us);
        }

        // Update PWM (50Hz = 20ms period)
        uint16_t level = (uint16_t)(((float)servo_profiles[i].current_pw) * PWM_US_TO_LEVEL);
        pwm_set_gpio_level(SERVO_PINS[i], level);
        
        // Release the mutex
        mutex_exit(&servo_mutex);
    }
}

/*!
 * @brief Updates a single servo position.
 * @param servo_index The index of the servo to update.
 * @param elapsed Elapsed time in microseconds since movement started.
 * @param duration_us Total movement duration in microseconds.
 */
static void update_servo_position(int servo_index, uint32_t elapsed, uint32_t duration_us) {
    // Calculate progress as a float between 0.0 and 1.0
    float progress = (float)elapsed / (float)duration_us;
    
    // Apply S-curve for smooth acceleration and deceleration
    float eased_progress;
    
    if (progress < 0.5f) {
        // First half - accelerate
        eased_progress = 2.0f * progress * progress;
    } else {
        // Second half - decelerate
        float p = progress - 1.0f;
        eased_progress = 1.0f - 2.0f * p * p;
    }
    
    // Calculate new position
    int32_t delta = servo_profiles[servo_index].target_pw - 
                   servo_profiles[servo_index].current_pw;
                  
    uint16_t new_pw = servo_profiles[servo_index].current_pw + 
                     (uint16_t)((float)delta * eased_progress);
                     
    // Constrain within valid range
    servo_profiles[servo_index].current_pw = constrain_u16(new_pw, 
                                 SERVO_MIN_PULSE, 
                                 SERVO_MAX_PULSE);
}

/** @} */ // end of servo_control_impl
