#include "Robohand_rgb.h"
#include "Robohand_struct.h"

#include <math.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "pico/mutex.h"

//Gamma correction table (gamma = 2.8) for brightness smoothing
const uint8_t gamma_table[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
    2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  11,  11,
    12,  12,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  20,
    20,  21,  22,  22,  23,  24,  24,  25,  26,  27,  27,  28,  29,  30,  31,  31,
    32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
    49,  50,  51,  52,  53,  55,  56,  57,  59,  60,  61,  63,  64,  65,  67,  68,
    70,  71,  73,  74,  76,  78,  79,  81,  83,  84,  86,  88,  90,  91,  93,  95,
    97,  99,  101, 103, 105, 107, 109, 111, 113, 115, 117, 120, 122, 124, 126, 129,
    131, 134, 136, 138, 141, 143, 146, 148, 151, 154, 156, 159, 162, 165, 167, 170,
    173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 204, 207, 210, 213, 217, 220,
    224, 227, 231, 234, 238, 241, 245, 249, 252, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};
static struct repeating_timer blink_timer;                          ///< Timer for triggering the blink callback

static bool blink_callback(struct repeating_timer *t);

/** @defgroup rgb_control_impl RGB LED Control Implementation
 *  @brief Implementation of RGB LED control functions.
 *  @{
 */

/*!
 * @brief Initializes the RGB LED subsystem.
 * @details Configures PWM hardware and initializes mutexes.
 */
void init_rgb(void) {
    if (HAS_RGB) {
        if (DEBUG > 0) {
            printf("Initializing common cathode RGB LED on pins R:%d G:%d B:%d\r\n", 
                  RGB_RED_PIN, RGB_GREEN_PIN, RGB_BLUE_PIN);
        }
        
        // Initialize pins
        gpio_init(RGB_RED_PIN);
        gpio_init(RGB_GREEN_PIN);
        gpio_init(RGB_BLUE_PIN);
        
        // Set to output mode
        gpio_set_dir(RGB_RED_PIN, GPIO_OUT);
        gpio_set_dir(RGB_GREEN_PIN, GPIO_OUT);
        gpio_set_dir(RGB_BLUE_PIN, GPIO_OUT);
        
        // Initialize PWM
        gpio_set_function(RGB_RED_PIN, GPIO_FUNC_PWM);
        gpio_set_function(RGB_GREEN_PIN, GPIO_FUNC_PWM);
        gpio_set_function(RGB_BLUE_PIN, GPIO_FUNC_PWM);

        // Initialize RGB structure with specific settings
        init_rgb_state_struct(&rgb_conf);
        
        // Override the pwm_wrap with a smaller value for better resolution
        rgb_conf.pwm_wrap = 255;
        
        // Configure PWM with 8-bit resolution
        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, rgb_conf.pwm_wrap);
        // Use a small clock divider for brighter output
        pwm_config_set_clkdiv(&config, 1.0f);
        
        // Get the slices for each pin
        uint slices[] = {
            pwm_gpio_to_slice_num(RGB_RED_PIN),
            pwm_gpio_to_slice_num(RGB_GREEN_PIN),
            pwm_gpio_to_slice_num(RGB_BLUE_PIN)
        };
        
        // Initialize the PWM pins
        for(int i = 0; i < 3; i++) {
            pwm_init(slices[i], &config, true);
        }
        
        // Set maximum brightness
        rgb_conf.current_brightness = 1.0f;
        
        // Turn the LED on with test colors at full brightness
        rgb_set_color(255, 0, 0);  // Full red
        
        if (DEBUG > 0) {
            printf("RGB LED initialization complete\r\n");
        }
    }
}
 
/*!
 * @brief Sets the RGB LED color.
 * @param[in] r Red component (0-255).
 * @param[in] g Green component (0-255).
 * @param[in] b Blue component (0-255).
 */
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    if (HAS_RGB) {
        if (DEBUG > 0) {
            printf("Setting RGB color to (%d, %d, %d)\r\n", r, g, b);
        }
        
        mutex_enter_blocking(&rgb_conf.rgb_mutex);
        // Store original values
        rgb_conf.current_r = r;
        rgb_conf.current_g = g;
        rgb_conf.current_b = b;
        
        // Skip gamma correction temporarily for testing
        // Just apply brightness
        uint16_t red = (uint16_t)(r * rgb_conf.current_brightness);
        uint16_t green = (uint16_t)(g * rgb_conf.current_brightness);
        uint16_t blue = (uint16_t)(b * rgb_conf.current_brightness);
        mutex_exit(&rgb_conf.rgb_mutex);
        
        // Set PWM levels directly
        mutex_enter_blocking(&rgb_conf.pwm_mutex);
        pwm_set_gpio_level(RGB_RED_PIN, red);
        pwm_set_gpio_level(RGB_GREEN_PIN, green);
        pwm_set_gpio_level(RGB_BLUE_PIN, blue);
        mutex_exit(&rgb_conf.pwm_mutex);
        
        if (DEBUG > 0) {
            printf("PWM levels set to R:%d G:%d B:%d\r\n", red, green, blue);
        }
    }
}

/*!
 * @brief Sets the brightness of RGB LED.
 * @param brightness Brightness level (0.0-1.0).
 */
void rgb_set_brightness(float brightness) {
    if (HAS_RGB) {
        mutex_enter_blocking(&rgb_conf.rgb_mutex);

        rgb_conf.current_brightness = fmaxf(0.0f, fminf(1.0f, brightness));

        // Apply brightness and gamma correction
        uint16_t red = gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)];
        uint16_t green = gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)];
        uint16_t blue = gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)];

        mutex_exit(&rgb_conf.rgb_mutex);

        // Set PWM levels
        mutex_enter_blocking(&rgb_conf.pwm_mutex);
        pwm_set_gpio_level(RGB_RED_PIN, red);
        pwm_set_gpio_level(RGB_GREEN_PIN, green);
        pwm_set_gpio_level(RGB_BLUE_PIN, blue);
        mutex_exit(&rgb_conf.pwm_mutex);
    }
}

/*!
 * @brief Configures the RGB to blink at a specified interval.
 * @param enable Enable or disable blinking.
 * @param interval_ms Blink interval in milliseconds.
 */
void rgb_blink(bool enable, uint32_t interval_ms) {
    if (!HAS_RGB) return;
    
    mutex_enter_blocking(&rgb_conf.rgb_mutex);
    
    // If we're changing state, safely handle the timer
    if (rgb_conf.blink_active != enable) {
        if (rgb_conf.blink_active) {
            // Stop the timer first
            bool timer_cancelled = cancel_repeating_timer(&blink_timer);
            if (!timer_cancelled && DEBUG > 0) {
                printf("Warning: Failed to cancel blink timer\r\n");
            }
            rgb_conf.blink_active = false;
            
            // Restore color when disabling
            rgb_set_color(rgb_conf.current_r, rgb_conf.current_g, rgb_conf.current_b);
        }
        
        // Starting a new timer
        if (enable) {
            rgb_conf.blink_interval = interval_ms;
            rgb_conf.blink_state = true; // Start in the ON state
            rgb_conf.blink_active = add_repeating_timer_ms(
                -(int32_t)interval_ms, 
                &blink_callback, 
                NULL, 
                &blink_timer
            );
            
            if (!rgb_conf.blink_active && DEBUG > 0) {
                printf("Error: Failed to start blink timer\r\n");
            }
        }
    } 
    // Just updating the interval of an active timer
    else if (enable && rgb_conf.blink_interval != interval_ms) {
        cancel_repeating_timer(&blink_timer);
        rgb_conf.blink_interval = interval_ms;
        rgb_conf.blink_active = add_repeating_timer_ms(
            -(int32_t)interval_ms, 
            &blink_callback, 
            NULL, 
            &blink_timer
        );
    }
    
    mutex_exit(&rgb_conf.rgb_mutex);
}

/*!
 * @brief Callback allowing for toggling of RGB functionality.
 * @param t Pointer to the repeating timer structure.
 * @return Always returns true to continue the timer.
 */
static bool blink_callback(struct repeating_timer *t) {
    (void)t;

    if (HAS_RGB && mutex_try_enter(&rgb_conf.rgb_mutex, NULL) && rgb_conf.blink_active) {
        rgb_conf.blink_state = !rgb_conf.blink_state;
                
        if(rgb_conf.blink_state) {
            // Restore original color
            pwm_set_gpio_level(RGB_RED_PIN, gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)]);
            pwm_set_gpio_level(RGB_GREEN_PIN, gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)]);
            pwm_set_gpio_level(RGB_BLUE_PIN, gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)]);
        }
            
        else {
            // Turn off LEDs
            pwm_set_gpio_level(RGB_RED_PIN, 0);
            pwm_set_gpio_level(RGB_GREEN_PIN, 0);
            pwm_set_gpio_level(RGB_BLUE_PIN, 0);
        }

        mutex_exit(&rgb_conf.rgb_mutex);
    }
    
    return true;
}

/** @} */ // end of rgb_control_impl