/*!
 * \file Robohand_timing.c
 * \brief Timing functionality for performance evaluation.
 * \details Only fucntional when the debugging flag is > 0.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_timing.h"

#include <stdio.h>
#include <string.h>

#include "pico/mutex.h"

#if DEBUG > 0

// Timing statistics structure
typedef struct {
    const char* function_name;
    uint32_t call_count;
    uint32_t total_time_us;
    uint32_t min_time_us;
    uint32_t max_time_us;
} timing_stats_t;

#define MAX_TIMING_ENTRIES 50
static timing_stats_t timing_data[MAX_TIMING_ENTRIES];
static int timing_count = 0;
static mutex_t timing_mutex;

void timing_init(void) {
    mutex_init(&timing_mutex);
    memset(timing_data, 0, sizeof(timing_data));
    timing_count = 0;
}

uint32_t timing_start(void) {
    return time_us_32();
}

void timing_end(const char* func_name, uint32_t start_time) {
    uint32_t elapsed = time_us_32() - start_time;
    
    mutex_enter_blocking(&timing_mutex);
    
    // Look for existing entry
    int idx = -1;
    for (int i = 0; i < timing_count; i++) {
        if (strcmp(timing_data[i].function_name, func_name) == 0) {
            idx = i;
            break;
        }
    }
    
    // Create new entry if not found
    if (idx == -1) {
        if (timing_count < MAX_TIMING_ENTRIES) {
            idx = timing_count++;
            timing_data[idx].function_name = func_name;
            timing_data[idx].min_time_us = UINT32_MAX;
        } else {
            // No more space for new entries
            mutex_exit(&timing_mutex);
            return;
        }
    }
    
    // Update statistics
    timing_data[idx].call_count++;
    timing_data[idx].total_time_us += elapsed;
    
    if (elapsed < timing_data[idx].min_time_us)
        timing_data[idx].min_time_us = elapsed;
        
    if (elapsed > timing_data[idx].max_time_us)
        timing_data[idx].max_time_us = elapsed;
        
    mutex_exit(&timing_mutex);
}

void timing_print_report(void) {
    mutex_enter_blocking(&timing_mutex);
    
    printf("\n===== TIMING REPORT =====\n");
    printf("%-30s %10s %10s %10s %10s\n", 
        "Function", "Calls", "Total(us)", "Avg(us)", "Max(us)");
    printf("---------------------------------------------------------------\n");
    
    for (int i = 0; i < timing_count; i++) {
        float avg = (float)timing_data[i].total_time_us / 
            ((float) timing_data[i].call_count > 0 ? (float) timing_data[i].call_count : 1);
                    
        printf("%-30s %10lu %10lu %10.1f %10lu\n",
            timing_data[i].function_name,
            timing_data[i].call_count,
            timing_data[i].total_time_us,
            avg,
            timing_data[i].max_time_us);
    }
    
    printf("=========================\n");
    mutex_exit(&timing_mutex);
}

#endif // DEBUG > 0