/*!
 * \file Robohand_timing.h
 * \brief Timing functionality for performance evaluation.
 * \details Only fucntional when the debugging flag is > 0.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_TIMING_H
#define ROBOHAND_TIMING_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "Robohand_common.h"

#if DEBUG > 0
    uint32_t timing_start(void);
    void timing_end(const char* func_name, uint32_t start_time);
    void timing_print_report(void);
    void timing_init(void);
#else
    static inline uint32_t timing_start(void) {
        return 0;
    }

    static inline void timing_end(const char* func_name, uint32_t start_time) { 
        (void)func_name; (void)start_time;
    }

    static inline void timing_print_report(void) {

    }

    static inline void timing_init(void) {

    }

#endif

#ifdef __cplusplus
}
#endif

#endif