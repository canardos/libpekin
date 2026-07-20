/*
 * Your program must contain:
 *
 * 1) A `libpekin_config.h` file in the include path with optional macro
 *    definitions:
 *    - see `libpekin_config_example.h`
 *
 * 2) Implementations of the functions declared in this file:
 *    - some may be omitted depending on which lib functions are used.
 */

#ifndef LIBPEKIN_LIBPEKIN_H_
#define LIBPEKIN_LIBPEKIN_H_

#ifdef __has_include
    #if __has_include(<libpekin_config.h>)
        #include <libpekin_config.h>
    #else
        #warning "Application needs to define a 'libpekin_config.h' header. See libpekin.h for details. Defaulting to 'libpekin_config_default.h'"
        #include "libpekin_config_default.h"
    #endif
#else
    // Application needs to define a 'libpekin_config.h' header
    #include <libpekin_config.h>
#endif

#include <cstdint>

#ifndef LP_ASSERT
#pragma message("You probably want to define LP_ASSERT in your 'libpekin_config.h' to defer to your platform-specific assert macro/function. Using assert from 'cassert' header.")
#include <cassert>
#define LP_ASSERT(condition) assert(condition)
#endif


namespace libp {

/**
 * @param millis delay in milliseconds
 */
void delayMs(uint32_t millis);

/**
 * @param micros delay in microseconds
 */
void delayUs(uint32_t micros);

/**
 * @param nanos delay in nanoseconds
 */
void delayNs(uint32_t nanos);


#ifndef get_millis_ret_t
#define get_millis_ret_t uint32_t
#endif

/**
 * Returns the number of milliseconds since the timers were initialized or last
 * overflowed.
 *
 * Refer to the the specific implementation for limits.
 *
 * STM32F1 for example use a 32-bit counter and will overflow every ~49 days.
 *
 * @return
 */
get_millis_ret_t getMillis();

} // namespace libp


#endif /* LIBPEKIN_LIBPEKIN_H_ */
