/**
 * This file contains platform-dependent functions that must be implemented
 * implemented on a per platform basis (some may be omitted depending on which
 * lib functions are used.
 */
#ifndef LIBPEKIN_LIBPEKIN_HAL_H_
#define LIBPEKIN_LIBPEKIN_HAL_H_

#include "libpekin.h"
#include <cstdint>

#ifndef LP_ASSERT
#pragma message("You may want to define LP_ASSERT here to defer to your platform specific assert macro/function")
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
#ifdef USE_64_BIT_TIMERS
uint64_t getMillis();
#else
uint32_t getMillis();
#endif

} // namespace libp

#endif /* LIBPEKIN_LIBPEKIN_HAL_H_ */
