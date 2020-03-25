/**
 * This file contains platform-dependent functions that must be implemented
 * on a per platform basis.
 */
#ifndef LIB_LIBPEKIN_LIBPEKIN_HAL_H_
#define LIB_LIBPEKIN_LIBPEKIN_HAL_H_

#include <cstdint>
#include "libpekin.h"

namespace Libp {

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

} // namespace Libp

#endif /* LIB_LIBPEKIN_LIBPEKIN_HAL_H_ */
