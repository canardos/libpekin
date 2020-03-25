/**
 * Libpekin library functions include this single header for all STM32 HAL XXXX.
 *
 * Make changes for different hardware here only.
 */
#ifndef LIB_LIBPEKIN_STM32_HAL_H_
#define LIB_LIBPEKIN_STM32_HAL_H_

#include <cstdint>

// TODO: Update macros for actual chip
// TODO: client code should include approprite cmsis header
/*

From stm32f1xx.h

STM32F100xB
STM32F100xE
STM32F101x6
STM32F101xB
STM32F101xE
STM32F101xG
STM32F102x6
STM32F102xB
STM32F103x6
STM32F103xB
STM32F103xE
STM32F103xG
STM32F105xC
STM32F107xC

*/
//stm32f10x_map.h
#include "stm32f1xx.h"

namespace Libp {

using SysTickHandler = void(*)();
/**
 * Initialize delay timers.
 *
 * Required by the following functions:
 * - `getMillis`
 * - `delayMs`
 * - `delayUs`
 * - `delayNs`
 *
 * Start DWT (Debug Watchpoint Trace) counter for `delay_us` function
 *
 * @param handler TODO
 * @return true 0 on success, 1 on error
 */
bool libpekinInitTimers(SysTickHandler handler = nullptr);

/**
 * Untested.
 *
 * Return the unique 96-bit device ID.
 *
 * @param id
 */
void getDeviceId(uint32_t (&id)[3]);

} // namespace Libp

#endif /* LIB_LIBPEKIN_STM32_HAL_H_ */
