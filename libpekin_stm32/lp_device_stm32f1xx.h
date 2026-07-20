#ifndef LIBPEKIN_STM32_LP_DEVICE_STM32F1XX_H_
#define LIBPEKIN_STM32_LP_DEVICE_STM32F1XX_H_

#include "lp_libpekin_stm32_hal.h"
#include <cstdint>

/**
 * WARNING: Untested.
 *
 * Return the unique 96-bit device ID.
 *
 * @param id
 */
inline void getDeviceId(uint32_t (&id)[3])
{
  id[0] = *reinterpret_cast<const uint32_t*>(UID_BASE);
  id[1] = *reinterpret_cast<const uint32_t*>(UID_BASE + 4U);
  id[2] = *reinterpret_cast<const uint32_t*>(UID_BASE + 8U);
}





#endif /* LIBPEKIN_STM32_LP_DEVICE_STM32F1XX_H_ */
