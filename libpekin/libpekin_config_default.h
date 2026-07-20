/*
 * Default libpekin_config.h file
 */

#ifndef LIBPEKIN_LIBPEKIN_CONFIG_DEFAULT_H_
#define LIBPEKIN_LIBPEKIN_CONFIG_DEFAULT_H_

// === Timers ===

// Set the type returned by the function libp::getMillis.
// Your application must provide an implementation of this function for use of
// several library components.
//
// The default type is uint32_t
//
// See `libpekin.h` for details

// #include <cstdint>
// #define get_millis_ret_t uint32_t
// #define get_millis_ret_t uint64_t


// === Debugging ===

// The library uses the assert macro LP_ASSERT in various places. IF a
// definition is not provided, it defaults the `assert` provided by the cassert
// header.
// Provide a definition here specific to the platform in use:

// e.g. SiLabs EFR32
//
// #include "em_assert.h"
// #define LP_ASSERT EFM_ASSERT

// e.g. STM32
//
// #include "stm32_assert.h"
// #define LP_ASSERT assert_param


// === Logging ===

// Enable global logging and logging for individual components.
// The application must provide the logging function implementation.
// See `lp_logging.h` for details.
//
// #define LP_LOG_ENABLE
// #define EN_LOG_BSM
// #define EN_LOG_ONOFFLED
// #define EN_LOG_ZCL_SCRIPTS
//
#endif /* LIBPEKIN_LIBPEKIN_CONFIG_DEFAULT_H_ */
