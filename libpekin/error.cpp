/**
 * @file XXXX
 * @author XXXX
 * @version XXXXX
 * @date XXXXXX
 * @brief XXXX
 * @param message
 *
 * @verbatim
 * XXXX
 * @endverbatim
 *
 * @attention
 * COPYRIGHT
 */

#include <stdarg.h>
#include "error.h"
#include "libpekin.h"

namespace Libp {

void Error::report(const char* message, ...) {
    va_list argptr;
    va_start(argptr, message);
    serial_.printf(message, argptr);
    va_end(argptr);
}

void Error::halt(uint8_t code, const char* message, ...) {
    va_list argptr;
    va_start(argptr, message);
    serial_.printf("(%u) ", code);
    serial_.printf(message, argptr);
    serial_.printf("\r\n");
    va_end(argptr);
    halt(code);
}

static constexpr uint16_t dot_len_ms = 100;
static constexpr uint16_t dash_len_ms = 500;
/// Pause between each 1-bit
static constexpr uint16_t gap_len_ms = 500;
/// Pause between each 8-bit output
static constexpr uint16_t pause_len_ms = 4000;

void Error::halt(uint8_t code) {
    while (true) {
        flashCode(code);
        delayMs(pause_len_ms - gap_len_ms);
    }
}

void Error::flashCode(uint8_t code) {
    for (uint8_t i = 0; i < 8; i++) {
        set_led_func_(true);
        delayMs( (code & (1 << i)) ? dash_len_ms : dot_len_ms);
        set_led_func_(false);
        delayMs(gap_len_ms);
    }
}

} // namespace Libp
