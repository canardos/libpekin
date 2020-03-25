#ifndef LIB_LIBPEKIN_STM32_ERROR_H_
#define LIB_LIBPEKIN_STM32_ERROR_H_

#include "serial/serial_writer.h"

namespace Libp {

/// Output only
/// May change - do not relay on specific values
namespace ErrCode {
    inline constexpr uint8_t general = 0x01;
    inline constexpr uint8_t display = 0x02;
    inline constexpr uint8_t timer_init = 0x03;

    inline constexpr uint8_t illegal_state = 0x04;
    inline constexpr uint8_t mem_alloc_fail = 0x05;
    inline constexpr uint8_t assert_fail = 0x06;

    // Hardware
    inline constexpr uint8_t adc_init_failure = 0x07;
    inline constexpr uint8_t dma = 0x08;
    inline constexpr uint8_t spi = 0x09;
    inline constexpr uint8_t i2c = 0x0a;
    inline constexpr uint8_t uart_overrun = 0x0b;
    inline constexpr uint8_t flash = 0x0c;
    inline constexpr uint8_t unhandled_exception = 0x0d;

    // Cortex errors
    inline constexpr uint8_t hard_fault = 0x0e;
    inline constexpr uint8_t mem_manage = 0x0f;
    inline constexpr uint8_t bus_fault = 0x10;
    inline constexpr uint8_t usage_fault = 0x11;
}

/**
 * Simple error reporting class for embedded devices.
 *
 * Errors are communicated by flashing a LED with an 8-bit code and/or
 * outputting a text message over a serial output device.
 */

class Error {
public:
    /// Function to turn the error LED on/off
    using SetLedFunc = void (*)(bool);
    /**
     *
     * @param set_led_func pointer to a function to turn the error LED on/off.
     * @param serial device to output error messages to.
     */
    Error(SetLedFunc set_led_func, ISerialIo& serial)
            :  set_led_func_(set_led_func), serial_(serial) {  }

    /**
     * Output an error message via the serial device provided on construction.
     *
     * @param message error message and related parameters
     */
    void report(const char* message, ...);

    /**
     * Flash the provided 8-bit code, pause for 3 seconds, and repeat
     * indefinitely - this function never returns.
     *
     * The code is shown by 8 flashes with a brief flash representing 0 and a
     * longer flash representing 1. LSB first
     *
     * The error code and any provided message will be output via the serial
     * device provided on construction.
     *
     * @param code 8-bit error code
     * @param message error message and related parameters
     */
    [[noreturn]]
    void halt(uint8_t code, const char* message, ...);

    /**
     * Flash the provided 8-bit code, pause for 3 seconds, and repeat
     * indefinitely - this function never returns.
     *
     * The code is shown by 8 flashes with a brief flash representing 0 and a
     * longer flash representing 1. LSB first
     *
     * @param code 8-bit error code
     */
    [[noreturn]]
    void halt(uint8_t code);

    /**
     * Flash the provided 8-bit code once.
     *
     * The code is shown by 8 flashes with a brief flash representing 0 and a
     * longer flash representing 1. LSB first
     *
     * @param code 8-bit error code
     */
    void flashCode(uint8_t code);

private:
    SetLedFunc const set_led_func_;
    static constexpr uint8_t max_output_len = 128;
    const SerialWriter<max_output_len> serial_;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_STM32_ERROR_H_ */
