/**
 * C++20 "concepts" for buses and GPIO used by libpekin.
 *
 * These concepts are used to provide a level of abstraction for drivers etc.
 */
#ifndef LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_
#define LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_

#include <cstdint>
#include <cstddef>
#include <concepts>

/// HAL interface for a basic writeable 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitWritable = requires (T bus, uint8_t byte, const uint8_t* data, uint32_t count, uint32_t len) {
    { bus.write8(byte)       } -> std::convertible_to<bool>;
    { bus.write(byte, count) } -> std::convertible_to<bool>;
    { bus.write(data, len)   } -> std::convertible_to<bool>;
};

/// HAL interface for a basic readable 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitReadable = requires (T bus, uint8_t* data, uint32_t len) {
    { bus.read8()         } -> std::same_as<uint8_t>;
    { bus.read(data, len) } -> std::convertible_to<bool>;
};

/// HAL interface for a basic half-duplex 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitHalfDuplex = Bus8BitReadable<T> && Bus8BitWritable<T>;

/// HAL interface for a basic full-duplex 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitFullDuplex = Bus8BitHalfDuplex<T> && requires (T bus, uint8_t byte, const uint8_t* data_out, uint8_t* data_in, uint32_t len) {
    { bus.readwrite8(byte) }                  -> std::same_as<uint8_t>;
    { bus.readwrite(data_out, data_in, len) } -> std::convertible_to<bool>;
};

/// HAL interface for a basic I2C bus master
template<typename T>
concept I2cMaster = requires (
        T bus,
        uint8_t i2c_addr, uint8_t reg_addr, const uint8_t* data_out, uint8_t* data_in, uint32_t len, uint32_t timeout_ms) {
    { bus.masterTransmit(i2c_addr, reg_addr, data_out, len, timeout_ms) }            -> std::convertible_to<bool>;
    { bus.masterTransmit(i2c_addr, data_out, len, timeout_ms) }                      -> std::convertible_to<bool>;
    { bus.masterReceive(i2c_addr, data_in, len, timeout_ms) }                        -> std::convertible_to<bool>;
    { bus.masterTransmitReceive(i2c_addr, data_out, len, data_in, len, timeout_ms) } -> std::convertible_to<bool>;
};

/// HAL interface for a device with readable registers with 8-bit addresses
template<typename T>
concept ReadReg8bitAddr = requires (T bus, uint8_t reg_addr, uint8_t* data_in, uint32_t len) {
    { bus.readReg(reg_addr, data_in, len) }        -> std::convertible_to<bool>;
    /// No error detection, use readReg(reg_addr, data_in, 1) to check for errors
    { bus.readReg(reg_addr) }                      -> std::same_as<uint8_t>;
};

/// HAL interface for a device with writable registers with 8-bit addresses
template<typename T>
concept WriteReg8bitAddr = requires (T bus, uint8_t reg_addr, const uint8_t* data_out, uint8_t reg_dat, uint32_t len) {
    { bus.writeReg(reg_addr, data_out, len) }      -> std::convertible_to<bool>;
    { bus.writeReg(reg_addr, reg_dat) }            -> std::convertible_to<bool>;
};

/// HAL interface for a device with read/writable registers with 8-bit
/// addresses
template<typename T>
concept ReadWriteReg8bitAddr =
        ReadReg8bitAddr<T> &&
        WriteReg8bitAddr<T> &&
        requires (T bus, uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_dat) {

    { bus.updateReg(reg_addr, reg_mask, reg_dat) } -> std::convertible_to<bool>;
};

/// HAL interface for digital set/clear operations on a single GPIO pin
template<typename T>
concept GpioPinSetClear = requires (T pin) {
    { pin.set()      };
    { pin.clear()    };
};

/// HAL interface for digital output operations on a single GPIO pin
template<typename T>
concept GpioPinSettable = GpioPinSetClear<T> && requires (T pin, bool value) {
    { pin.set(value) };
    { pin.toggle()   };
};

/// HAL interface for digital input operations on a single GPIO pin
template<typename T>
concept GpioPinReadable = requires (const T pin) {
    { pin.read()     } -> std::convertible_to<bool>;
};

/// HAL interface for I/O operations on a single GPIO pin
template<typename T>
concept GpioPin = GpioPinSettable<T> && GpioPinReadable<T>;


#endif /* LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_ */
