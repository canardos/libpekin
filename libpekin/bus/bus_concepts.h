/**
 * C++20 "concepts" for buses and GPIO used by libpekin.
 *
 * These concepts are used to provide a level of abstraction for drivers etc.
 */
#ifndef LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_
#define LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_

#include <cstdint>
#include <cstddef>
#if __GNUC__ >= 10
#include <concepts>
#endif

/// HAL interface for a basic writeable 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitWritable = requires (const T bus, uint8_t byte, const uint8_t* data, uint32_t len) {
#if __GNUC__ >= 10
    { bus.write8(byte)     } -> std::same_as<bool>;
    { bus.write(data, len) } -> std::same_as<bool>;
#else
    { bus.write8(byte)     } -> bool;
    { bus.write(data, len) } -> bool;
#endif
};


/// HAL interface for a basic readable 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitReadable = requires (const T bus, uint8_t* data, uint32_t len) {
#if __GNUC__ >= 10
    { bus.read8()         } -> std::same_as<uint8_t>;
    { bus.read(data, len) } -> std::same_as<bool>;
#else
    { bus.read8()         } -> uint8_t;
    { bus.read(data, len) } -> bool;
#endif
};

/// HAL interface for a basic half-duplex 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitHalfDuplex = Bus8BitReadable<T> && Bus8BitWritable<T>;

/// HAL interface for a basic full-duplex 8-bit bus (no addressing/IRQ/DMA)
template<typename T>
concept Bus8BitFullDuplex = Bus8BitHalfDuplex<T> && requires (const T bus, uint8_t byte, const uint8_t* data_out, uint8_t* data_in, uint32_t len) {
#if __GNUC__ >= 10
    { bus.readwrite8(byte)                  } -> std::same_as<uint8_t>;
#else
    { bus.readwrite8(byte)                  } -> uint8_t;
#endif
    { bus.readwrite(data_out, data_in, len) };
};

/// HAL interface for a basic I2C bus master
template<typename T>
concept I2cMaster = requires (
        T bus,
        uint8_t i2c_addr, uint8_t reg_addr, const uint8_t* data_out, uint8_t* data_in, size_t len, uint32_t timeout_ms) {
#if __GNUC__ >= 10
    { bus.masterTransmit(i2c_addr, reg_addr, data_out, len, timeout_ms) } -> std::same_as<bool>;
    { bus.masterTransmit(i2c_addr, data_out, len, timeout_ms) } -> std::same_as<bool>;
    { bus.masterReceive(i2c_addr, data_in, len, timeout_ms) } -> std::same_as<bool>;
    { bus.masterTransmitReceive(i2c_addr, data_out, len, data_in, len, timeout_ms) } -> std::same_as<bool>;
#else
    { bus.masterTransmit(i2c_addr, reg_addr, data_out, len, timeout_ms) } -> bool;
    { bus.masterTransmit(i2c_addr, data_out, len, timeout_ms) } -> bool;
    { bus.masterReceive(i2c_addr, data_in, len, timeout_ms) } -> bool;
    { bus.masterTransmitReceive(i2c_addr, data_out, len, data_in, len, timeout_ms) } -> bool;
#endif
};

/// HAL interface for a device with read/writable registers with 8-bit
/// addresses
template<typename T>
concept ReadWriteReg8bitAddr = requires (T bus, uint8_t reg_addr, const uint8_t* data_out, uint8_t* data_in, uint8_t reg_mask, uint8_t reg_dat, size_t len) {
#if __GNUC__ >= 10
    { bus.writeReg(reg_addr, data_out, len) } -> std::same_as<bool>;
    { bus.readReg(reg_addr, data_in, len) } -> std::same_as<bool>;
    { bus.readReg(reg_addr) } -> std::same_as<uint16_t>;
    { bus.writeReg(reg_addr, reg_dat) } -> std::same_as<bool>;
    { bus.updateReg(reg_addr, reg_mask, reg_dat) } -> std::same_as<uint16_t>;
#else
    { bus.writeReg(reg_addr, data_out, len) } -> bool;
    { bus.readReg(reg_addr, data_in, len) } -> bool;
    { bus.readReg(reg_addr) } -> uint16_t;
    { bus.writeReg(reg_addr, reg_dat) } -> bool;
    { bus.updateReg(reg_addr, reg_mask, reg_dat) } -> uint16_t;
#endif
};

/// HAL interface for digital output operations on a single GPIO pin
template<typename T>
concept GpioPinSettable = requires (const T pin, bool value) {
    { pin.set()      };
    { pin.set(value) };
    { pin.clear()    };
    { pin.toggle()   };
};

/// HAL interface for digital input operations on a single GPIO pin
template<typename T>
concept GpioPinReadable = requires (const T pin, bool value) {
#if __GNUC__ >= 10
    { pin.read()     } -> std::same_as<bool>;
#else
    { pin.read()     } -> bool;
#endif
};

/// HAL interface for I/O operations on a single GPIO pin
template<typename T>
concept GpioPin = GpioPinSettable<T> && GpioPinReadable<T>;


#endif /* LIB_LIBPEKIN_BUS_BUS_CONCEPTS_H_ */
