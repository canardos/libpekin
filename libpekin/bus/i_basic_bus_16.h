#ifndef SRC_I_BASIC_BUS_16_H_
#define SRC_I_BASIC_BUS_16_H_

#include <cstdint>

namespace Libp {

/**
 * Interface functions for I/O over a 16-bit non-addressable bus.
 */
class IBasicBus16 {
public:
    virtual ~IBasicBus16() = default;

//    virtual void chipSelect() = 0;

//    virtual void chipDeselect() = 0;

    /**
     * Write a single 8-bit data value to the bus.
     * @param data the data to write
     */
    virtual void write8(uint8_t data) = 0;

    /**
     * Write a single 16-bit data value to the bus.
     * @param data the data to write
     */
    virtual void write16(uint16_t data) = 0;

    /**
     * Write a single 8-bit command to the bus.
     *
     * This may be the same as write8 depending on the implementation.
     *
     * @param cmd the command to write
     */
    virtual void write8Cmd(uint8_t cmd) = 0;

    /**
     * Write a single 8-bit data value to the bus `n` number of times.
     * @param data the data to write
     * @param n the number of times to write (i.e. total of n bytes will be
     *          written)
     */
    virtual void write8Repeat(uint8_t data, uint32_t n) = 0;
    /**
     * Write a single 16-bit data value to the bus `n` number of times.
     * @param data the data to write
     * @param n the number of times to write (i.e. total of n*2 bytes will be
     *          written)
     */
    virtual void write16Repeat(uint16_t data, uint32_t n) = 0;

    /**
     * Write an 8-bit data array to the bus.
     * @param data pointer to array of data to write
     * @param len length of array
     */
    virtual void write8Array(const uint8_t* data, uint32_t len) = 0;
    /**
     * Write a 16-bit data array to the bus.
     * @param data pointer to array of data to write
     * @param len length of array (i.e. len*2 bytes will be written)
     */
    virtual void write16Array(const uint16_t* data, uint32_t len) = 0;

    /**
     * Read an 8-bit value from the bus.
     * @return
     */
    virtual uint8_t read8() = 0;

    /**
     * Read a 16-bit value from the bus.
     * @return
     */
    virtual uint16_t read16() = 0;
};


} // namespace Libp

#endif /* SRC_I_BASIC_BUS_16_H_ */
