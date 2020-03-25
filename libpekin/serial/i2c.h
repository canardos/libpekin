#ifndef LIB_LIBPEKIN_SERIAL_I2C_H_
#define LIB_LIBPEKIN_SERIAL_I2C_H_

#include "bus/bus_concepts.h"

namespace Libp {

/**
 * ReadWriteReg8bitAddr "interface" implementation for a to communicate with a
 * slave device connected over an I2C bus.
 *
 * @tparam I2c
 */
template <I2cMaster I2c>
class I2cWrapper {
public:
    /**
     *
     * @param i2c
     * @param slave_addr TODO
     */
    I2cWrapper(I2c& i2c, uint8_t slave_addr) : i2c_(i2c), slave_addr_(slave_addr)
    {
        static_assert(ReadWriteReg8bitAddr<I2cWrapper<I2c>>);
    }

    /**
     * Read an 8-bit register value using an 8-bit register address.
     *
     * S -> slave_addr W -> write 8-bit reg_addr -> SR -> slave_addr R -> read 8-bit data -> P
     *
     * @param reg_addr register address
     *
     * @return 8-bit register value or >255 on error (timeout)
     */
    uint16_t readReg(uint8_t reg_addr) const
    {
        uint8_t data;
        bool success = i2c_.masterTransmitReceive(slave_addr_, &reg_addr, 1, &data, 1, i2c_timeout_ms_);
        return success ? data : 256;
    }

    /**
     * Read multiple bytes using an 8-bit register address.
     *
     * S -> slave_addr W -> write 8-bit reg_addr -> SR -> slave_addr R -> read 8-bit data[0] -> ... -> read 8-bit data[n] -> P
     *
     * @param [in]  reg_addr register address
     * @param [out] buffer buffer to receive the read bytes
     * @param [in]  len number of bytes to read into `buffer`.
     *
     * @return true on success, false on timeout
     */
    bool readReg(uint8_t reg_addr, uint8_t* buffer, size_t len) const
    {
        return i2c_.masterTransmitReceive(slave_addr_, &reg_addr, 1, buffer, len, i2c_timeout_ms_);
    }

    /**
     * Write an 8-bit value using an 8-bit register address.
     *
     * S -> slave_addr W -> write 8-bit reg_addr -> write 8-bit reg_vaue -> P
     *
     * @param reg_addr register address
     * @param reg_value register value
     *
     * @return true on success, false on timeout
     */
    bool writeReg(uint8_t reg_addr, uint8_t reg_dat) const
    {
        return i2c_.masterTransmit(slave_addr_, reg_addr, &reg_dat, 1, i2c_timeout_ms_);
    }

    /**
     * Write multiple bytes using an 8b-t register address.
     *
     * S -> slave_addr W -> write 8-bit reg_addr -> write 8-bit reg_vaue[0] -> ... -> write 8-bit reg_vaue[n] -> P
     *
     * @param reg_addr register address
     * @param reg_dat array of bytes to write
     * @param len number of bytes in `reg_dat` to write
     *
     * @return true on success, false on timeout
     */
    bool writeReg(uint8_t reg_addr, const uint8_t* reg_dat, size_t len) const
    {
        return i2c_.masterTransmit(slave_addr_, reg_addr, reg_dat, len, i2c_timeout_ms_);
    }

    /**
     * Update select bits in an 8-bit value in aregister using an 8-bit
     * address.
     *
     * S -> slave_addr W -> write 8-bit reg_addr -> SR -> slave_addr R -> read 8-bit data -> P
     * S -> slave_addr W -> write 8-bit reg_addr -> SR -> write 8-bit value -> P
     *
     * @param reg_addr register address
     * @param reg_mask mask of bits to clear
     * @param reg_value value of bits to apply
     *
     * @return the prior 8-bit register value or >255 on error (timeout)
     */
    uint16_t updateReg(uint8_t reg_addr, uint8_t mask, uint8_t value) const
    {
        uint8_t data[] = { reg_addr, 0 };
        uint8_t cur_value;
        if (!i2c_.masterTransmitReceive(slave_addr_, data, 1, &cur_value, 1, i2c_timeout_ms_))
            return 256;
        data[1] = (cur_value & ~mask) | value;
        if (!i2c_.masterTransmit(slave_addr_, data, 2, i2c_timeout_ms_))
            return 256;
        return cur_value;
    }


private:
    const I2c& i2c_;
    const uint8_t slave_addr_;
    static constexpr uint32_t i2c_timeout_ms_ = 100;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_SERIAL_I2C_H_ */
