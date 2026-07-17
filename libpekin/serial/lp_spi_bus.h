#ifndef LIBPEKIN_SERIAL_LP_SPI_BUS_H_
#define LIBPEKIN_SERIAL_LP_SPI_BUS_H_

#include "libpekin.h"
#include "bus/lp_bus_concepts.h"
#include <cstdint>

namespace libp {

/**
 * Platform independent basic SPI bus class.
 *
 * The class is thin wrapper around the provided `Bus8BitFullDuplex` to set CS
 * active/inactive before/after each call.
 *
 * There is no delay, so if the slave has strict timing requirements, wrap the
 * GpipPinSetClear as needed.
 *
 * Satisfies `Bus8BitFullDuplex`/`ReadWriteReg8bitAddr`.
 *
 * @tparam Bus raw SPI bus implementation for the platform
 * @tparam CsPin CS pin to en/disable the slave device
 */
template <
    Bus8BitFullDuplex auto& Bus,
    GpioPinSetClear auto& CsPin,
    bool CS_IS_ACTIVE_LOW = true>
class SpiBus {
public:
    /**
     * CS will be set inactive on construction.
     */
    SpiBus() {
        static_assert(Bus8BitFullDuplex<SpiBus<Bus, CsPin, CS_IS_ACTIVE_LOW>>, "SpiBus should implement Bus8BitFullDuplex functions");
        static_assert(ReadWriteReg8bitAddr<SpiBus<Bus, CsPin, CS_IS_ACTIVE_LOW>>, "SpiBus should implement ReadWriteReg8bitAddr functions");
        setCsInactive();
    };

    /*
     * Bus8BitFullDuplex functions
     */

    uint8_t read8()
    {
        setCsActive();
        uint8_t data = Bus.readwrite8(0);
        setCsInactive();
        return data;
    }

    bool read(uint8_t* buf, uint32_t len)
    {
        setCsActive();
        bool success = Bus.read(buf, len);
        setCsInactive();
        return success;
    }

    bool write8(uint8_t data)
    {
        setCsActive();
        bool success = Bus.write8(data);
        setCsInactive();
        return success;
    }

    bool write(const uint8_t* data, uint32_t len)
    {
        setCsActive();
        bool success = Bus.write(data, len);
        setCsInactive();
        return success;
    }

    bool write(uint8_t byte, uint32_t count)
    {
        setCsActive();
        bool success = Bus.write(byte, count);
        setCsInactive();
        return success;
    }

    uint8_t readwrite8(uint8_t data)
    {
        setCsActive();
        uint8_t rec = Bus.readwrite8(data);
        setCsInactive();
        return rec;
    }

    bool readwrite(const uint8_t* data_out, uint8_t* data_in, uint32_t len)
    {
        setCsActive();
        bool success = Bus.readwrite(data_out, data_in, len);
        setCsInactive();
        return success;
    }

    /*
     * ReadWriteReg8bitAddr functions
     */

    bool readReg(uint8_t reg_addr, uint8_t *data_in, uint32_t len)
    {
        setCsActive();
        bool success = Bus.write8(reg_addr);
        if (success && len > 0) {
            success = Bus.read(data_in, len);
        }
        setCsInactive();
        return success;
    }

    bool writeReg(uint8_t reg_addr, const uint8_t *data_out, uint32_t len)
    {
        setCsActive();
        bool success = Bus.write8(reg_addr);
        if (success && len > 0) {
            success = Bus.write(data_out, len);
        }
        setCsInactive();
        return success;
    }

    uint8_t readReg(uint8_t reg_addr)
    {
        setCsActive();
        Bus.write8(reg_addr); // ignore return
        uint8_t data = Bus.read8();
        setCsInactive();
        return data;
    }

    bool writeReg(uint8_t reg_addr, uint8_t reg_dat)
    {
        setCsActive();
        bool success = Bus.write8(reg_addr);
        if (success) {
            success = Bus.write8(reg_dat);
        }
        setCsInactive();
        return success;
    }

    bool updateReg(uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_dat)
    {
        setCsActive();
        bool success = Bus.write8(reg_addr);
        if (success) {
            uint8_t cur_val = Bus.read8();
            setCsInactive();
            uint8_t new_val = (cur_val & ~reg_mask) | (reg_dat & reg_mask);
            setCsActive();
            success = Bus.write8(reg_addr);
            if (success) {
                success = Bus.write8(new_val);
            }
        }
        setCsInactive();
        return success;
    }


private:
    inline __attribute__((always_inline))
    void setCsActive()
    {
        if constexpr (CS_IS_ACTIVE_LOW) {
            CsPin.clear();
        }
        else {
            CsPin.set();
        }
    }
    inline __attribute__((always_inline))
    void setCsInactive()
    {
        if constexpr (CS_IS_ACTIVE_LOW) {
            CsPin.set();
        }
        else {
            CsPin.clear();
        }
    }
};

} // namespace libp

#endif // LIBPEKIN_SERIAL_LP_SPI_BUS_H_
