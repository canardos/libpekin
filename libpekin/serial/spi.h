#ifndef LIB_LIBPEKIN_SERIAL_SPI_H_
#define LIB_LIBPEKIN_SERIAL_SPI_H_

#include <cstdint>
#include "libpekin.h"
#include "bus/bus_concepts.h"

namespace Libp {

/**
 * Platform independent basic SPI bus operations class.
 *
 * Communication with the underlying hardware is handled by the SPI bus
 * provided on construction.
 *
 * @tparam Bus8BitFullDuplex Platform specific SpiBus implementation
 */
template <Bus8BitFullDuplex spibus>
class SpiRegisterOps {
public:
    using CsEnableFunc = void (*)(bool active);

    /**
     *
     * @param bus SpiBus
     * @param cs_func
     */
    SpiRegisterOps(spibus& bus, CsEnableFunc cs_func) : bus_(bus), cs_set_active_(cs_func) {
        cs_set_active_(false);
    };

    uint8_t read(uint8_t addr)
    {
        cs_set_active_(true);
        bus_.readwrite8(addr);
        uint8_t data = bus_.readwrite8(0);
        cs_set_active_(false);
        return data;
    }

    void read(uint8_t addr, uint8_t* buf, uint8_t len)
    {
        cs_set_active_(true);
        bus_.readwrite8(addr);
        bus_.read(buf, len);
        cs_set_active_(false);
    }

    void write(uint8_t addr, uint8_t data)
    {
        cs_set_active_(true);
        bus_.readwrite8(addr);
        bus_.readwrite8(data);
        cs_set_active_(false);
    }

    void write(uint8_t addr, uint8_t* data, uint8_t len)
    {
        cs_set_active_(true);
        bus_.readwrite8(addr);
        bus_.write(data, len);
        cs_set_active_(false);
    }
private:
    const spibus& bus_;
    CsEnableFunc cs_set_active_;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_SERIAL_SPI_H_ */
