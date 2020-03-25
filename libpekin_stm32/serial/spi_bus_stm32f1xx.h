/**
 *
 */
#ifndef LIB_LIBPEKIN_STM32_SERIAL_SPI_BUS_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_SERIAL_SPI_BUS_STM32F1XX_H_

#include <cstdint>

#include "libpekin.h"
#include "libpekin_stm32_hal.h"
#include "bus/bus_concepts.h"

namespace LibpStm32::Spi {

enum class CpolCpha : uint8_t {
    /// MSBit capture strobe = SCK first edge (rising)
    cpha0cpol0 = 0,
    /// MSBit capture strobe = SCK first edge (falling)
    cpha0cpol1 = SPI_CR1_CPOL,
    /// MSBit capture strobe = SCK second edge (falling)
    cpha1cpol0 = SPI_CR1_CPHA,
    /// MSBit capture strobe = SCK second edge (rising)
    cpha1cpol1 = SPI_CR1_CPHA | SPI_CR1_CPOL
};
enum class BaudRate {
    pclk_div_2   = 0b000 << SPI_CR1_BR_Pos,
    pclk_div_4   = 0b001 << SPI_CR1_BR_Pos,
    pclk_div_8   = 0b010 << SPI_CR1_BR_Pos,
    pclk_div_16  = 0b011 << SPI_CR1_BR_Pos,
    pclk_div_32  = 0b100 << SPI_CR1_BR_Pos,
    pclk_div_64  = 0b101 << SPI_CR1_BR_Pos,
    pclk_div_128 = 0b110 << SPI_CR1_BR_Pos,
    pclk_div_256 = 0b111 << SPI_CR1_BR_Pos
};
enum DataFrameFormat {
    bits_8 = 0,
    bits_16 = SPI_CR1_DFF
};
enum BitEndianess {
    msb_first = 0,
    lsb_first = SPI_CR1_LSBFIRST
};
enum MasterSlave {
    slave = 0,
    master = SPI_CR1_MSTR
};

/**
 * STM32F1xx SPI implementation of the following Libpekin concepts:
 * - `Bus8BitWritable`
 * - `Bus8BitReadable`
 * - `Bus8BitHalfDuplex`
 * - `Bus8BitFullDuplex`
 *
 * Tested on STM32F103xx
 */
template <uint32_t port_addr_>
class SpiBus {
private:
    static inline SPI_TypeDef* const port_ = reinterpret_cast<SPI_TypeDef*>(port_addr_);

public:

    SpiBus()
    {
        // Confirm that interfaces are correctly implemented
        static_assert(Bus8BitWritable<SpiBus<port_addr_>>);
        static_assert(Bus8BitReadable<SpiBus<port_addr_>>);
        static_assert(Bus8BitHalfDuplex<SpiBus<port_addr_>>);
        static_assert(Bus8BitFullDuplex<SpiBus<port_addr_>>);
    }

    /**
     * Full duplex, no interrupts/DMA, no CRC
     *
     * Assumes appropriate GPIO pins and SPI clock are already configured.
     *
     * @param master
     * @param clock
     * @param baud
     * @param data_ff
     * @param endian
     */
    void start(MasterSlave master, CpolCpha clock, BaudRate baud, DataFrameFormat data_ff, BitEndianess endian) const
    {
        port_->CR1 &= ~SPI_CR1_SPE; // some settings can't be changed while enabled

        port_->CR1 = Libp::enumBaseT(master)  | // master or slave
                     Libp::enumBaseT(clock)   | // clock polarity and phase
                     Libp::enumBaseT(baud)    | // baud rate
                     Libp::enumBaseT(data_ff) | // 8 or 16-bit
                     Libp::enumBaseT(endian)  | // lsb or msb first
                     SPI_CR1_SSM          | // Use external GPIOs for slave select
                     SPI_CR1_SSI;           // Use external GPIOs for slave select

        port_->CR2 = 0;                        // no interrupts
        port_->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD, // SPI mode, not I2S
        port_->CR1 |= SPI_CR1_SPE;
    }

    /**
     * Does not stop the SPI peripheral clock
     */
    inline
    void stop() const
    {
        port_->CR1 &= ~SPI_CR1_SPE;
    }

    bool read(uint8_t* buf, uint32_t len) const
    {
        while(len--) {
            // TODO:
            while(!(port_->SR & SPI_SR_TXE))
                ;
            port_->DR = 0;
            while(!(port_->SR & SPI_SR_RXNE))
                ;
            *buf++ = port_->DR;
        }
        return true;
    }

    bool write(const uint8_t* data, uint32_t len) const
    {
        while(len--) {
            while(!(port_->SR & SPI_SR_TXE))
                ;
            port_->DR = *data++;
        }
        return true;
    }

    uint16_t readwrite16(uint16_t data) const
    {
        while(!(port_->SR & SPI_SR_TXE))
            ;
        port_->DR = data;
        while(!(port_->SR & SPI_SR_RXNE))
            ;
        return port_->DR;
    }

    uint8_t readwrite8(uint8_t data) const
    {
        while(!(port_->SR & SPI_SR_TXE))
            ;
        port_->DR = data;
        while(!(port_->SR & SPI_SR_RXNE))
            ;
        return port_->DR;
    }

    bool readwrite(const uint8_t* out, uint8_t* in, uint32_t len) const
    {
        while(len--) {
            while(!(port_->SR & SPI_SR_TXE))
                ;
            port_->DR = *out++;
            while(!(port_->SR & SPI_SR_RXNE))
                ;
            *in++ = port_->DR;
        }
        return true;
    }

    inline
    bool write8(uint8_t data) const
    {
        readwrite8(data);
        return true;
    }

    inline
    void write16(uint16_t data) const
    {
        readwrite16(data);
    }

    inline
    uint8_t read8() const
    {
        return readwrite8(0);
    }

    inline
    uint16_t read16() const
    {
        return readwrite16(0);
    }
};

} // namespace LibpStm32::Spi

#endif /* LIB_LIBPEKIN_STM32_SERIAL_SPI_BUS_STM32F1XX_H_ */
