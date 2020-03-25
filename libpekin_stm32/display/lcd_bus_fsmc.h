#ifndef SRC_DISPLAY_LCD_BUS_FSMC_H_
#define SRC_DISPLAY_LCD_BUS_FSMC_H_

#include <cstdint>
#include <libpekin_stm32_hal.h>
#include "bus/i_basic_bus_16.h"
#include "dma_stm32f1xx.h"

// One address line is used to signal data/cmd to the display (RS)
// Set the dat/cmd address based on the line used.

namespace LibpStm32 {

// TODO: migrate to a standard bus concept
/**
 * ILcdBus implementation for the STM32 FSMC controller with optional DMA
 * functionality.
 *
 * See IBasicBus16 for member function documentation.
 *
 * The Data/Cmd single line of the display (e.g. RS) is driven by one of the
 * FSMC address lines. Set the FSMC data/cmd address based on the line used.
 *
 *
 * @tparam fsmc_dat_addr_ FSMC data address
 * @tparam fsmc_cmd_addr_ FSMC cmd address
 * @tparam dma_ch_
 * @tparam dma_addr_
 */
template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_ = 1, uint32_t dma_addr_ = DMA1_BASE>
class LcdBusFsmc : public Libp::IBasicBus16 {
public:

    /**
     * If `dma` is provided, DMA will be used in `write16Array` function.
     *
     * @param dma may be null
     */
    LcdBusFsmc(LibpStm32::Dma::DmaDevice<dma_ch_, dma_addr_>* dma = nullptr) : dma_(dma) { }
    virtual ~LcdBusFsmc() { };

    void write8(uint8_t data);
    void write16(uint16_t data);
    void write8Cmd(uint8_t cmd);

    void write8Repeat(uint8_t data, uint32_t n);
    void write16Repeat(uint16_t data, uint32_t n);

    void write8Array(const uint8_t* data, uint32_t len);

    /**
     * If DMA is enabled, DMA transfer with interrupts will be used and this
     * function will return immediately.
     *
     * @param data
     * @param len
     */
    void write16Array(const uint16_t* data, uint32_t len);

    uint8_t read8();
    uint16_t read16();

private:
    LibpStm32::Dma::DmaDevice<dma_ch_, dma_addr_>* const dma_;
    volatile uint16_t& fsmc_tft_dat = *reinterpret_cast<volatile uint16_t *>(fsmc_dat_addr_);
    volatile uint16_t& fsmc_tft_cmd = *reinterpret_cast<volatile uint16_t *>(fsmc_cmd_addr_);
};

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write8(uint8_t data) {
    fsmc_tft_dat = data;
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write16(uint16_t data) {
    fsmc_tft_dat = data;
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write8Cmd(uint8_t cmd) {
    fsmc_tft_cmd = cmd;
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write8Repeat(uint8_t data, uint32_t n) {
    while (n-- != 0) {
        fsmc_tft_dat = data;
    }
}

// TODO: DMA
template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write16Repeat(uint16_t data, uint32_t n) {
    while (n-- != 0) {
        fsmc_tft_dat = data;
    }
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write8Array(const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        fsmc_tft_dat = data[i];
    }
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
void LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::write16Array(const uint16_t* data, uint32_t len) {
    if (dma_) {
        dma_->start((uint32_t)data, fsmc_dat_addr_, len);
    }
    else {
        for (uint32_t i = 0; i < len; i++) {
            fsmc_tft_dat = data[i];
        }
    }
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
uint8_t LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::read8() {
    return fsmc_tft_dat;
}

template <uint32_t fsmc_dat_addr_ , uint32_t fsmc_cmd_addr_ , uint8_t dma_ch_, uint32_t dma_addr_>
uint16_t LcdBusFsmc<fsmc_dat_addr_, fsmc_cmd_addr_, dma_ch_, dma_addr_>::read16() {
    return fsmc_tft_dat;
}


} // namespace LibpStm32

#endif /* SRC_DISPLAY_LCD_BUS_FSMC_H_ */
