/**
 * Not even close to ready for use
 */
#ifndef LIB_LIBPEKIN_STM32_DAC_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_DAC_STM32F1XX_H_

#include <cstdint>
#include "libpekin.h"
#include "libpekin_stm32_hal.h"
#include "dma_stm32f1xx.h"

#ifndef DAC_BASE
static_assert(false, "STM32 CMSIS header must be included before this file");
#endif

namespace LibpStm32::Dac {

enum class Channel : uint8_t {
    ch1 = 0,
    ch2 = 0x10
};

enum class Format : uint8_t {
    // value is register offset for output
    r_align_12bit = 0x08,
    l_align_12bit = 0x0C,
    r_align_8bit  = 0x10
};

// For STM32F10x microcontrollers, DAC channel1 is connected to the DMA
// channel3 and DAC channel2 is connected to DMA channel4.

// TIM6 and TIM7 are basic timers and are intended for DAC triggering


/**
 * DAC configuration structure intended to facilitate easy calculation of a DAC
 * CR configuration register value at compile time.
 *
 * \code
 * Usage:
 *
 * using namespace LibpStm32::Dac
 *
 * constexpr Cfg cfg = {
 *     .ch1_output_buf_en = false,
 *     .ch1_dma_en = true,
 *     .ch1_wave_gen = WaveGen::triangle,
 * };
 * constexpr uint32_t dma_cr_reg = cfg.toCrRegVal();
 *
 * // OR
 *
 * constexpr uint32_t dma_cr_reg = []() {
 *     Cfg cfg;
 *     cfg.ch1_output_buf_en = false;
 *     cfg.ch1_dma_en = true;
 *     cfg.ch1_wave_gen = WaveGen::triangle;
 *
 *     return cfg.toCrRegVal();
 * }();
 *
 * DacDevice<DAC_BASE>::configure(dma_cr_reg);
 *
 * \endcode
 */
struct Cfg {
    enum class Trigger : uint32_t {
        none        = 0b1111,
        tim6_trgo   = 0b0000,
        tim3_8_trgo = 0b0001, ///< TIM3 for connectivity line, TIM8 for high/XL density
        tim7_trgo   = 0b0010,
        tim5_trgo   = 0b0011,
        tim2_trgo   = 0b0100,
        tim4_trgo   = 0b0101,
        ext_line9   = 0b0110,
        software    = 0b0111,
    };
    enum class WaveGen : uint32_t {
        disabled = 0b00,
        noise    = 0b01,
        triangle = 0b10
    };
    /// LFSR (Linear Feedback Shift Register)/triangle waveform amplitude
    /// Not used if WaveGen::disabled
    enum class WaveGenLfsrAmp : uint32_t  {
        amp_1    = 0b0000, ///< amp_1
        amp_3    = 0b0001, ///< amp_3
        amp_7    = 0b0010, ///< amp_7
        amp_15   = 0b0011, ///< amp_15
        amp_31   = 0b0100, ///< amp_31
        amp_63   = 0b0101, ///< amp_63
        amp_127  = 0b0110, ///< amp_127
        amp_255  = 0b0111, ///< amp_255
        amp_511  = 0b1000, ///< amp_511
        amp_1023 = 0b1001, ///< amp_1023
        amp_2047 = 0b1010, ///< amp_2047
        amp_4095 = 0b1011, ///< amp_4095
    };

    bool ch1_dma_en = false;
    bool ch2_dma_en = false;
    bool ch1_output_buf_en = true;
    bool ch2_output_buf_en = true;
    Trigger ch1_trigger = Trigger::none;
    Trigger ch2_trigger = Trigger::none;
    WaveGen ch1_wave_gen = WaveGen::disabled;
    WaveGen ch2_wave_gen = WaveGen::disabled;
    WaveGenLfsrAmp ch1_wave_gen_amp = WaveGenLfsrAmp::amp_1;
    WaveGenLfsrAmp ch2_wave_gen_amp = WaveGenLfsrAmp::amp_1;

    /// Calculate DAC_CR register value from configuration
    constexpr uint32_t toCrRegVal()
    {
        using namespace Libp;
        return
              ((!ch1_output_buf_en) << DAC_CR_BOFF1_Pos)
            | ((!ch2_output_buf_en) << DAC_CR_BOFF2_Pos)
            | (ch1_dma_en << DAC_CR_DMAEN1_Pos)
            | (ch2_dma_en << DAC_CR_DMAEN2_Pos)
            | ((ch1_trigger != Trigger::none) << DAC_CR_TEN1_Pos)
            | ((ch2_trigger != Trigger::none) << DAC_CR_TEN2_Pos)
            | ((ch1_trigger == Trigger::none) ? 0 : enumBaseT(ch1_trigger) << DAC_CR_TSEL1_Pos)
            | ((ch2_trigger == Trigger::none) ? 0 : enumBaseT(ch2_trigger) << DAC_CR_TSEL2_Pos)
            | (enumBaseT(ch1_wave_gen) << DAC_CR_WAVE1_Pos)
            | (enumBaseT(ch2_wave_gen) << DAC_CR_WAVE2_Pos)
            | (enumBaseT(ch1_wave_gen_amp) << DAC_CR_MAMP1_Pos)
            | (enumBaseT(ch2_wave_gen_amp) << DAC_CR_MAMP2_Pos);
    }
};


/**
 *
 * @tparam base_addr_
 * @tparam channel_ 1 or 2
 */
template <uint32_t base_addr_ = DAC_BASE, uint8_t channel_ = 1>
class DacDevice {
    static inline DAC_TypeDef* const dac_ = reinterpret_cast<DAC_TypeDef*>(base_addr_);
public:
    inline __attribute__((always_inline))
    static void disable(Channel dac_ch)
    {
        dac_->CR &= ~(DAC_CR_EN1 << Libp::enumBaseT(dac_ch));
    }
    inline __attribute__((always_inline))
    static void enable(Channel dac_ch)
    {
        dac_->CR |= (DAC_CR_EN1 << Libp::enumBaseT(dac_ch));
    }

    /**
     * Use `Cfg` class to generate a configuration value.
     *
     * Does not alter channel enabled status
     *
     * @param cr_reg_val
     */
    inline __attribute__((always_inline))
    static void configure(uint32_t cr_reg_val)
    {
        dac_->CR = (dac_->CR & (DAC_CR_EN1 | DAC_CR_EN2)) | cr_reg_val;
    }

    template <uint32_t dma_base_addr, uint8_t dma_channel>
    static void stopDma(Channel dac_ch)
    {
        // disable DAC and DAC DMA
        dac_->CR &= ~( (DAC_CR_DMAEN1 << Libp::enumBaseT(dac_ch)) | (DAC_CR_EN1 << Libp::enumBaseT(dac_ch)) );
        //Stm32Dma<dma_base_addr, dma_channel>::abort();
    }


    /// DAC DMA does not need to be enabled prior to calling
    template <uint8_t dma_channel, uint32_t dma_base_addr = DMA1_BASE>
    static void startDma(Channel dac_ch, uint32_t* data, uint32_t len, Format fmt)
    {
        dac_->CR |= (dac_ch == Channel::ch1 ? DAC_CR_DMAEN1 : DAC_CR_DMAEN2);
        const uint32_t dac_addr = base_addr_ + Libp::enumBaseT(fmt) + (dac_ch == Channel::ch2 ? 12 : 0);
        LibpStm32::Dma::DmaDevice<dma_channel, dma_base_addr>::start(dac_addr, (uint32_t)data, len);
        enable(dac_ch);
    }
};

} // namespace LibpStm32::Dac

#endif /* LIB_LIBPEKIN_STM32_DAC_STM32F1XX_H_ */
