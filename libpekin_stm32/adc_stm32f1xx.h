/**
 * Basic STM32f1xx ADC functions
 * - Single conversion mode
 * - No DMA/IT
 * - No sequences
 * - Work in progress - largely untested
 *
 * Macro for the MCU in use (as used in the ST CMSIS headers) must be defined.
 * e.g. STM32F101xB, STM32F103xG, STM32F105xC etc.
 */
#ifndef LIB_LIBPEKIN_STM32_ADC_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_ADC_STM32F1XX_H_

#include <cstdint>
#include "libpekin.h"
#include "libpekin_stm32_hal.h"

static_assert(ADC1_BASE > 0, "STM32 CMSIS header must be included before this file");

namespace LibpStm32::Adc {

enum class Rank : uint8_t {
    // DO NOT REORDER
    reg_rank_1 = 1, reg_rank_2, reg_rank_3, reg_rank_4,
    reg_rank_5, reg_rank_6, reg_rank_7, reg_rank_8,
    reg_rank_9, reg_rank_10, reg_rank_11, reg_rank_12,
    reg_rank_13, reg_rank_14, reg_rank_15, reg_rank_16

};

// Using same setting across all channels
/// Sampling time In ADC clock cycles.
enum class SampleTime : uint32_t {
    // octal
    cycles_1pt5   = 00000000000,
    cycles_7pt5   = 01111111111,
    cycles_13pt5  = 02222222222,
    cycles_28pt5  = 03333333333,
    cycles_41pt5  = 04444444444,
    cycles_55pt5  = 05555555555,
    cycles_71pt5  = 06666666666,
    cycles_239pt5 = 07777777777
};

enum class Channel : uint32_t {
    // DO NOT REORDER
    ch0 = 0,ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
    ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16, ch17
};


enum class Align : uint32_t {
    right = 0,
    left = ADC_CR2_ALIGN
};

enum class ScanEnable : uint32_t {
    disable = 0,
    enable = ADC_CR1_SCAN
};

template <uint32_t base_addr_>
class AdcDevice {
    static inline ADC_TypeDef* const adc_ = reinterpret_cast<ADC_TypeDef*>(base_addr_);

public:
    static void setup(Channel ch, SampleTime time)
    {
        adc_->SMPR1 = Libp::enumBaseT(time) & 0x00FFFFFF; // upper 8-bits must be kept at
                                                           // reset value according to docs
        adc_->SMPR2 = Libp::enumBaseT(time);
        // No sequence, single channel
        adc_->SQR1 = 0x0000;
        adc_->SQR2 = 0x0000;
        adc_->SQR3 = 0x0000 | Libp::enumBaseT(ch);
    }

    static bool isEnabled()
    {
        return (adc_->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON;
    }

    // power up
    static bool enable()
    {
        if (isEnabled())
            return true;

        adc_->CR2 |= ADC_CR2_ADON;

        // delay 1uS
        uint32_t wait_loop_index = (1u * (72'000'000 / 1'000'000U));
        while(wait_loop_index != 0U)
          wait_loop_index--;

        uint64_t start = Libp::getMillis();
        while(!isEnabled())
        {
          if((Libp::getMillis() - start) > 2u)
              return false;
        }
        return true;
    }
    // power down
    static void disable()
    {
        adc_->CR2 &= (~ADC_CR2_ADON);
    }

/*    static void start()
    {
        enable();
        adc_->SR &= !ADC_SR_EOS;

        if (ADC_IS_SOFTWARE_START_REGULAR(hadc) && ADC_NONMULTIMODE_OR_MULTIMODEMASTER(hadc) )
        {
          // Start ADC conversion on regular group with SW start
          SET_BIT(hadc->Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
        }
        else
        {
          // Start ADC conversion on regular group with external trigger
          SET_BIT(hadc->Instance->CR2, ADC_CR2_EXTTRIG);
        }
    }

    static void stop()
    {
        disable();
    }*/

    static uint32_t poll()
    {
        enable();
        // Start conversion
        adc_->CR2 |= ADC_CR2_ADON;
        // TODO: timeout
        while (!(adc_->SR & ADC_SR_EOS))
            ;
        uint32_t value = adc_->DR;
        disable();
        return value;
    }
};

} // namespace LibpStm32::Adc

#endif /* LIB_LIBPEKIN_STM32_ADC_STM32F1XX_H_ */
