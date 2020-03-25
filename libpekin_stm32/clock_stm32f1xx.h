/**
 * Basic STM32f1xx clock configuration functionality.
 *
 * Requires CMSIS functionality.
 *
 * Macro for the MCU in use (as used in the ST CMSIS headers) must be defined.
 * e.g. STM32F101xB, STM32F103xG, STM32F105xC etc.
 */
#ifndef LIB_LIBPEKIN_STM32_CLOCK_STM32_H_
#define LIB_LIBPEKIN_STM32_CLOCK_STM32_H_

#include <cstdint>
#include "libpekin.h"
#include "libpekin_stm32_hal.h"
#include "bits.h"

static_assert(GPIOA_BASE > 0, "STM32 CMSIS header must be included before this file");

namespace LibpStm32::Clk {

/// APB1 peripherals (STM32F1xx)
enum class Apb1 : uint8_t {
    // DO NOT REORDER
    tim2 = 0, tim3, tim4, tim5, tim6, tim7, tim12, tim13, tim14,
	wwd = 11,
    spi2 = 14, spi3,
    usart2 = 17, usart3, usart4, usart5, i2c1, i2c2, usb,
    can = 25,
    bkp = 27, pwr, dac
};

/// APB2 peripherals (STM32F1xx)
enum class Apb2 : uint8_t {
    // DO NOT REORDER
    afio = 0,
    iopa = 2, iopb, iopc, iopd, iope, iopf, iopg, adc1, adc2, tim1, spi1, tim8, usart1, adc3,
    time9 = 19, tim10, tim11
};

/// AHB peripherals (STM32F1xx)
enum class Ahb : uint8_t {
    // DO NOT REORDER
    dma1 = 0, dma2, sram,
    flitf = 4,
    crce = 6,
    fsmc = 8,
    sdio = 10
};


/**
 * Enable one or more APB1 peripheral clocks.
 *
 * e.g. Clk::enable<Clk::Apb1::tim2, Clk::Apb1::spi2>();
 *
 * Note:
 * No delay is performed after enabling. Per STM errata, a delay of 1 or 2
 * APB cycles is required on some variants prior to the enabling becoming
 * effective. If required, a dummy read of the relevant register is sufficient.
 *
 * @tparam peripheral
 */
template <Apb1... peripheral>
inline __attribute__((always_inline))
void enable()
{
    RCC->APB1ENR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
}

/**
 * Enable one or more APB2 peripheral clocks.
 *
 * e.g. Clk::enable<Clk::Apb2::afio, Clk::Apb2::iopa>();
 *
 * Note:
 * No delay is performed after enabling. Per STM errata, a delay of 1 or 2
 * APB cycles is required on some variants prior to the enabling becoming
 * effective. If required, a dummy read of the relevant register is sufficient.
 *
 * @tparam peripheral
 */
template <Apb2... peripheral>
inline __attribute__((always_inline))
void enable()
{
    RCC->APB2ENR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
}

/**
 * Enable one or more AHB peripheral clocks.
 *
 * e.g. Clk::enable<Clk::Ahb::fsmc, Clk::Ahb::sdio>();
 *
 * Note:
 * No delay is performed after enabling. Per STM errata, a delay of 1 or 2
 * AHB cycles is required on some variants prior to the enabling becoming
 * effective. If required, a dummy read of the relevant register is sufficient.
 *
 * @tparam peripheral
 */
template <Ahb... peripheral>
inline __attribute__((always_inline))
void enable()
{
    RCC->AHBENR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
}


/**
 * Disable one or more APB1 peripheral clocks.
 *
 * e.g. Clk::disable<Clk::Apb1::tim2, Clk::Apb1::spi2>();
 *
 * @tparam peripheral
 */
template <Apb1... peripheral>
inline __attribute__((always_inline))
void disable()
{
    RCC->APB1ENR &= ~((1u << Libp::enumBaseT(peripheral)) | ...);
}

/**
 * Disable one or more APB2 peripheral clocks.
 *
 * e.g. Clk::enable<Clk::Apb2::afio, Clk::Apb2::iopa>();
 *
 * @tparam peripheral
 */
template <Apb2... peripheral>
inline
void disable()
{
    RCC->APB2ENR &= ~((1u << Libp::enumBaseT(peripheral)) | ...);
}

/**
 * Disable one or more AHB peripheral clocks.
 *
 * e.g. Clk::enable<Clk::Ahb::fsmc, Clk::Ahb::sdio>();
 *
 * @tparam peripheral
 */
template <Ahb... peripheral>
inline __attribute__((always_inline))
void disable()
{
    RCC->AHBENR &= ~((1u << Libp::enumBaseT(peripheral)) | ...);
}

/**
 * Reset one or more APB1 peripherals.
 *
 * e.g. Clk::reset<Clk::Apb1::tim2, Clk::Apb1::spi2>();
 *
 * @tparam peripheral
 */
template <Apb1... peripheral>
inline __attribute__((always_inline))
void reset()
{
    RCC->APB1RSTR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
    RCC->APB1RSTR = 0;
}

/**
 * Reset one or more APB2 peripherals.
 *
 * e.g. Clk::reset<Clk::Apb2::afio, Clk::Apb2::iopa>();
 *
 * @tparam peripheral
 */
template <Apb2... peripheral>
inline __attribute__((always_inline))
void reset()
{
    RCC->APB2RSTR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
    RCC->APB2RSTR = 0;
}

/**
 * Reset one or more AHB peripherals.
 *
 * e.g. Clk::reset<Clk::Ahb::fsma, Clk::Ahb::sdio>();
 *
 * @tparam peripheral
 */
// AHBRSTR is not included in RCC_TypeDef
/*template <Ahb... peripheral>
inline
void reset()
{
    RCC->AHBRSTR |= ((1u << Libp::enumBaseT(peripheral)) | ...);
    RCC->AHBRSTR = 0;
}*/


/// See `setSysClk`
enum class SysClkSrc : uint8_t {
    ext_high_speed_clk, ///< External High-speed Clock (HSE bypass)
    ext_high_speed_osc, ///< External High-speed Crystal/Resonator (HSE crystal)
    int_high_speed,     ///< Internal High-speed Oscillator (HSI) 8 MHz
    pll                 ///< PLL fed by HSI/HSE
};

/// See `setSysClk`
enum class PllSrc : uint8_t {
    hsi_div2,
    hse,
    hse_div2
};

/**
 * Set the SYSCLK source.
 *
 * If PLL is to be used, the PLL source must be setup first (i.e. call this
 * function twice).
 *
 * e.g. Set PLL 72 MHz (assuming external 8 MHz crystal)
 *
 * // Enable external oscillator
 * setSysClk(SysClkSrc::ext_high_speed_osc, 2);
 *
 * // Enable PLL using external oscillator * 9
 * setSysClk(SysClkSrc::pll, 2, PllSrc::hse, 9);
 *
 * Flash prefetch will be enabled, half cycle access disabled.
 *
 * Notes (from datasheet):
 * -----------------------
 * - When the HSI is used as a PLL clock input, the maximum system clock
 *   frequency that can be achieved is 64 MHz.
 * - For the USB function to be available, both HSE and PLL must be enabled,
 *   with USBCLK running at 48 MHz.
 * - To have an ADC conversion time of 1 μs, APB2 must be at 14 MHz, 28 MHz or
 *   56 MH.
 *
 * @param source
 * @param flash_latency 0: SysClk <  24 MHz
 *                      1: SysClk <  48 MHz
 *                      2: SysClk <= 72 MHz.
 * @param pll_source Only used if `source==SysClkSrc::pll`
 * @param pll_multiplier 2->16. Only used if `source==SysClkSrc::pll`
 */
inline
void setSysClk(SysClkSrc source, uint8_t flash_latency, PllSrc pll_source = PllSrc::hse, uint8_t pll_multiplier = 1)
{
    /*
     * SYSCLK can be driven by three primary sources:
     * - HSI - high speed internal (8 MHz)
     * - HSE - high speed external (ext. crystal/resonator or ext. clock - max
     *         25 MHz)
     * - PLL - phase locked loop
     *
     * and two secondary:
     * - LSI - low speed internal (~40 kHz)
     * - LSE - low speed external (ext. crystal - 4->16 MHz)
     */

    // Adjust flash latency, set prefetch prior to clock change
    // Not clear from docs what benefit half cycle access provides when using
    // low clock speeds. Keeping disabled
    Libp::Bits::setBits(
            FLASH->ACR,
            FLASH_ACR_PRFTBE | FLASH_ACR_HLFCYA | FLASH_ACR_LATENCY,
            FLASH_ACR_PRFTBE | flash_latency);

    // Temporarily switch to SYSCLK = HSI
    uint32_t saved_rcc_cr_hsion_state = RCC->CR & RCC_CR_HSION;
    if ( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI ) {
        RCC->CR |= RCC_CR_HSION;                // Enable HSI oscillator
        Libp::Bits::setBits(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
        while ( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI )
            ;                                   // Wait for SYSCLK == HSI
    }
    if (source == SysClkSrc::int_high_speed)
        return;

    switch (source) {
    case SysClkSrc::ext_high_speed_clk:
        RCC->CR &= ~RCC_CR_HSEON;                // Disable HSE oscillator
        RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON; // Enable high speed external clock bypass
        //RCC->CFGR |= RCC_CFGR_SW_HSE;            // SYSCLK = HSE
        Libp::Bits::setBits(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);
        while (!(RCC->CFGR & RCC_CFGR_SWS_HSE))
            ;                                    // Wait for SYSCLK == HSE
        break;

    case SysClkSrc::ext_high_speed_osc:
        RCC->CR |= RCC_CR_HSEON;                          // Enable HSE oscillator
        while (!(RCC->CR & RCC_CR_HSERDY))
            ;                                             // Wait for HSE oscillator ready
        Libp::Bits::setBits(
                RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE); // SYSCLK = HSE
        while (!(RCC->CFGR & RCC_CFGR_SWS_HSE))
            ;                                             // Wait for SYSCLK == HSE
        break;

    case SysClkSrc::pll:
        RCC->CR &= ~RCC_CR_PLLON;                // Disable PLL
        if (pll_source == PllSrc::hsi_div2) {
            RCC->CFGR &= ~RCC_CFGR_PLLSRC;       // Input = HSI div/2
        }
        else {
            RCC->CFGR |= RCC_CFGR_PLLSRC;
            if (pll_source == PllSrc::hse_div2)
                RCC->CFGR |= RCC_CFGR_PLLXTPRE;  // Input = HSE div/2
            else
                RCC->CFGR &= ~RCC_CFGR_PLLXTPRE; // Input = HSE
        }
        Libp::Bits::setBits(
                RCC->CFGR, RCC_CFGR_PLLMULL_Msk,
                (pll_multiplier - 2) << RCC_CFGR_PLLMULL_Pos);

        RCC->CR |= RCC_CR_PLLON;                 // Enable PLL
        while (!(RCC->CR & RCC_CR_PLLRDY))
            ;                                    // Wait for PLL ready

        Libp::Bits::setBits(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); // SYSCLK = PLL
        while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
            ;                                    // Wait for SYSCLK == PLL
        break;

    case SysClkSrc::int_high_speed:
        break;
    }

    // Restore HSI ON state
    RCC->CR = (RCC->CR & ~RCC_CR_HSION) | saved_rcc_cr_hsion_state;

    // Update CMSIS SystemCoreClock var
    SystemCoreClockUpdate();
}

/// See `setHClk`
enum class AhbPrescaler : uint8_t {
    div1 = 1,
    div2 = 8,
    div4 = 9,
    div8 = 10,
    div16 = 11,
    div64 = 12,
    div128 = 13,
    div256 = 14,
    div512 = 15
};
/// See `setPeripheralClk`
enum class ApbPrescaler : uint8_t {
    div1 = 1,
    div2 = 4,
    div4 = 5,
    div8 = 6,
    div16 = 7
};
/// See `setPeripheralClk`
enum class AdcPrescaler : uint8_t {
    div2 = 0,
    div4 = 1,
    div6 = 2,
    div8 = 3,
};

// TODO: test
/**
 * Set the AHB prescaler for HCLK.
 *
 * HCLK = SYSCLK / this prescaler.
 *
 * @param ahb_pscale
 */
inline
void setHClk(AhbPrescaler ahb_pscale)
{
    Libp::Bits::setBits(RCC->CFGR, RCC_CFGR_HPRE, Libp::enumBaseT(ahb_pscale) << RCC_CFGR_HPRE_Pos);
    // Update CMSIS SystemCoreClock var
    SystemCoreClockUpdate();
}


// TODO: test
/**
 * Set the peripheral clock prescalers.
 *
 * PCLK1 = HCLK / apb1 prescaler (36 MHz max)
 * PCLK2 = HCLK / apb2 prescaler (72 MHz max)
 * ADCCLK = PCLK2 / adc prescaler (14 MHz max)
 *
 * @param apb1_pscale prescaler for the low-speed peripheral clock PCLK1.
 * @param apb2_pscale prescaler for the high-speed peripheral clock PCLK2.
 * @param adc_pscale prescaler for the adc clock ADCCLK.
 */
inline
void setPeripheralClk(ApbPrescaler apb1_pscale, ApbPrescaler apb2_pscale, AdcPrescaler adc_pscale = AdcPrescaler::div2)
{
    Libp::Bits::setBits(RCC->CFGR,
            RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE,
              (Libp::enumBaseT(apb1_pscale) << RCC_CFGR_PPRE1_Pos)
            | (Libp::enumBaseT(apb2_pscale) << RCC_CFGR_PPRE2_Pos)
            | (Libp::enumBaseT(adc_pscale) << RCC_CFGR_ADCPRE_Pos) );
}

// SystemCoreClock is SYSCLK / AHB Prescale = HClock

/**
 * @return the current PCLK1 frequency in Hertz
 */
inline
uint32_t getPClk1()
{
    uint8_t prescale = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    switch (static_cast<ApbPrescaler>(prescale)) {
    case ApbPrescaler::div2: return SystemCoreClock / 2;
    case ApbPrescaler::div4: return SystemCoreClock / 4;
    case ApbPrescaler::div8: return SystemCoreClock / 8;
    case ApbPrescaler::div16: return SystemCoreClock / 16;
    default: return SystemCoreClock; // 0xx = not divided
    }
}

/**
 * @return the current PCLK2 frequency in Hertz
 */
inline
uint32_t getPClk2()
{
    uint8_t prescale = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    switch (static_cast<ApbPrescaler>(prescale)) {
    case ApbPrescaler::div2: return SystemCoreClock / 2;
    case ApbPrescaler::div4: return SystemCoreClock / 4;
    case ApbPrescaler::div8: return SystemCoreClock / 8;
    case ApbPrescaler::div16: return SystemCoreClock / 16;
    default: return SystemCoreClock; // 0xx = not divided
    }
}

// TODO: test
/**
 * @return the current ADC clock frequency in Hertz
 */
inline
uint32_t getAdcClk()
{
    uint8_t prescale = (RCC->CFGR & RCC_CFGR_ADCPRE) >> RCC_CFGR_ADCPRE_Pos;
    return SystemCoreClock >> prescale;
}

} // namespace LibpStm32::Clk

#endif /* LIB_LIBPEKIN_STM32_CLOCK_STM32_H_ */
