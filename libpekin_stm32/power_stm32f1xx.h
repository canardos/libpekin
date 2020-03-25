/**
 * Sleep, stop, standby operation functionality for STM32f1xx MCUs.
 *
 */
#ifndef LIB_LIBPEKIN_STM32_POWER_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_POWER_STM32F1XX_H_

#include "libpekin_stm32_hal.h"
#include "bits.h"

namespace LibpStm32 {

inline __attribute__((always_inline))
void sleepOnExitEnable()
{
    // use for ISR driven app without main
    Libp::Bits::setBit(SCB->SCR, SCB_SCR_SLEEPONEXIT_Pos);
}
inline __attribute__((always_inline))
void sleepOnExitDisable()
{
    Libp::Bits::clearBit(SCB->SCR, SCB_SCR_SLEEPONEXIT_Pos);
}
inline __attribute__((always_inline))
void deepSleepEnable()
{
    Libp::Bits::setBit(SCB->SCR, SCB_SCR_SLEEPDEEP_Pos);
}
inline __attribute__((always_inline))
void deepSleepDisable()
{
    Libp::Bits::clearBit(SCB->SCR, SCB_SCR_SLEEPDEEP_Pos);
}
inline __attribute__((always_inline))
void deepSleepVregOn()
{
    Libp::Bits::clearBit(PWR->CR, PWR_CR_LPDS_Pos);
}
inline __attribute__((always_inline))
void deepSleepVregOff()
{
    Libp::Bits::setBit(PWR->CR, PWR_CR_LPDS_Pos);
}

/*
 * AN2629 Application note
 *
 * Three different clock sources can be used to drive the system clock
 * (SYSCLK):
 * - HSI oscillator clock
 * - HSE oscillator clock
 * - PLL clock
 *
 * The devices have the two secondary clock sources listed below:
 * - 40 kHz low-speed internal RC (LSI RC) that drives the independent watchdog
 *   and optionally the RTC used for Auto-Wakeup from Stop/Standby mode.
 * - 32.768 kHz low speed external crystal (LSE crystal) that optionally drives
 *   the real-time clock (RTCCLK)
 *
 * Each clock source can be switched on or off independently when not used, to
 * optimize power consumption.
 */

/**
 * Enter sleep mode.
 *
 */
inline __attribute__((always_inline))
void pwrCtrlSleep()
{
    deepSleepDisable();
    sleepOnExitDisable();
    __WFI();
}

// TODO: disable systick

/**
 * Enter stop mode.
 *
 * Clock setup is saved and restored on wake, but since interrupt handlers are
 * executed immediately on wake, the clock setup will not be restored until
 * after they complete.
 *
 * Note:
 * All EXTI Line pending bits (in Pending register (EXTI_PR)) and RTC Alarm
 * flag must be reset. Otherwise, the Stop mode entry procedure is ignored and
 * program execution continues.
 *
 * @param vreg_low_pwr put internal voltage regulator in low power mode.
 */
inline __attribute__((always_inline))
void pwrCtrlStop(bool vreg_low_pwr = true/*, bool rtc = false, bool lsi = false, bool lse = false*/)
{
    //* @param rtc real-time clock enabled during stop
    //* @param lsi internal RC oscillator enabled during stop
    //* @param lse external 32.768 kHz oscillator enabled during stop


    Libp::Bits::clearBit(PWR->CR, PWR_CR_PDDS_Pos); // stop mode

    deepSleepVregOff();
    deepSleepEnable();
    sleepOnExitDisable();

    // save state
//    const uint32_t rcc_sw = RCC->CFGR & RCC_CFGR_SW;
    const uint32_t rcc_cfgr = RCC->CFGR;
    const uint32_t rcc_cr = RCC->CR;
    const uint32_t systick = SysTick->CTRL & SysTick_CTRL_TICKINT_Msk;

    // stop
    Libp::Bits::clearBit(SysTick->CTRL, SysTick_CTRL_TICKINT_Pos);  // SysTick_CTRL_ENABLE_Pos
    __WFI();

    // restore state
    //Libp::Bits::setBits(RCC->CFGR, RCC_CFGR_SW, rcc_sw);
    // TODO: any harm writing read-only hardware status bits?
    RCC->CFGR = rcc_cfgr;
    RCC->CR = rcc_cr;


    Libp::Bits::setBits(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk, systick);  // SysTick_CTRL_ENABLE_Pos

    // TODO: do we need to wait for clock status? (SWS)
//    if (reenable_systick)
  //      Libp::Bits::setBit(SysTick->CTRL, SysTick_CTRL_TICKINT_Pos);  // SysTick_CTRL_ENABLE_Pos
    //if (reenable_systick)

    deepSleepDisable();
}

// The ADC or DAC can also consume power during the Stop mode,
// unless they are disabled before entering it. To disable them,
// the ADON bit in the ADC_CR2 register and the ENx bit in the DAC_CR register must both be written to 0.

inline __attribute__((always_inline))
void pwrCtrlStandby()
{

}

} // namespace LibpStm32

#endif /* LIB_LIBPEKIN_STM32_POWER_STM32F1XX_H_ */
