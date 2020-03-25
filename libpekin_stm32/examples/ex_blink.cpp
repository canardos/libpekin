#include <cstdint>

#include "libpekin.h"
#include "libpekin_stm32_hal.h"

#include "clock_stm32f1xx.h"
#include "pins_stm32f1xx.h"

// Change port and pin here for your device
// Don't forget to enable the correct GPIO clock below
LibpStm32::PinC<13> pcb_led;

static void initSysClock()
{
    using namespace LibpStm32;

    // Setup HSE source
    Clk::setSysClk(Clk::SysClkSrc::ext_high_speed_osc, 2, Clk::PllSrc::hse, 2);

    // Setup PLL as SysClock with HSE * 9 as source (72 MHz)
    Clk::setSysClk(Clk::SysClkSrc::pll, 2, Clk::PllSrc::hse, 9);

    // APB1 can't exceed 36 MHz
    Clk::setPeripheralClk(Clk::ApbPrescaler::div2, Clk::ApbPrescaler::div1);
}

static void initPeripherals()
{
    using namespace LibpStm32;

    Clk::enable<Clk::Apb2::iopc>();
    pcb_led.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
}


int main()
{
    initSysClock();
    initPeripherals();
    Libp::lpInitTimers();
    while (true) {
        pcb_led.toggle();
        Libp::delayMs(500);
    }
}
