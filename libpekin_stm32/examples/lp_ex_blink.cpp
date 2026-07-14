#include <cstdint>

#include "lp_clock_stm32f1xx.h"
#include "lp_libpekin_stm32_hal.h"
#include "lp_pins_stm32f1xx.h"
#include "libpekin.h"


// Change port and pin here for your device
// Don't forget to enable the correct GPIO clock below
libp_stm32::PinC<13> pcb_led;

static void initSysClock()
{
    using namespace libp_stm32;

    // Setup HSE source
    clk::setSysClk(clk::SysClkSrc::ext_high_speed_osc, 2, clk::PllSrc::hse, 2);

    // Setup PLL as SysClock with HSE * 9 as source (72 MHz)
    clk::setSysClk(clk::SysClkSrc::pll, 2, clk::PllSrc::hse, 9);

    // APB1 can't exceed 36 MHz
    clk::setPeripheralClk(clk::ApbPrescaler::div2, clk::ApbPrescaler::div1);
}

static void initPeripherals()
{
    using namespace libp_stm32;

    clk::enable<clk::Apb2::iopc>();
    pcb_led.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
}


// Rename to main
int rename_me_to_main()
{
    initSysClock();
    initPeripherals();
    libp::lpInitTimers();
    while (true) {
        pcb_led.toggle();
        libp::delayMs(500);
    }
}
