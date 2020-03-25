#include <cstdint>
#include "libpekin_hal.h"
#include "libpekin_stm32_hal.h"
#include "core_cm3.h"

static volatile uint32_t ticks_ms_ = 0;
static Libp::SysTickHandler client_systick_handler_ = nullptr;
#ifdef __cplusplus
extern "C" {
#endif
void SysTick_Handler(void)
{
    // This is not atomic, but the read/store
    // is and this is the only write operation.
    ticks_ms_++;
    if (client_systick_handler_)
        client_systick_handler_();
}
#ifdef __cplusplus
}
#endif

namespace Libp {

static uint16_t ticks_per_us;
static uint16_t ns_per_tick;

static constexpr uint32_t systick_irq_priority = 0;

/**
 * Note:
 * `getMillis`/`delayMs` functions rely on the SysTick interrupt. Priority is
 * set to `0` here, but may be modified after this function returns. Take care
 * when relying on these functions within ISRs of higher or equal priority.
 *
 * ns/us functions use the DWT->CYCCNT counter and are thus independent of
 * interrupt handling.
 *
 * @param handler function to call on each SysTick. Pass `HAL_IncTick` if using
 *                STM32 HAL functions that rely on HAL SysTick functionality.
 *
 * @return
 */
bool libpekinInitTimers(SysTickHandler handler)
{
    // ** ms **
    client_systick_handler_ = handler;
    bool success = true;
    // TODO: return 1 on fail
    success = !SysTick_Config(SystemCoreClock / 1000);
    // SysTick_Config sets priority at lowest level
    NVIC_SetPriority (SysTick_IRQn, systick_irq_priority);

    // ** ns/us **
    // Start DWT (Debug Watchpoint Trace) counter

    // Cache freq
    ticks_per_us = SystemCoreClock / 1'000'000;
    ns_per_tick = 1'000'000'000 / SystemCoreClock;
    // Enable TRC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Unlock software access to DWT
    //DWT->LAR = 0xC5ACCE55;

    // Reset cycle count
    DWT->CYCCNT = 0;

    // Cycle Count Event Enable
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Wait for counter to tick over
    // TODO: Change this to other instruction - NOP is not guaranteed to be time consuming
    //       Ensure 3 instructions is sufficient on faster hardware
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    return success && DWT->CYCCNT;
}

/// Warning: Will overflow every ~49 days
uint32_t getMillis()
{
    return ticks_ms_;
}


/**
 * @param millis delay in millis (max = (2^32 - 1)/1000)
 */
void delayMs(uint32_t millis)
{
    const uint32_t ticks_ms_start = ticks_ms_;
    // will still work on overflow
    while ( (ticks_ms_ - ticks_ms_start) < millis)
        ;
}

/**
 * @param microseconds delay in microseconds (max = 2^32 - 1)
 */
void delayUs(uint32_t micros)
{
    // TODO: document min/max delay
    const uint32_t clk_cycle_start = DWT->CYCCNT;
    const uint32_t clk_cycle_delay = micros * ticks_per_us;
    // will still work on overflow
    while ( (DWT->CYCCNT - clk_cycle_start) < clk_cycle_delay)
        ;
}

/**
 * Not accurate and intended only for situations where a delay < 1uS is
 * desired.
 *
 * For a 72 MHz system for example, the minimum achievable delay is probably
 * around 300ns:
 *
 * Assuming the function isn't inlined, call overhead to get into the function
 * is at least several cycles (push regs etc.). The UDIV below will add another
 * 10ish cycles, so assuming 20 cycles minimum, that's ~280ns on a 72 MHz
 * system.
 *
 * @param nanos
 */
void delayNs(uint32_t nanos)
{
    const uint32_t clk_cycle_start = DWT->CYCCNT;
    const uint32_t clk_cycle_delay = nanos / ns_per_tick;
    // will still work on overflow
    while ( (DWT->CYCCNT - clk_cycle_start) < clk_cycle_delay)
        ;
}

void getDeviceId(uint32_t (&id)[3])
{
  id[0] = *reinterpret_cast<uint32_t*>(UID_BASE);
  id[1] = *reinterpret_cast<uint32_t*>(UID_BASE + 4U);
  id[2] = *reinterpret_cast<uint32_t*>(UID_BASE + 8U);
}

} // namespace Libp
