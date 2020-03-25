/**
 * STM32F1xx timer functionality. WIP
 * - No external clock
 * - No advanced timer func, ext trigger, input compare etc.
 * - Most functionality not well tested.
 *
 * Examples:
 *
 * // --------------------------
 * // Basic timer, no interrupts
 * // --------------------------
 *
 * void flashLed();
 *
 * using namespace LibpStm32
 *
 * Clk::enable<Clk::Apb1::tim2>();
 * Tim::BasicTimer<TIM2_BASE> timer;
 *
 * timer.initBasic(72'000'000, false);
 * timer.enable();
 * while (true) {
 *     while (!timer.getUpdFlag())
 *          ;
 *     timer.clearUpdFlag();
 *     flashLed();
 * }
 *
 * // --------------------------
 * // Basic timer, interrupt
 * // --------------------------
 *
 * void flashLed();
 *
 * using namespace LibpStm32
 *
 * Clk::enable<Clk::Apb1::tim2>();
 * Tim::BasicTimer<TIM2_BASE> timer;
 *
 * timer.initBasic(72'000'000, false);
 * timer.enableEvents(true, false);
 * timer.enableIrq();
 * timer.enable();
 * while (true)
 *     ;
 *
 * extern "C" {
 * void TIM2_IRQHandler(void)
 * {
 *     flashLed();
 *     timer.clearPendingIrqBit();
 * }
 * }
 *
 * // --------------------------
 * // Basic timer, output
 * // --------------------------
 *
 * using namespace Libp
 * using namespace LibpStm32
 *
 * Clk::enable<Clk::Apb1::tim2>();
 * Tim::BasicTimer<TIM2_BASE> timer;
 *
 * // TIM2 CH2 = Pin A1
 *
 * Clk::enable<Clk::Apb2::iopa>();
 * PinA<1>::.setAsOutput(OutputMode::alt_pushpull, OutputSpeed::low);
 *
 * timer.initBasic(72'000'000, false);
 *
 * timer.setupOutputChannel(
 *     Tim::Channel::ch2,
 *     Tim::OutputCompareMode::toggle,
 *     Tim::CaptureComparePolarity::oc_active_high_ic_non_inv);
 *
 * timer.enable();
 *
 * // Pin A1 will toggle every 72'000'000 cycles
 *
 * // --------------------------
 * // Basic timer, PWM output
 * // --------------------------
 *
 * using namespace LibpStm32
 *
 * Clk::enable<Clk::Apb1::tim2>();
 * Tim::BasicTimer<TIM2_BASE> timer;
 *
 * // TIM2 CH2 = Pin A1
 *
 * Clk::enable<Clk::Apb2::iopa>();
 * PinA<1>::.setAsOutput(OutputMode::alt_pushpull, OutputSpeed::low);
 *
 * timer.initPwm(Channel::ch2, 100, 50, true);
 * timer.enable();
 *
 * // Pulse LED connected on A1
 *
 * while (true) {
 *     uint8_t i = 1;
 *     while (i++ < 100) {
 *         timer.setCompare(Channel::ch2, i);
 *         Libp::delayMs(5);
 *     }
 *     while (i-- > 0) {
 *         timer.setCompare(Channel::ch2, i);
 *         Libp::delayMs(5);
 *     }
 * }
 *
 */


/*
 * Advanced-configuration timers like TIM1 and TIM8 among others.
 * General-purpose configuration timers like TIM2 and TIM3 among others
 * Lite-configuration timers like TIM9, TIM10, TIM12 and TIM16 among others
 * Basic-configuration timers like TIM6 and TIM7 among others.
 *
 * Counter Register (TIMx_CNT)
 * Prescaler Register (TIMx_PSC)
 * Auto-Reload Register (TIMx_ARR)
 *
 * STM32 Timer Modes
 * -----------------
 * Timer
 * Counter
 *
 * xPWM
 * xOne-pulse
 *
 * PWM advanced
 * xInput capture
 * Output compare
 * Encoder
 * Timer gate
 * Timer DMA burst
 * IRTIM infrared
 *
 * Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)*(RCR + 1))
 *
 */
#ifndef LIB_LIBPEKIN_STM32_TIMER_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_TIMER_STM32F1XX_H_

#include <cstdint>
#include <tuple>
#include "bits.h"
#include "libpekin.h"
#include "libpekin_stm32_hal.h"

static_assert(TIM1_BASE > 0, "STM32 CMSIS header must be included before this file");

namespace LibpStm32::Tim {

/**
 * TRGO Trigger Type
 *
 * When a timer is selected as a master timer, the corresponding trigger output
 * signal (TRGO) is used by the slave internal trigger (when configured).
 */
enum class TrgoMode : uint16_t {
    reset          = 0b000 << TIM_CR2_MMS_Pos, ///< TRGO output on UG bit from EGR register
    enable         = 0b001 << TIM_CR2_MMS_Pos, ///< TRGO output on counter enable signal (use
                                               ///< to start multiple timer simultaneously)
    update         = 0b010 << TIM_CR2_MMS_Pos, ///< Update event used as trigger output (e.g.
                                               ///< master timer used as prescaler for a slave)
    compare_pulse  = 0b011 << TIM_CR2_MMS_Pos, ///< TRGO pulse when CC1IF flag is to be set as
                                               ///< as soon as a capture or compare match occurs
    compare_oc1ref = 0b100 << TIM_CR2_MMS_Pos, ///< OC1REF signal used as trigger output.
    compare_oc2ref = 0b101 << TIM_CR2_MMS_Pos, ///< OC2REF signal used as trigger output.
    compare_oc3ref = 0b110 << TIM_CR2_MMS_Pos, ///< OC3REF signal used as trigger output.
    compare_oc4ref = 0b111 << TIM_CR2_MMS_Pos  ///< OC4REF signal used as trigger output.
};

// The clock of the slave timer and ADC must be enabled prior to receiving events from
// the master timer, and must not be changed on-the-fly while triggers are received from
// the master timer

/*enum class ClockSource : uint8_t {
    // DO NOT REORDER
    /// internal
    internal,
    /// external input pin (TIx)
    ext_mode1,
    /// external trigger input (ETR)
    ext_mode2,
    /// Internal trigger inputs (ITRx)
    int_trigger
};*/

enum class CountMode : uint16_t {
    up              = 0b00 << TIM_CR1_CMS_Pos | 0b0 << TIM_CR1_DIR_Pos,
    down            = 0b00 << TIM_CR1_CMS_Pos | 0b1 << TIM_CR1_DIR_Pos,
    center_aligned1 = 0b01 << TIM_CR1_CMS_Pos,
    center_aligned2 = 0b10 << TIM_CR1_CMS_Pos,
    center_aligned3 = 0b11 << TIM_CR1_CMS_Pos
};

enum class CcDma : uint8_t {
    req_on_ccx_event = 0,
    req_on_update_event = 1
};

/// Ratio of digital filters sampling clock to the timer clock
enum class ClockDiv : uint16_t {
    x1 = 0b00 << TIM_CR1_CKD_Pos,
    x2 = 0b01 << TIM_CR1_CKD_Pos,
    x4 = 0b10 << TIM_CR1_CKD_Pos
};

/// Determines OCxRef behavior based on CNT value
enum class OutputCompareMode : uint8_t {
    frozen                = 0b000, ///!< CCRx/CNT compare has no effect on output
    ch1_active_on_match   = 0b001, ///!< OCxRef high when CNT==CCRx
    ch1_inactive_on_match = 0b010, ///!< OCxRef low when CNT==CCRx
    toggle                = 0b011, ///!< OCxRef toggle when CNT==CCRx
    force_inactive        = 0b100, ///!< OCxRef forced low independent of CNT
    force_active          = 0b101, ///!< OCxRef forced high independent of CNT
    pwm_mode1             = 0b110, ///!< up-counting: high when CNT < CCRx, otherwise low
                                   ///!< dn-counting:  low when CNT > CCRx, otherwise high
    pwm_mode2             = 0b111  ///!< up-counting:  low when CNT < CCRx, otherwise high
                                   ///!< dn-counting: high when CNT > CCRx, otherwise low
};

/// Polarity of output compare pin when channel is configured as output, or
/// edge detection type for input compare pin when channel is configured as
/// input.
enum class CaptureComparePolarity : uint8_t {

    /// OC pin active high / non-inverted: capture is done on a rising edge
    /// of IC pin. When used as external trigger, IC pin is non-inverted.
    oc_active_high_ic_non_inv = 0,

    /// OC pin active low / inverted: capture is done on a falling edge of
    /// IC pin. When used as external trigger, IC pin is inverted.
    ///
    oc_active_low_ic_inv = 1,
    /// Disable output/input compare
    //oc_ic_disable = 2
};

enum class OcIdleState : uint8_t {
    low = 0,
    high = 1
};

enum class Channel : uint8_t {
    ch1 = 0x00,
    ch2 = 0x04,
    ch3 = 0x08,
    ch4 = 0x0c
};

/**
 *
 * @tparam base_addr_
 */
template <uint32_t base_addr_>
class BasicTimer {
    static inline TIM_TypeDef* const tim_ = reinterpret_cast<TIM_TypeDef*>(base_addr_);

//
//
//
//

// Counter is clocked by prescalar output CK_CNT

/*
 *
 * General:
 *
 * Basic:
 *
 *
 * Basic mode:
 *
 * Up mode:
 * - Counts from 0 to ARR value, generates overflow, and starts again at 0
 * - Update event is generated on each overflow if UG bit set in EGR
 *
 * Down mode:
 * - Same as up mode but starts at ARR value and countries down to 0.
 *
 * Up/Down (center aligned) mode:
 * - Counts from 0 to ARR-1, generates overflow event, then counts down from
 *   ARR to 0, generates underflow event, repeat.
 *
 * Clock:
 * - Internal        : CK_INT. Typical mode.
 * - External mode 1 : External input pin.
 * - Extrenal mode 2 : External trigger input.
 * - Internal trigger: Use one timer as prescalar for another.
 *
 * PWM:
 *
 *
 * Registers:
 * - PSC prescalar
 * - CNT counter
 * - ARR auto-reload
 *
 * - ARR and PSC can be updated while running
 *
 * Output Mode:
 * When configured in output mode, the content of the TIMx_CCRy channel
 * register is compared to the timer counter. Timer channel internal output
 * (OCyREF) is set/cleard based on the comparison result and output mode.
 * OCyREF affects the timer output pins based on the DTG/output settings.
 *
 *
 */

public:
    /**
     * Initialize timer with basic settings:
     * - Up counting
     * - TRGO = update
     * - No output compare
     *
     * `enable` must be called after initializing.
     *
     * @param period timer period in number of clock cycles
     * @param one_pulse Counter stops counting at the next update event (CEN is
     *                  cleared)
     */
    static void initBasic(uint32_t period, bool one_pulse = false)
    {
        tim_->CR1 = TIM_CR1_UDIS;    // disable update events
        tim_->SR = 0;                // clear events
        tim_->CR2 = Libp::enumBaseT(TrgoMode::update);

        setPeriod(period);
        tim_->CR1 = one_pulse << TIM_CR1_OPM_Pos;

        // Generate an update event to re-init counters
        tim_->EGR  |= TIM_EGR_UG;
        clearUpdFlag();
    }

    /**
     * Initialize timer with extended settings:
     *
     * `enable` must be called after initializing.
     *
     * @param trgo_mode
     * @param count_mode
     * @param clock_div
     * @param one_pulse Counter stops counting at the next update event (CEN is
     *                  cleared)
     */
    static void init(TrgoMode trgo_mode, CountMode count_mode, ClockDiv clock_div, bool one_pulse = false)
    {
        tim_->CR1 |= TIM_CR1_UDIS;    // disable updates
        tim_->SR = 0;                // clear events
        tim_->CR2 = Libp::enumBaseT(trgo_mode);
        // TODO: count_mode is read only when the timer is configured in Center-aligned mode or Encoder mode
        tim_->CR1 = Libp::enumBaseT(count_mode) | Libp::enumBaseT(clock_div) | one_pulse << TIM_CR1_OPM_Pos;
        // Generate an update event to re-init counters
        tim_->EGR  |= TIM_EGR_UG;
        clearUpdFlag();
        // ADV tim1/8
        /* Set the Repetition Counter value */
            //TIMx->RCR = Structure->RepetitionCounter;
        /* Generate an update event to reload the Prescaler
            and the repetition counter(only for TIM1 and TIM8) value immediatly */
         //TIMx->EGR = TIM_EGR_UG;
        // slave mode ctrl reg
        // tim_->SMCR not used in basic timer - slave connection setup
    }


    // Output event happens when the active capture/compare reg = CNT.
    // CNT resets? when it hits the reload value

    /**
     * Initialize timer in PWM mode.
     *
     * `enable` must be called after initializing.
     *
     * @param channel output channel
     * @param period in cycles
     * @param active_period in cycles
     * @param high_to_low if true, output channel is high for active_period
     *                    cycles, then low until end of period (PWM mode 1).
     *                    If false, PWM mode 2 (low to high).
     * @param one_pulse Counter stops counting at the next update event (CEN is
     *                  cleared)
     */
    static void initPwm(Channel channel, uint32_t period, uint32_t active_period, bool high_to_low, bool one_pulse = false)
    {
        initBasic(period, one_pulse);
        // Duty cycle
        // psc adjustment already done in setCompare func
        //if (tim_->PSC)
        //    active_period /= (tim_->PSC + 1); // CK_CNT = Fck_psc / (PSC + 1)
            //active_period /= (tim_->PSC << 1);
        setCompare(channel, active_period);

        setupOutputChannel(
                channel,
                high_to_low ? OutputCompareMode::pwm_mode1 : OutputCompareMode::pwm_mode2);
    }

    /**
     * En/disable IRQ and DMA events
     *
     * @param irq_en enable update interrupt
     * @param dma_en enable update DMA req.
     */
    static void enableEvents(bool irq_en, bool dma_en)
    {
        Libp::Bits::setBits(
                tim_->DIER,
                TIM_DIER_UDE_Msk | TIM_DIER_UIE_Msk,
                (dma_en << TIM_DIER_UDE_Pos) | (irq_en << TIM_DIER_UIE_Pos));
    }

    /**
     * TODO
     *
     * @param channel
     * @param oc_mode
     * @param icoc_polarity
     */
    static void setupOutputChannel(Channel channel, OutputCompareMode oc_mode,
            CaptureComparePolarity icoc_polarity = CaptureComparePolarity::oc_active_high_ic_non_inv)
    {
        // active/inactive/toggle
        // https://www.cpe.ku.ac.th/~cpj/204322/slides/04-timers.pdf

        constexpr bool fast_en = false;
        //constexpr bool preload_en = false;
        const bool preload_en = (oc_mode == OutputCompareMode::pwm_mode1) || (oc_mode == OutputCompareMode::pwm_mode2);

        switch (channel) {
        case Channel::ch1:
            // Channel must be disabled to modify CCxS (i.e. set output mode)
            tim_->CCER &= ~TIM_CCER_CC1E_Msk;
            tim_->CCMR1 = (tim_->CCMR1 & ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_CC1S_Msk))
                    | (Libp::enumBaseT(oc_mode) << TIM_CCMR1_OC1M_Pos);     // output mode
            tim_->CCER  = (tim_->CCER & ~TIM_CCER_CC1P_Msk)
                    | (Libp::enumBaseT(icoc_polarity) << TIM_CCER_CC1P_Pos) // output channel polarity
                    | TIM_CCER_CC1E_Msk;                                // re-enable output
            break;
        case Channel::ch2:
            tim_->CCER &= ~TIM_CCER_CC2E_Msk;
            Libp::Bits::setBits(tim_->CCMR1,
                    TIM_CCMR1_OC2M_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2PE_Msk | TIM_CCMR1_OC2FE_Msk,
                    Libp::enumBaseT(oc_mode) << TIM_CCMR1_OC2M_Pos |  // output mode
                    fast_en << TIM_CCMR1_OC2FE_Pos |              // fast enable
                    preload_en << TIM_CCMR1_OC2PE_Pos );          // preload enable

            // TODO:
            // Disable the preload feature for CCx by writing OCxPE in CCMRx register
            // for OC mode timing/active/inactive/toggle

            // for PWM modes
            //  Set the preload bit in CCMRx register and the ARPE bit in the CR1 regist

            // OC ouptut enable - set MOE bit on TIM2_BDTR
            Libp::Bits::setBits(tim_->CCER,
                    TIM_CCER_CC2P_Msk,
                    (Libp::enumBaseT(icoc_polarity) << TIM_CCER_CC2P_Pos) // output channel polarity
                    | TIM_CCER_CC2E_Msk);                             // re-enable output
            break;
        case Channel::ch3:
            tim_->CCER &= ~TIM_CCER_CC3E_Msk; // Channel must be disabled to modify CCxS (i.e. set output mode)
            tim_->CCMR2 = (tim_->CCMR2 & ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_CC3S_Msk))
                    | (Libp::enumBaseT(oc_mode) << TIM_CCMR2_OC3M_Pos);     // output mode
            tim_->CCER  = (tim_->CCER & ~TIM_CCER_CC3P_Msk)
                    | (Libp::enumBaseT(icoc_polarity) << TIM_CCER_CC3P_Pos) // output channel polarity
                    | TIM_CCER_CC3E_Msk;                                // re-enable output
            break;
        case Channel::ch4:
            tim_->CCER &= ~TIM_CCER_CC4E_Msk; // Channel must be disabled to modify CCxS (i.e. set output mode)
            tim_->CCMR2 = (tim_->CCMR2 & ~(TIM_CCMR2_OC4M_Msk | TIM_CCMR2_CC4S_Msk))
                    | (Libp::enumBaseT(oc_mode) << TIM_CCMR2_OC4M_Pos);     // output mode
            tim_->CCER  = (tim_->CCER & ~TIM_CCER_CC4P_Msk)
                    | (Libp::enumBaseT(icoc_polarity) << TIM_CCER_CC4P_Pos) // output channel polarity
                    | TIM_CCER_CC4E_Msk;                                // re-enable output
            break;
        }
    }


    /**
     * Return the CCR register for the given channel.
     *
     * @param channel
     * @return
     */
    inline __attribute__((always_inline))
    static uint32_t getRegCcr(Channel channel)
    {
        const uint32_t cr_reg_addr = base_addr_ + offsetof(TIM_TypeDef, CCR1) + Libp::enumBaseT(channel);
        return *reinterpret_cast<uint32_t*>(cr_reg_addr);
    }

    /**
     * Set the CCR register for the given channel.
     *
     * @param channel
     * @param ccr
     */
    inline __attribute__((always_inline))
    static void setRegCcr(Channel channel, uint32_t ccr)
    {
        const uint32_t cr_reg_addr = base_addr_ + offsetof(TIM_TypeDef, CCR1) + Libp::enumBaseT(channel);
        *reinterpret_cast<uint32_t*>(cr_reg_addr) = ccr;
    }


    /**
     * Set the compare value, (CCR) for the given channel, adjusted for the
     * prescalar value.
     *
     * @param channel
     * @param compare will be adjusted based on the current prescalar value
     */
    inline __attribute__((always_inline))
    static void setCompare(Channel channel, uint32_t compare)
    {
        setRegCcr(channel, compare / (tim_->PSC + 1));
        // TODO: check arithmetic

        /*const uint32_t cr_reg_addr = base_addr_ + offsetof(TIM_TypeDef, CCR1) + Libp::enumBaseT(channel);
        *reinterpret_cast<uint32_t*>(cr_reg_addr) = compare / (tim_->PSC + 1);
        */

        //*(&tim_->CCR1 + (Libp::enumBaseT(channel) >> 2u)) = compare;
    }

    /**
     * Get the compare value (CCR) for the given channel.
     *
     * The returned value is the actual CCR value multiplied by the (prescalar + 1)
     * (i.e. the actual compare value in cycles)
     *
     * @param channel
     *
     * @return
     */
    inline __attribute__((always_inline))
    static uint32_t getCompare(Channel channel)
    {
        return getRegCcr(channel) * (tim_->PSC + 1);
        /*const uint32_t cr_reg_addr = base_addr_ + offsetof(TIM_TypeDef, CCR1) + Libp::enumBaseT(channel);
        const uint32_t compare = *reinterpret_cast<uint32_t*>(cr_reg_addr);
        return compare * (tim_->PSC + 1);*/

        //return static_cast<uint16_t>(*reinterpret_cast<uint32_t*>(cr_reg_addr));
    }


    /**
     * Set the prescaler register (PSC).
     *
     * Actual counter clock frequency = F(ck_psc) / (PSC + 1)
     *
     * @param psc
     */
    inline __attribute__((always_inline))
    static void setRegPsc(uint16_t psc)
    {
        tim_->PSC = psc;
    }


    /**
     * Set the reload register (ARR).
     *
     * Counter clock frequency = F(ck_psc) / (PSC + 1), so
     * Counter period = F(ck_psc) / (PSC + 1) / ARR
     *
     * @param arr
     */
    inline __attribute__((always_inline))
    static void setRegArr(uint16_t arr)
    {
        tim_->ARR = arr;
    }


    /**
     * Sets the auto reload and prescaler registers to trigger an event every
     * `count` counter clock cycles.
     *
     * @param count
     */
    static void setPeriod(uint32_t count)
    {
        // ARR register is 16-bit only, so reduce the clock frequency with the
        // prescalar if our desired period is > 16-bit
        uint16_t prescaler = 1;
        uint32_t arr = count;
        while (arr > 0xFFFF) {
            arr = arr >> 1;
            prescaler = prescaler << 1;
        }
        // Counter clock freq = F(ck_psc) / (PSC + 1)
        tim_->PSC = prescaler - 1; // prescaler is 0 based
        tim_->ARR = arr;
    }

    static void enable()  { tim_->CR1 |= TIM_CR1_CEN;    }
    static void disable() { tim_->CR1 &= ~(TIM_CR1_CEN); }


    inline __attribute__((always_inline))
    static bool getUpdFlag() { return tim_->SR & TIM_SR_UIF; }


    inline __attribute__((always_inline))
    static void clearUpdFlag() { tim_->SR &= ~TIM_SR_UIF; }


    /// Enable IRQ line for timer (i.e. NVIC_ISER)
    /// afio clock must be enabled????
    inline __attribute__((always_inline))
    static void enableIrq()
    {
        NVIC_EnableIRQ(irqn());
    }


    /// Disable IRQ line for timer (i.e. NVIC_ICER)
    inline __attribute__((always_inline))
    static void disableIrq()
    {
        NVIC_DisableIRQ(irqn());
    }


    /// Set IRQ priority (i.e. NVIC_SetPriority)
    inline __attribute__((always_inline))
    static void setIrqPriority(uint32_t priority)
    {
        NVIC_SetPriority(irqn(), priority);
    }


    /// TIM_SR_UIF bit
    inline __attribute__((always_inline))
    static bool irqIsPending()
    {
        return tim_->SR & TIM_SR_UIF;
    }


    //// Clear Update IRQ pending bit
    inline __attribute__((always_inline))
    static void clearPendingIrqBit()
    {
        tim_->SR &= ~TIM_SR_UIF;
    }


    /// Clear IRQ pending in NVIC for this IRQ line (i.e. NVIC_ICPR)
    inline __attribute__((always_inline))
    static void clearPendingIrqLine()
    {
        //EXTI->PR |= (1<<pin_);
        // Pending state is normally cleared when mcu enters ISR..?
        //NVIC->ICPR
        NVIC_ClearPendingIRQ(irqn());
    }

    /**
     * Basic interrupts only. TIM1/8 will return update interrupt.
     *
     * @return Interrupt Number for timer.
     */
    static constexpr IRQn_Type irqn() {
        switch (base_addr_) {
// TODO: do some models have TIM1/8_IRQn?
        case TIM1_BASE: return TIM1_UP_IRQn;
        case TIM2_BASE: return TIM2_IRQn;
        case TIM3_BASE: return TIM3_IRQn;
        case TIM4_BASE: return TIM4_IRQn;
#ifdef TIM5_BASE
        case TIM5_BASE: return TIM5_IRQn;
#endif
#ifdef TIM6_BASE
        case TIM6_BASE: return TIM6_IRQn;
#endif
#ifdef TIM7_BASE
        case TIM7_BASE: return TIM7_IRQn;
#endif
#ifdef TIM8_BASE
        case TIM8_BASE: return TIM8_UP_IRQn;
#endif
        }
/*      TIM1_BRK_IRQn               = 24,     /!< TIM1 Break Interrupt
        TIM1_UP_IRQn                = 25,     /!< TIM1 Update Interrupt                                *
        TIM1_TRG_COM_IRQn           = 26,     /!< TIM1 Trigger and Commutation Interrupt               *
        TIM1_CC_IRQn                = 27,     /!< TIM1 Capture Compare Interrupt                       *
        TIM8_BRK_IRQn               = 43,     /!< TIM8 Break Interrupt                                 *
        TIM8_UP_IRQn                = 44,     /!< TIM8 Update Interrupt                                *
        TIM8_TRG_COM_IRQn           = 45,     /!< TIM8 Trigger and Commutation Interrupt               *
        TIM8_CC_IRQn                = 46,     /!< TIM8 Capture Compare Interrupt                       *
*/
    }
};


/**
 * Calculate timer prescalar value to give a desired frequency.
 *
 * If the prescaler value would exceed the maximum of 0xFFFF, it is halved
 * until valid, and a multiplier is provided. i.e.:
 *
 * tim_freq / desired_freq == prescaler * multiplier
 *
 * @param tim_freq
 * @param desired_freq
 * @return tuple(prescaler, multiplier);
 */
/*constexpr std::tuple<uint16_t, uint16_t> calcPrescaler(uint32_t tim_freq, uint32_t desired_freq)
{
    uint32_t multiplier = 1;
    uint32_t prescaler = tim_freq / desired_freq;
    while (prescaler > 0xFFFF) {
        prescaler = prescaler >> 1;
        multiplier = multiplier << 1;
    }
    return std::make_tuple(prescaler, multiplier);
}*/

} // Libp::Stm32::Tim

#endif /* LIB_LIBPEKIN_STM32_TIMER_STM32F1XX_H_ */
