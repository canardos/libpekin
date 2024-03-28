/**
 * Low level individual GPIO pin operations helpers for STM32f1xx MCUs.
 *
 * The abstractions used here should be optimized out by any reasonable
 * compiler with optimizations enabled.
 *
 * Direct register manipulation will likely be more efficient if multiple pins
 * are being modified simultaneously.
 *
 * Macro for the MCU in use (as used in the ST CMSIS headers) must be defined.
 * e.g. STM32F101xB, STM32F103xG, STM32F105xC etc.
 *
 * Note that when setting modes, only the GPIO registers are modified. If debug
 * pins such as PA13, PA14, PA15, PB3, PB4 are used, manual AFIO remapping is
 * required
 *
  __HAL_RCC_AFIO_CLK_ENABLE();

  // Disconnect JTAG-DP + SW-DP signals.
  // Warning: Need to reconnect under reset
  if ((pin == PA_13) || (pin == PA_14)) {
    __HAL_AFIO_REMAP_SWJ_DISABLE(); // JTAG-DP Disabled and SW-DP Disabled
  }
  if ((pin == PA_15) || (pin == PB_3) || (pin == PB_4)) {
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); // JTAG-DP Disabled and SW-DP enabled
  }
 *
 *
 * pinF1_DisconnectDebug
 *
 *
 * ============
 * Pin Examples
 * ============
 *
 * // ** Digital output **
 *
 * using namespace LibpStm32;
 *
 * PinB<11> led;
 * // or Pin<GPIOB_BASE, 11> led;
 *
 * Clk::enable<Clk::Apb2::iopb>();
 * led.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
 * led.set();       // high
 * led.clear();     // low
 * led.set(true);   // high again
 * led.toggle();    // low again
 *
 *
 * // ** Digital input, no interrupt **
 *
 * PinA<3> button;
 * // or Pin<GPIOA_BASE, 3> button;
 *
 * Clk::enable<Clk::Apb2::iopa>();
 * button.setAsInput(InputMode::pullup);
 * bool state = button.read();
 *
 *
 * // ** Digital input with interrupt **
 *
 * PinA<12> buttonIt;
 * // or Pin<GPIOA_BASE, 12> buttonIt;
 *
 * Clk::enable<Clk::Apb2::iopa, Clk::Apb2::afio>();
 *
 * buttonIt.setAsInputIrq(InputMode::pullup, false, true); // IRQ on falling edge
 * buttonIt.setIrqPriority(0);
 * buttonIt.enableIrq();                                  //
 *
 * #ifdef __cplusplus
 * extern "C" {
 * #endif
 * void EXTI15_10_IRQHandler(void)
 * {
 *     buttonIt.disableIrq();
 *     if (buttonIt.irqIsPending()) {
 *         // Do something
 *         buttonIt.clearPendingIrqBit();
 *         buttonIt.clearPendingIrqLine();
 *     }
 *     buttonIt.enableIrq();
 * }
 *
 *
 * }
 * #ifdef __cplusplus
 * }
 * #endif
 *
 * TODO
 * bool state = button.read();
 *
 * button.disableIrq();
 *
 *
 * // ** Analog input w/ADC **
 *
 * Adc<ADC1_BASE> adc;
 * PinA<7> y_pos;
 *
 * Clk::enable<Clk::Apb2::iopa, Clk::Apb2::adc1>();
 * adc.setup(Adc::Channel::ch7, Adc::SampleTime::cycles_1pt5);
 * y_pos.setAsInput(InputMode::analog);
 * uint32_t value = Adc<ADC1_BASE>::poll();
 *
 * =============
 * Port Examples
 * =============
 *
 * // ** Set output mode for multiple pins **
 *
 * Stm32GpioB::setOutputs<OutputMode::pushpull, OutputSpeed::low, 3, 4, 5, 7>();
 *
 *
 */
#ifndef LIB_LIBPEKIN_STM32_PINS_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_PINS_STM32F1XX_H_

#include "libpekin_stm32_hal.h"
#include "libpekin.h"
#include "bits.h"

static_assert(GPIOA_BASE > 0, "STM32 CMSIS header must be included before this file");

namespace LibpStm32 {

enum class InputMode : uint8_t {
    // values are CNF & MODE bits for GPIO CR reg.
    analog   = 0b0000,     ///< analog
    floating = 0b0100,     ///< floating
    pulldown = 0b1000,     ///< pulldown
    // pullup/down config reg values are the same, so
    // add a suffix to differentiate and remove later.
    pullup   = 0b1000 | 0b1///< pullup
};

enum class ExtItMode : uint8_t {
    rising,
    falling,
    rising_falling
};

enum class OutputMode : uint8_t {
    pushpull      = 0b00,
    opendrain     = 0b01,
    alt_pushpull  = 0b10,
    alt_opendrain = 0b11,
};

enum class OutputSpeed : uint8_t {
    low = 0b10,
    med = 0b01,
    high = 0b11
};

/// Convert STM32 HAL GPIO_PIN_X constant into a bit number
/*constexpr uint32_t log2(uint32_t n)
{
    return ( (n < 2) ? 0 : 1 + log2(n/2));
}*/

namespace GpioModeMask {

/// TODO
constexpr uint32_t getOutputModeMask(OutputMode mode, OutputSpeed speed, uint8_t i)
{
    uint32_t mask = (Libp::enumBaseT(mode) << 2u | Libp::enumBaseT(speed)) << (i*4u);
    return i == 0
            ? mask
            : mask | getOutputModeMask(mode, speed, --i);
}

/**
 * Generate port configuration values for all pins in a 32-bit GPIOx_CR reg.
 *
 * @param mode
 *
 * @return
 */
constexpr uint32_t getOutputModeMask(OutputMode mode, OutputSpeed speed)
{
    return getOutputModeMask(mode, speed, 7);
}

/// TODO
constexpr uint32_t getInputModeMask(uint32_t mode, uint8_t i)
{
    uint32_t mask = mode << (i*4u);
    return i == 0
            ? mask
            : mask | getInputModeMask(mode, --i);
}

/**
 * Generate port configuration values for all pins in a 32-bit GPIOx_CR reg.
 *
 * @param mode
 *
 * @return
 */
constexpr uint32_t getInputModeMask(InputMode mode)
{
    // remove trailing 0b1 on pullup enum value
    return getInputModeMask(Libp::enumBaseT(mode) & 0b1100, 7);
}

static_assert(getOutputModeMask(OutputMode::pushpull, OutputSpeed::low) == 0b0010'0010'0010'0010'0010'0010'0010'0010);
static_assert(getOutputModeMask(OutputMode::alt_opendrain, OutputSpeed::high) == 0b1111'1111'1111'1111'1111'1111'1111'1111);
static_assert(getInputModeMask(InputMode::floating) == 0b0100'0100'0100'0100'0100'0100'0100'0100);
static_assert(getInputModeMask(InputMode::pullup) ==   0b1000'1000'1000'1000'1000'1000'1000'1000);

} // namespace GpioModeMask

template <uint32_t base_addr_>
class GpioPort {
public:
    // constexpr doesn't support reinterpret_cast, but GCC inlines this as a constant
    static inline GPIO_TypeDef* const port_ = reinterpret_cast<GPIO_TypeDef*>(base_addr_);

    template <OutputMode mode, OutputSpeed speed, uint8_t... pins>
    static void setOutputs()
    {
        const uint64_t mask_cr = (((uint64_t)0b1111 << (pins * 4u)) | ...);
        const uint32_t mask_crl = static_cast<uint32_t>(mask_cr);
        const uint32_t mask_crh = static_cast<uint32_t>(mask_cr >> 32u);

        port_->CRL = (port_->CRL & ~mask_crl) | (mask_crl & GpioModeMask::getOutputModeMask(mode, speed));
        port_->CRH = (port_->CRH & ~mask_crh) | (mask_crh & GpioModeMask::getOutputModeMask(mode, speed));
    }

    template <InputMode mode, uint8_t... pins>
    static void setInputs()
    {
        const uint64_t mask_cr = (((uint64_t)0b1111 << (pins * 4u)) | ...);
        const uint32_t mask_crl = static_cast<uint32_t>(mask_cr);
        const uint32_t mask_crh = static_cast<uint32_t>(mask_cr >> 32u);
        port_->CRL = (port_->CRL & ~mask_crl) | (mask_crl & GpioModeMask::getInputModeMask(mode));
        port_->CRH = (port_->CRH & ~mask_crh) | (mask_crh & GpioModeMask::getInputModeMask(mode));

        const uint32_t mask_odr = ((1 << pins) | ...);
        // TODO: use set/clear regs
        if (mode == InputMode::pullup)
            port_->ODR |= mask_odr;
        else
            port_->ODR &= ~mask_odr;
    }
    /// Sets all pins to analog input mode. According to AN4899, analog input
    /// mode consumes the least power.
    inline __attribute__((always_inline))
    static void setAllAnalogInput()
    {
        port_->CRL = 0;
        port_->CRH = 0;
    }
    /// Set the pins of the provided mask
    /// e.g. set(6) will set pins 1,2
    inline __attribute__((always_inline))
    static void set(uint16_t mask)
    {
        port_->BSRR = mask;
    }
    /// Clear the pins of the provided mask
    /// e.g. clear(6) will clear pins 1,2
    inline __attribute__((always_inline))
    static void clear(uint16_t mask)
    {
        port_->BRR = mask;
    }
    /// Toggle the pins of the provided mask
    /// e.g. toggle(6) will toggle pins 1,2
    inline __attribute__((always_inline))
    static void toggle(uint16_t mask)
    {
        port_->ODR ^= mask;
    }
    /**
     * Set specific pins to specific value
     * e.g. set(0b110,0b010) will clear pin 2 and set pin 1
     *
     * @param mask bit mask
     * @param value new values
     */
    inline __attribute__((always_inline))
    static void write(uint16_t mask, uint16_t value)
    {
        port_->ODR = (port_->ODR & ~mask) | value;
    }
};

using GpioA = GpioPort<GPIOA_BASE>;
using GpioB = GpioPort<GPIOB_BASE>;
#ifdef GPIOC_BASE
using GpioC = GpioPort<GPIOC_BASE>;
#endif
#ifdef GPIOD_BASE
using GpioD = GpioPort<GPIOD_BASE>;
#endif
#ifdef GPIOE_BASE
using GpioE = GpioPort<GPIOE_BASE>;
#endif
#ifdef GPIOF_BASE
using GpioF = GpioPort<GPIOF_BASE>;
#endif
#ifdef GPIOG_BASE
using GpioG = GpioPort<GPIOG_BASE>;
#endif

/**
 * @param pin_ 0..15
 */
template <uint32_t base_addr_, uint8_t pin_>
class Pin{
    static_assert(pin_ <= 15, "Pin must be 0..15");
    // constexpr doesn't support reinterpret_cast, but GCC inlines this as a constant
    static inline GPIO_TypeDef* const port_ = reinterpret_cast<GPIO_TypeDef*>(base_addr_);
    //same
    //inline __attribute__((always_inline))
    //static GPIO_TypeDef* port_ { return reinterpret_cast<GPIO_TypeDef*>(port_addr_); }

    static constexpr uint8_t lsb_pos = (pin_ >=8) ? (pin_ - 8) * 4 : pin_ * 4;
    static constexpr uint32_t mask = ~(0b1111 << lsb_pos);

public:
    /// Pin number (i.e. 0..15)
    static constexpr uint8_t pin = pin_;
    /**
     * Set the GPIO output level (i.e. ODR value)
     *
     * @param high
     */
    inline __attribute__((always_inline))
    static void set(bool high)
    {
        /*
         * port_->BSRR = high<<pin_
         * port_->BRR  = (!high)<<pin_;
         */
        if (high)
            set();
        else
            clear();
    }

    inline __attribute__((always_inline))
    static void set()    { port_->BSRR = 1<<pin_; }

    inline __attribute__((always_inline))
    static void clear()  { port_->BRR  = 1<<pin_; }

    inline __attribute__((always_inline))
    static void toggle() { port_->ODR ^= 1<<pin_; }

    /**
     * Read the pin input level.
     *
     * @return IDR register value for the pin
     */
    inline __attribute__((always_inline))
    static bool read()   { return port_->IDR & 1<<pin_; }

    /**
     * Setup the pin as an input without interrupts.
     *
     * @param mode
     */
    inline __attribute__((always_inline))
    static void setAsInput(InputMode mode)
    {
        // 4-bits per pin [CNF1:CNF0:MODE1:MODE0]
        if constexpr (pin_ >= 8) {
            port_->CRH = (port_->CRH & mask) | ((Libp::enumBaseT(mode) & 0b1100) << lsb_pos);
            // remove trailing 0b1 on pullup enum value ------------------^
        }
        else {
            port_->CRL = (port_->CRL & mask) | ((Libp::enumBaseT(mode) & 0b1100) << lsb_pos);
            // remove trailing 0b1 on pullup enum value ------------------^
        }
        set(mode == InputMode::pullup);
    }

    /**
     * Setup the pin as an input with interrupt triggering.
     *
     * Note:
     * - `enableIrq` must be called.
     * - afio clock must be enabled.
     *
     * @param mode cannot be analog
     * @param rising true to trigger on rising edge.
     * @param falling true to trigger on falling edge.
     */
    static void setAsInputIrq(InputMode mode, bool rising, bool falling)
    {
        setAsInput(mode); // TODO: cannot be analog

        // Select port in appropriate AFIO_EXTICRn reg
        constexpr uint8_t bit_pos = pin_ % 4 * 4;
        constexpr uint32_t exti_cr_mask = 0xFUL << bit_pos;
        constexpr uint32_t exti_cr = getPortExtiCrBits(base_addr_) << bit_pos;

        //
        Libp::Bits::setBits(AFIO->EXTICR[pin_ / 4], exti_cr_mask, exti_cr);
        // Rising trigger sel reg
        EXTI->RTSR = (EXTI->RTSR & ~(1<<pin_)) | (rising << pin_);
        // Falling trigger sel reg
        EXTI->FTSR = (EXTI->FTSR & ~(1<<pin_)) | (falling << pin_);
        // Interrupt mask reg (bits 0..19 == line 0..19, 0==masked)
        EXTI->IMR |= (1<<pin_);
        // TODO: do we need to clear pending bit?
    }

    /// Mask (disable) interrupt bit for this pin (i.e. EXTI_IMR)
    inline __attribute__((always_inline))
    static void clearAsInputIrq()
    {
        EXTI->IMR &= ~(1<<pin_);
    }

    inline __attribute__((always_inline))
    static void setAsOutput(OutputMode mode, OutputSpeed speed)
    {
        // 4-bits per pin [CNF1:CNF0:MODE1:MODE0]
        if constexpr (pin_ >= 8) {
            port_->CRH = (port_->CRH & mask) | ( Libp::enumBaseT(mode) << 2 | Libp::enumBaseT(speed) ) << lsb_pos;
        }
        else {
            port_->CRL = (port_->CRL & mask) | ( Libp::enumBaseT(mode) << 2 | Libp::enumBaseT(speed) ) << lsb_pos;
        }
    }

    /// Enable IRQ line for this pin (i.e. NVIC_ISER)
    /// afio clock must be enabled
    inline __attribute__((always_inline))
    static void enableIrq()
    {
        // NVIC->ISER[]
        NVIC_EnableIRQ(irqn());
        //SCB->AIRCR.PRIGROUP
        //NVIC->IP[y]
    }

    /// Disable IRQ line for this pin (i.e. NVIC_ICER)
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

    /**
     *
     * @return true if an interrupt is pending (i.e. EXTI_PR register bit set
     *         for this pin).
     */
    inline __attribute__((always_inline))
    static bool irqIsPending()
    {
        return EXTI->PR & (1<<pin_);
    }

    /// Clear IRQ pending bit for this pin (i.e. EXTI_PR)
    inline __attribute__((always_inline))
    static void clearPendingIrqBit()
    {
        //EXTI->PR |= (1<<pin_);
        EXTI->PR = (1<<pin_);
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
     * @return Interrupt Number for GPIO pin.
     */
    static constexpr IRQn_Type irqn() {
        switch (pin_) {
        case 0: return EXTI0_IRQn;
        case 1: return EXTI1_IRQn;
        case 2: return EXTI2_IRQn;
        case 3: return EXTI3_IRQn;
        case 4: return EXTI4_IRQn;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            return EXTI9_5_IRQn;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            return EXTI15_10_IRQn;
        }
        static_assert(pin_ <= 15);
        // can't reach here
        return EXTI0_IRQn;
    }
private:

    /**
     * @param addr port address
     * @return bit value to use in AFIO_EXTICRn reg for provided port
     */
    static constexpr uint8_t getPortExtiCrBits(uint32_t addr)
    {
        switch (addr) {
        case GPIOA_BASE: return 0b0000;
        case GPIOB_BASE: return 0b0001;
#ifdef GPIOC_BASE
        case GPIOC_BASE: return 0b0010;
#endif
#ifdef GPIOD_BASE
        case GPIOD_BASE: return 0b0011;
#endif
#ifdef GPIOE_BASE
        case GPIOE_BASE: return 0b0100;
#endif
#ifdef GPIOF_BASE
        case GPIOF_BASE: return 0b0101;
#endif
#ifdef GPIOG_BASE
        case GPIOG_BASE: return 0b0110;
#endif
        }
        return 0;
    }
};

/// pin = 0..15
template <uint8_t pin>
using PinA = Pin<GPIOA_BASE, pin>;

/// pin = 0..15
template <uint8_t pin>
using PinB = Pin<GPIOB_BASE, pin>;

#ifdef GPIOC_BASE
/// pin = 0..15
template <uint8_t pin>
using PinC = Pin<GPIOC_BASE, pin>;
#endif

#ifdef GPIOD_BASE
/// pin = 0..15
template <uint8_t pin>
using PinD = Pin<GPIOD_BASE, pin>;
#endif

#ifdef GPIOE_BASE
/// pin = 0..15
template <uint8_t pin>
using PinE = Pin<GPIOE_BASE, pin>;
#endif

#ifdef GPIOF_BASE
/// pin = 0..15
template <uint8_t pin>
using PinF = Pin<GPIOF_BASE, pin>;
#endif

#ifdef GPIOG_BASE
/// pin = 0..15
template <uint8_t pin>
using PinG = Pin<GPIOG_BASE, pin>;
#endif

/**
 *
 */
namespace DefPin {
// Valid for STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx, STM32F107xx
#ifdef USART1_BASE
inline constexpr PinA <9> usart1_tx;
inline constexpr PinA<10> usart1_rx;
inline constexpr PinA <8> usart1_ck;
inline constexpr PinA<11> usart1_cts;
inline constexpr PinA<12> usart1_rts;
#endif

#ifdef USART2_BASE
inline constexpr PinA<2> usart2_tx;
inline constexpr PinA<3> usart2_rx;
inline constexpr PinA<4> usart2_ck;
inline constexpr PinA<0> usart2_cts;
inline constexpr PinA<1> usart2_rts;
#endif

#ifdef USART3_BASE
inline constexpr PinB<10> usart3_tx;
inline constexpr PinB<11> usart3_rx;
inline constexpr PinB<12> usart3_ck;
inline constexpr PinB<13> usart3_cts;
inline constexpr PinB<14> usart3_rts;
#endif

#ifdef USART4_BASE
inline constexpr PinC<10> usart4_tx;
inline constexpr PinC<11> usart4_rx;
#endif

#ifdef USART5_BASE
inline constexpr PinC<12> usart5_tx;
inline constexpr PinD <2> usart5_rx;
#endif

#ifdef SPI1_BASE
inline constexpr PinA<4> spi1_nss;
inline constexpr PinA<5> spi1_sck;
inline constexpr PinA<6> spi1_miso;
inline constexpr PinA<7> spi1_mosi;
#endif

#ifdef SPI2_BASE
inline constexpr PinB<12> spi2_nss;
inline constexpr PinB<13> spi2_sck;
inline constexpr PinB<14> spi2_miso;
inline constexpr PinB<15> spi2_mosi;
#endif

#ifdef SPI3_BASE
inline constexpr PinA<15> spi3_nss;
inline constexpr PinB <3> spi3_sck;
inline constexpr PinB <4> spi3_miso;
inline constexpr PinB <5> spi3_mosi;
#endif

#ifdef I2C1_BASE
inline constexpr PinB<6> i2c1_scl;
inline constexpr PinB<7> i2c1_sda;
inline constexpr PinB<5> i2c1_smba;
#endif

#ifdef I2C2_BASE
inline constexpr PinB<10> i2c2_scl;
inline constexpr PinB<11> i2c2_sda;
inline constexpr PinB<12> i2c2_smba;
#endif

}

} // namespace LibpStm32

#endif /* LIB_LIBPEKIN_STM32_PINS_STM32F1XX_H_ */
