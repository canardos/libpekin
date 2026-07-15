/**
 * Basic STM32F1xx USART functionality.
 *
 * May be usable on other targets
 */
#ifndef LIB_LIBPEKIN_STM32_SERIAL_UART_STM32_H_
#define LIB_LIBPEKIN_STM32_SERIAL_UART_STM32_H_

#include "lp_bits.h"
#include "lp_types.h"
#include "lp_ring_buffer.h"
#include "serial/lp_i_serial_io.h"
#include "lp_clock_stm32f1xx.h"
#include "libpekin.h"
#include <cstdint>
#include <limits>


namespace libp_stm32::uart {

enum class Parity : uint32_t {
    none = 0b00 << USART_CR1_PS_Pos,
    even = 0b10 << USART_CR1_PS_Pos,
    odd  = 0b11 << USART_CR1_PS_Pos,
};

/// The 0.5 Stop bit and 1.5 Stop bit are not available for UART4 & UART5
enum class StopBits : uint32_t {
    bits_0pt5 = 0b01 << USART_CR2_STOP_Pos,
    bits_1    = 0b00 << USART_CR2_STOP_Pos,
    bits_1pt5 = 0b11 << USART_CR2_STOP_Pos,
    bits_2    = 0b10 << USART_CR2_STOP_Pos

};

enum class Mode : uint32_t {
    rx_only = USART_CR1_RE,
    tx_only = USART_CR1_TE,
    rx_tx   = USART_CR1_RE | USART_CR1_TE
};

/// Usart interrupt trigger types
enum class IrqType : uint32_t {
    tx_empty = USART_CR1_TXEIE,
    tx_complete   = USART_CR1_TCIE,
    rx_data_ready = USART_CR1_RXNEIE, ///< will also enable overrun event
    idle = USART_CR1_IDLEIE,
    parity_error = USART_CR1_PEIE,
    // Not currently implemented
    //cts = USART_CR3_CTSIE,
    //break_flag = USART_CR2_LBDIE,
    //noise_flag = USART_CR3_EIE ///< only used in DMA mode
};

inline constexpr uint32_t cr1_irq_events_mask =
        USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE;

/**
 * ISerialIo implementation for STM32f10xx USART.
 *
 * Read functions rely on Libpekin `libp::getMillis` function. Libpekin timers must
 * be initialized before calling if a timeout is used.
 *
 * The class may be used in direct polling mode or in interrupt mode. Write
 * functions are identical in both modes, but read functions differ. The mode
 * is determined by the `mode_` template parameter:
 *
 * ------------
 * Polling Mode
 * ------------
 *
 * `read` functions read the hardware directly with no buffering.
 *
 * \code
 * using namespace libp_stm32;
 *
 * uart::UartIo<USART1_BASE> my_uart;
 * \endcode
 *
 * --------------
 * Interrupt Mode
 * --------------
 *
 * All `read` functions will read from an internal buffer. The buffer is
 * written to by calling `serviceIrq` from the UART ISR on each RXNE interrupt.
 *
 * \code
 * using namespace libp_stm32;
 *
 * uart::UartIo<USART1_BASE, uart::IrqMode::interrupt, 256> my_uart;
 * my_uart.configIrq<Stm32::uart::IrqType::rx_data_ready>();
 * my_uart.enableIrq();
 *
 * void USART1_IRQHandler(void)
 * {
 *     uart_rn52.clearPendingIrq();
 *     uart_rn52.serviceIrq();
 * }
 * \endcode
 *
 * @tparam base_addr_
 * @tparam mode_
 * @tparam buffer_size_ must be a power of 2
 */

/// UartIo interrupt mode
enum class IrqMode : uint8_t {
    polling,
    interrupt
};

template <uint32_t base_addr_, IrqMode mode_ = IrqMode::polling, uint32_t buffer_size_ = 0>
class UartIo : public libp::ISerialIo {
    static_assert(
               (mode_ == IrqMode::polling && buffer_size_ == 0)
            || (mode_ == IrqMode::interrupt && buffer_size_ > 0),
            "buffer_size_ must be > 0 in interrupt mode.");

    static_assert(
            base_addr_ == USART1_BASE
#ifdef USART2_BASE
            || base_addr_ == USART2_BASE
#endif
#ifdef USART3_BASE
            || base_addr_ == USART3_BASE
#endif
#ifdef USART4_BASE
            || base_addr_ == USART4_BASE
#endif
#ifdef USART5_BASE
            || base_addr_ == USART5_BASE
#endif
            , "base_addr_ should be one of USART[1..5]_BASE");

    // can't use constexpr with reinterpret_cast
    static inline USART_TypeDef* const port_ = reinterpret_cast<USART_TypeDef*>(base_addr_);

    // Conditionally include buffer member at compile
    // time if interrupt mode is specified
    struct empty { };
    using ring_buffer_t = std::conditional_t<mode_ == IrqMode::polling, empty, libp::RingBuffer<uint8_t, buffer_size_>>;
    [[no_unique_address]] ring_buffer_t rec_buffer_;

public:

    /**
     * @return Interrupt Number for USART.
     */
    static constexpr IRQn_Type irqn()
    {
        switch (base_addr_) {
        case USART1_BASE: return USART1_IRQn;
#ifdef USART2_BASE
        case USART2_BASE: return USART2_IRQn;
#endif
#ifdef USART3_BASE
        case USART3_BASE: return USART3_IRQn;
#endif
#ifdef USART4_BASE
        case USART4_BASE: return USART4_IRQn;
#endif
#ifdef USART5_BASE
        case USART5_BASE: return USART5_IRQn;
#endif
        }
        return USART1_IRQn; // TODO: error
    }

    /// Tx/Rx, no parity, 1-stop bit, no interrupts,
    /// defaults for everything else (see STM32 docs).
    /// CR1 and CR2 will be cleared. Make custom changes (e.g. `enableIrq`)
    /// after calling this function.
    static void start(uint32_t baud)
    {
        start(Mode::rx_tx, Parity::none, StopBits::bits_1, baud);
    }

    /// No interrupts, defaults for everything else (see STM32 docs).
    /// CR1 and CR2 will be cleared. Make custom changes (e.g. `enableIrq`)
    /// after calling this function.
    static void start(Mode mode, Parity parity, StopBits stop_bits, uint32_t baud)
    {
        // TODO: move baud to first param
        const uint32_t clk = base_addr_ == USART1_BASE
                ? clk::getPClk2()
                : clk::getPClk1();
        port_->BRR = clk / baud; // TODO: do we need to round?
        port_->CR1 = libp::enumVal(mode) | libp::enumVal(parity)/* | libp::enumVal(IrqType::rx_data_ready)*/;
        port_->CR2 = libp::enumVal(stop_bits);
        port_->CR1 |= USART_CR1_UE;
    }

    /// Stop USART prescalars and outputs at the end of the current byte transfer.
    static void stop()
    {
        port_->CR1 &= ~USART_CR1_UE;
    }

    /**
     * Enable/disable specific interrupt types.
     *
     * All interrupts not specified will be disabled.
     *
     * Interrupts still need to be enabled in the NVIC via `enableIrq`
     *
     * @tparam it_types
     */
    template <IrqType ...it_types>
    static void configIrq()
    {
        // TODO do we need to stop/start?
        constexpr uint32_t enable_mask = (libp::enumVal(it_types) | ... | 0);
        libp::bits::setBits(port_->CR1, cr1_irq_events_mask, enable_mask);
    }


    /**
     * Read a single byte (if available) and write it to the receive buffer.
     *
     * Must be called from ISR on each RXNE interrupt.
     *
     * Does not check if buffer is full. Failure to call `read` functions will
     * result in buffer overwriting.
     */
    inline __attribute__((always_inline))
    void serviceIrq()
    {
        if constexpr (mode_ == IrqMode::interrupt) {
            if(port_->SR & USART_SR_RXNE) {
                rec_buffer_.write(port_->DR);
            }
        }
    }

    /**
     * Clear pending interrupt flag for this uart in NVIC
     */
    inline __attribute__((always_inline))
    static void clearPendingIrq()
    {
        NVIC_ClearPendingIRQ(irqn());
    }

    /**
     * Enable interrupts for this uart in NVIC
     */
    inline __attribute__((always_inline))
    static void enableIrq()
    {
        NVIC_EnableIRQ(irqn());
    }

    /**
     * Disable interrupts for this uart in NVIC
     */
    inline __attribute__((always_inline))
    static void disableIrq()
    {
        NVIC_DisableIRQ(irqn());
    }

    /**
     *
     * @return nullptr if mode_ == IrqMode::polling
     */
    inline libp::RingBuffer<uint8_t, buffer_size_>* buffer()
    {
        if constexpr (mode_ == IrqMode::interrupt) {
            return &rec_buffer_;
        }
        else {
            return nullptr;
        }
    }

    /**
     * @return true if uart buffer overrun flag is set
     */
    inline __attribute__((always_inline))
    static bool isOverrun()
    {
        return port_->SR & USART_SR_ORE;
    }

    void write(char ch) const override
    {
        while (!(port_->SR & USART_SR_TXE))
            ;
        port_->DR = ch;
        // TODO: Check USART_SR_TC to ensure all sent before returning?
    }

    void write(const char* msg) const override
    {
        uint16_t i = 0;
        uint8_t c;
        while ((c = msg[i++]) != '\0') {
            while (!(port_->SR & USART_SR_TXE))
                ;
            port_->DR = c;
        }
        // TODO: Check USART_SR_TC to ensure all sent before returning?
    }

    void write(const char* msg, uint16_t len) const override
    {
    	uint16_t i = 0;
        while (len-- > 0) {
            while (!(port_->SR & USART_SR_TXE))
                ;
            port_->DR = msg[i++];
        }
        // TODO: Check USART_SR_TC to ensure all sent before returning?
    }


    /**
     * Relies on Libpekin `libp::getMillis` function. Libpekin timers must be
     * initialized if `timeout_ms != 0`
     */
    uint16_t read(char* buf, uint16_t len, uint32_t timeout_ms) override
    {
        uint16_t i = 0;
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        while (len-- > 0) {
            uint32_t start_time = libp::getMillis();
            bool timed_out = false;

            // ** interrupt version - read from buffer
            if constexpr (mode_ == IrqMode::interrupt) {
                while (rec_buffer_.isEmpty() && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i++] = rec_buffer_.read();
            }

            // ** polling version - read directly from hardware
            else {
                while ( !(port_->SR & USART_SR_RXNE) && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i++] = port_->DR; // clears RXNE bit
            }
        }
        return i;
    }

    /**
     * Relies on Libpekin `libp::getMillis` function. Libpekin timers must be
     * initialized if `timeout_ms != 0`
     */
    uint16_t read(char* buf, char terminator, uint16_t max_len, uint32_t timeout_ms) override
    {
        uint16_t i = 0;
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        while (max_len-- > 0) {
            uint32_t start_time = libp::getMillis();
            bool timed_out = false;

            // ** interrupt version - read from buffer
            if constexpr (mode_ == IrqMode::interrupt) {
                while (rec_buffer_.isEmpty() && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i] = rec_buffer_.read();
            }

            // ** polling version - read directly from hardware
            else {
                while ( !(port_->SR & USART_SR_RXNE) && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i] = port_->DR; // clears RXNE bit
            }

            if (buf[i++] == terminator)
                break;
        }
        return i;
    }

    // TODO: test
    /**
     * Relies on Libpekin `libp::getMillis` function. Libpekin timers must be
     * initialized if `timeout_ms != 0`
     */
    uint16_t readln(char* buf, Eol eol_type, uint16_t max_len, uint32_t timeout_ms) override
    {
        if (max_len == 0)
            return 0;

        const char term_ch = (eol_type == Eol::lf) ? '\n' : '\r';
        bool waiting_on_lf = false;

        uint16_t i = 0;
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        while (--max_len > 0) {
            uint32_t start_time = libp::getMillis();
            bool timed_out = false;

            // ** interrupt version - read from buffer
            if constexpr (mode_ == IrqMode::interrupt) {
                while (rec_buffer_.isEmpty() && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i] = rec_buffer_.read();
            }

            // ** polling version - read directly from hardware
            else {
                while ( !(port_->SR & USART_SR_RXNE) && !timed_out)
                    timed_out = (libp::getMillis() - start_time) > timeout_ms;
                if (timed_out)
                    break;
                buf[i] = port_->DR; // clears RXNE bit
            }

            if (eol_type == Eol::crlf) {
                if (waiting_on_lf && buf[i] == '\n') {
                    i--; // put null terminator in the right place
                    break;
                }
                else {
                    waiting_on_lf = (buf[i] == '\r');
                }
            }
            else {
                if (buf[i] == term_ch)
                    break;
            }
            i++;
        }
        buf[i] = '\0';
        return i;
    }
};

} // namespace libp_stm32::uart

#endif /* LIB_LIBPEKIN_STM32_SERIAL_UART_STM32_H_ */
