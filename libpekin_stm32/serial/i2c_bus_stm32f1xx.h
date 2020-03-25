#ifndef LIB_LIBPEKIN_STM32_SERIAL_I2C_BUS_H_
#define LIB_LIBPEKIN_STM32_SERIAL_I2C_BUS_H_

#include <cstdint>
#include <limits>

#include "libpekin.h"
#include "libpekin_stm32_hal.h"
#include "bus/bus_concepts.h"
#include "pins_stm32f1xx.h"
#include "clock_stm32f1xx.h"
#include "bits.h"

namespace LibpStm32::I2c {

/*    void clearBusyBitErrata()
    {
        // See ST errata notes:

        //1.Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register.
        //2.     Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        //3.     Check SCL and SDA High level in GPIOx_IDR.
        //4.     Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        //5.     Check SDA Low level in GPIOx_IDR.
        //6.     Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        //7.     Check SCL Low level in GPIOx_IDR.
        //8.     Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        //9.     Check SCL High level in GPIOx_IDR.
        //10.   Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 toGPIOx_ODR).
        //11.   Check SDA High level in GPIOx_IDR.
        //12.   Configure the SCL and SDA I/Os as Alternate function Open-Drain.
        //13.   Set SWRST bit in I2Cx_CR1 register.
        //14.   Clear SWRST bit in I2Cx_CR1 register.
        //15.   Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register

    }*/

/**
 * Attempts to clear "stuck" I2C slaves by toggling the clock line (@~50kHz)
 * until SDA is released or a time out occurs.
 *
 * The provided SCL/SDA pins will be temporarily put into opendrain mode and
 * returned to alt_opendrain mode prior to return.
 *
 * It is recommended to generate a stop condition on successful return.
 *
 * @param scl
 * @param sda
 * @param timeout_ms approximate timeout value. Cannot be zero.
 *
 * @return true if SDA is successfully released.
 */
template <uint32_t port_addr, uint8_t pin1, uint8_t pin2>
inline bool clearSlave(const Pin<port_addr, pin1>& scl, const Pin<port_addr, pin2>& sda, uint32_t timeout_ms)
{
    scl.setAsOutput(OutputMode::opendrain, OutputSpeed::high);
    sda.setAsOutput(OutputMode::opendrain, OutputSpeed::high);
    sda.set();
    timeout_ms *= 50; // ~20uS per loop
    while (!sda.read() && timeout_ms > 0) {
        scl.clear();
        Libp::delayUs(10);
        scl.set();
        Libp::delayUs(10);
        timeout_ms--;
    }
    bool success = sda.read();
    scl.setAsOutput(OutputMode::alt_opendrain, OutputSpeed::high);
    sda.setAsOutput(OutputMode::alt_opendrain, OutputSpeed::high);
    return success;
}

/// t_low / t_high ratio
enum class FmDuty : uint8_t {
    ratio_2_to_1 = 0,
    ratio_16_to_9 = 1
};

/**
 * Basic STM32F1xx I2C driver.
 *
 * Master mode only, 7-bit, single addressing, no interrupts or DMA.
 *
 * Tested on STM32F103xx.
 *
 * To achieve exact speeds, specific pclk1 frequencies are required:
 *
 *     FREQpclk1 / (i2c_freq * x) must be a whole number,
 *
 *     where:
 *
 *     x = 2 if i2c_freq <= 100k
 *         3 if i2c_freq > 100k & i2c_fmduty ==  ratio_2_to_1
 *        25 if i2c_freq > 100k & i2c_fmduty ==  ratio_16_to_9
 *
 * For example for exactly 400kHz I2C freq:
 *
 *     ratio_2_to_1,  FREQpclk1 should be a multiple of 1.2 MHz
 *     ratio_16_to_9, FREQpclk1 should be a multiple of 10 MHz
 *
 * @tparam base_addr_ peripheral memory address
 * @tparam i2c_freq I2C frequency in hertz
 * @tparam i2c_fmduty only used if `i2c_freq > 100000`
 */
template <uint32_t base_addr_, uint32_t i2c_freq = 100000, FmDuty i2c_fmduty = FmDuty::ratio_16_to_9>
class I2cBus {
private:

    // Can't use constexpr given reinterpret_cast, but compiler will inline
    static inline I2C_TypeDef* const port_ = reinterpret_cast<I2C_TypeDef*>(base_addr_);

    /// See code comments where this is used
    static constexpr bool fast_mode_no_btf_check = false;


public:

    I2cBus()
    {
        static_assert(I2cMaster<I2cBus<base_addr_, i2c_freq, i2c_fmduty>>);
    }

    /**
     * Enable the I2C peripheral.
     *
     * Does not start the I2C peripheral clock.
     *
     * The peripheral must be configured first via `configure`.
     */
    inline __attribute__((always_inline))
    static void enable()
    {
        port_->CR1 |= I2C_CR1_PE;
    }

    /**
     * Disable the I2C peripheral.
     *
     * Does not stop the I2C peripheral clock.
     */
    inline __attribute__((always_inline))
    static void disable()
    {
        // TODO: abort operation if we're using DMA
        port_->CR1 &= ~I2C_CR1_PE;
    }

    /**
     * Configure the I2C peripheral according to the template parameters
     * provided.
     *
     * Performs a software reset first.
     *
     * Assumes I2C GPIO pins (alt. func. open drain) and clock are already
     * configured.
     *
     * Peripheral must be enabled via `enable` after calling this function, and
     * may subsequently be disabled/enabled multiple times without needing to
     * be initialized again.
     */
    static void initialize()
    {
        // TODO: NVIC priority and enable
        // TODO: DMA config

        // Reset & disable

        port_->CR1 = I2C_CR1_SWRST;
        // TODO: some suggest a delay is needed here
        port_->CR1 = 0;

        // TODO: error if pclk1 too slow
        // Set timings

        const uint32_t pclk1_freq = LibpStm32::Clk::getPClk1();
        const uint8_t pclk1_freq_mhz = pclk1_freq / 1'000'000;
        constexpr bool fast_mode = i2c_freq > 100'000;

        port_->CR2 = pclk1_freq_mhz; // lower 5-bits are pclk1 freq in MHz (2->50)

        // Master mode max rise time
        port_->TRISE = fast_mode
                ? (pclk1_freq_mhz * 300 / 1000) + 1
                : pclk1_freq_mhz + 1;

        constexpr uint32_t ccr_multiplier = fast_mode
                ? (i2c_fmduty == FmDuty::ratio_2_to_1 ? 3 : 25)
                : 2;
        uint32_t ccr = pclk1_freq / (i2c_freq * ccr_multiplier);
        port_->CCR = fast_mode << I2C_CCR_FS_Pos | Libp::enumBaseT(i2c_fmduty) << I2C_CCR_DUTY_Pos | ccr;

        //hi2c->Instance->CR1 = (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode);

        // 7-bit, single addressing mode
        constexpr uint8_t stm32_i2c_addr1 = 0x33;
        constexpr uint8_t stm32_i2c_addr2 = 0x00;
        constexpr bool dual_addr_mode = false;

        // STM32 RM0008 reference manual (rev 20) section 26.6.3:
        // "Bit 14 Should always be kept at 1 by software."
        port_->OAR1 = stm32_i2c_addr1 << 1 | 1 << 14;
        port_->OAR2 = dual_addr_mode | stm32_i2c_addr2 << 1;

        // TODO: states
    }

private:

    inline __attribute__((always_inline))
    static void masterGenerateStart()
    {
        port_->CR1 |= I2C_CR1_START;
    }

    inline __attribute__((always_inline))
    static void clearAddrFlag()
    {
        [[maybe_unused]]
        volatile uint32_t clear_addr_flag;
        clear_addr_flag = port_->SR1;
        clear_addr_flag = port_->SR2;
    }

    inline __attribute__((always_inline))
    static void masterWriteSlaveAddr(uint8_t slave_addr, bool read)
    {
        port_->DR = (slave_addr << 1) | read;
    }

    inline __attribute__((always_inline))
    static bool masterWriteBlock(const uint8_t buffer[], size_t len, uint32_t start_time_ms, uint32_t timeout_duration_ms)
    {
        port_->DR = buffer[0];
        for (size_t i = 1; i < len ; ) {

            // RM0008 Rev 20 - pg.759
            // ----------------------
            // If TxE is set and a data byte was not written in the DR register
            // before the end of the last data transmission, BTF is set and the
            // interface waits until BTF is cleared by a read from I2C_SR1
            // followed by a write to I2C_DR, stretching SCL low.
            //
            // RM0008 Rev 20 - pg.760
            // ----------------------
            // The EV8 software sequence must complete before the end of the
            // current byte transfer. In case EV8 software sequence can not be
            // managed before the current byte end of transfer, it is
            // recommended to use BTF instead of TXE with the drawback of
            // slowing the communication

            // So..., this really depends on sys clock speed vs I2C speed.
            // There are very few scenarios (excl. IRQ) where the shift reg.
            // could empty before another byte is written into DR, but...the
            // cost (in terms of I2C slowdown) of checking BTF seems marginal,
            // for precisely the same reason.

            constexpr uint32_t flag_to_wait_for = fast_mode_no_btf_check
                    ? I2C_SR1_TXE
                    : I2C_SR1_BTF;
            if (!waitForSr1FlagSet(flag_to_wait_for, start_time_ms, timeout_duration_ms))
                return false;
            port_->DR = buffer[i++];
        }
        return waitForSr1FlagSet(I2C_SR1_BTF, start_time_ms, timeout_duration_ms); // <-- here
    }

    inline __attribute__((always_inline))
    static void masterGenerateStop()
    {
        port_->CR1 |= I2C_CR1_STOP;
        // TODO: we should be waiting for stop?
        // see: https://stackoverflow.com/questions/2556794/stm32-i2c1-start-bit-not-set-on-sr1-register
    }

    static bool waitForSr1FlagSet(uint32_t flag, uint32_t start_time_ms, uint32_t timeout_duration_ms)
    {
        bool timed_out = false;
        while (!(port_->SR1 & flag) && !timed_out)
            timed_out = (Libp::getMillis() - start_time_ms) > timeout_duration_ms;
        return !timed_out;
    }

    static bool waitForStopFlagClear(uint32_t start_time_ms, uint32_t timeout_duration_ms)
    {
        bool timed_out = false;
        while ((port_->CR1 & I2C_CR1_STOP) && !timed_out)
            timed_out = (Libp::getMillis() - start_time_ms) > timeout_duration_ms;
        return !timed_out;
    }

    /* TODO:
     */
    enum class I2CState : uint8_t {
        disabled,
        idle,
        xfer_done,
        busy,
        error
    };

public:

    /**
     * Performs a master transmit.
     *
     * NOOP if `len ==0`
     *
     * @param slave_addr 7-bit slave address (not shifted)
     * @param buffer data to transmit.
     * @param len number of bytes to transmit.
     * @param timeout_ms
     */
    bool masterTransmit(uint8_t slave_addr, const uint8_t* buffer, size_t len, uint32_t timeout_ms) const
    {
        if (len == 0)
            return true;

        // TODO: assumes Libp::getMillis returns uint32_t
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        uint32_t start_time = Libp::getMillis();

        //while (port_->SR2 & I2C_SR2_BUSY)
        //    ;
        masterGenerateStart();
        if (!waitForSr1FlagSet(I2C_SR1_SB, start_time, timeout_ms))
            return false;

        masterWriteSlaveAddr(slave_addr, false);
        if (!waitForSr1FlagSet(I2C_SR1_ADDR, start_time, timeout_ms))
            return false;

        clearAddrFlag();

        if (!masterWriteBlock(buffer, len, start_time, timeout_ms))
            return false;
        masterGenerateStop();
        return true;
    }

    /**
     * Performs a master transmit.
     *
     * NOOP if `len ==0` (use other overload for single byte transmission)
     *
     * @param slave_addr 7-bit slave address (not shifted)
     * @param reg_addr single byte to transmit prior to the buffer data
      *                (typically a register address)
     * @param buffer data to transmit.
     * @param len number of bytes to transmit (excluding reg_addr - i.e. `buffer` length).
     * @param timeout_ms
     */
    bool masterTransmit(uint8_t slave_addr, uint8_t reg_addr, const uint8_t* buffer, size_t len, uint32_t timeout_ms) const
    {
        if (len == 0)
            return true;

        // TODO: assumes Libp::getMillis returns uint32_t
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        uint32_t start_time = Libp::getMillis();

        //while (port_->SR2 & I2C_SR2_BUSY)
        //    ;
        masterGenerateStart();
        if (!waitForSr1FlagSet(I2C_SR1_SB, start_time, timeout_ms))
            return false;

        masterWriteSlaveAddr(slave_addr, false);
        if (!waitForSr1FlagSet(I2C_SR1_ADDR, start_time, timeout_ms))
            return false;
        clearAddrFlag();

        if ( !masterWriteBlock(&reg_addr, 1, start_time, timeout_ms)
          || !masterWriteBlock(buffer, len, start_time, timeout_ms) )
            return false;

        masterGenerateStop();
        return true;
    }

    /**
     * Performs a master receive.
     *
     * NOOP if `len ==0`
     *
     * @param slave_addr 7-bit slave address (not shifted)
     * @param buffer buffer to receive data.
     * @param length number of bytes to receive.
     * @param timeout_ms
     */
    bool masterReceive(uint8_t slave_addr, uint8_t* buffer, size_t len, uint32_t timeout_ms) const
    {
        if (len == 0)
            return true;

        // Process based on AN2824 (Doc ID 15021 Rev.4)
        //
        // Interrupts are enabled/disabled below due to the guidelines on page
        // 6 of the application note:
        // ----------------------------------------
        //
        // Due to the “Wrong data read into data register” limitation
        // described in the device errata sheet, interrupts must be masked
        // between STOP programming and DataN-1 reading. Please refer to
        // the device errata sheet for more details.
        //
        // The EV6_1 software sequence must complete before the ACK pulse
        // of the current byte transfer. To ensure this, interrupts must be
        // disabled between ADDR clearing and ACK clearing.

        // TODO: assumes Libp::getMillis returns uint32_t
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max(); // never timeout
        uint32_t start_time = Libp::getMillis();

        masterGenerateStart();
        if (!waitForSr1FlagSet(I2C_SR1_SB, start_time, timeout_ms))
            return false;

        masterWriteSlaveAddr(slave_addr, true);
        if (!waitForSr1FlagSet(I2C_SR1_ADDR, start_time, timeout_ms))
            return false;

        size_t i = 0;
        size_t bytes_left = len;

        // *** >=3 bytes to read procedure

        if (bytes_left > 2) {
            clearAddrFlag();
            while (bytes_left > 3) {
                constexpr uint32_t flag_to_wait_for = fast_mode_no_btf_check
                        ? I2C_SR1_RXNE
                        : I2C_SR1_BTF;
                if (!waitForSr1FlagSet(flag_to_wait_for, start_time, timeout_ms))
                    return false;
                buffer[i++] = port_->DR;
                bytes_left--;
            }

            if ( !waitForSr1FlagSet(I2C_SR1_RXNE, start_time, timeout_ms))
                return false;

            if (!waitForSr1FlagSet(I2C_SR1_BTF, start_time, timeout_ms)) // Don't read reg, wait for next byte
                return false;

            port_->CR1 &= ~I2C_CR1_ACK;

            buffer[i++] = port_->DR; // N-2
            bytes_left--;

            __disable_irq();
            masterGenerateStop();

            buffer[i++] = port_->DR; // N-1
            bytes_left--;

            __enable_irq();
            if (!waitForSr1FlagSet(I2C_SR1_RXNE, start_time, timeout_ms))
                return false;

            buffer[i++] = port_->DR; // N
            bytes_left--;

            // Wait until STOP is cleared by hardware
            if (!waitForStopFlagClear(start_time, timeout_ms))
                return false;

            port_->CR1 |= I2C_CR1_ACK;
        }

        // *** 2 bytes to read procedure

        else if (bytes_left == 2) {
            // TODO: can do this before ADDR flag set
            port_->CR1 |= I2C_CR1_POS | I2C_CR1_ACK;
            __disable_irq();
            clearAddrFlag();
            port_->CR1 &= ~I2C_CR1_ACK;
            __enable_irq();

            if (!waitForSr1FlagSet(I2C_SR1_BTF, start_time, timeout_ms))
                return false;

            __disable_irq();
            masterGenerateStop();

            buffer[i++] = port_->DR; // N-1
            bytes_left--;

            __enable_irq();

            buffer[i++] = port_->DR; // N
            bytes_left--;

            // Wait until STOP is cleared by hardware
            if (!waitForStopFlagClear(start_time, timeout_ms))
                return false;

            port_->CR1 &= ~I2C_CR1_POS;
            port_->CR1 |= I2C_CR1_ACK;
        }

        // *** 1 byte to read procedure

        else if (bytes_left == 1) {
            port_->CR1 &= ~I2C_CR1_ACK;
            __disable_irq();

            clearAddrFlag();

            masterGenerateStop();

            __enable_irq();
            if (!waitForSr1FlagSet(I2C_SR1_RXNE, start_time, timeout_ms))
                return false;

            buffer[i++] = port_->DR; // N
            bytes_left--;

            // Wait until STOP is cleared by hardware
            if (!waitForStopFlagClear(start_time, timeout_ms))
                return false;

            port_->CR1 |= I2C_CR1_ACK;
        }
        return true;
    }

    /**
     * Performs a master transmit followed by a master receive.
     *
     * This is typically used to read memory/registers from a slave device.
     *
     * NOOP if `transmit_len ==0` or `rec_len == 0`.
     *
     * @param slave_addr 7-bit slave address (not shifted)
     * @param transmit_buf data to transmit.
     * @param transmit_len number of bytes to transmit.
     * @param receive_buf buffer to receive data.
     * @param rec_len number of bytes to receive.
     * @param timeout_ms
     *
     * @return false on error
     */
    bool masterTransmitReceive(
            uint8_t slave_addr, const uint8_t* transmit_buf, size_t transmit_len, uint8_t* receive_buf,
            size_t rec_len, uint32_t timeout_ms) const
    {
        if (transmit_len == 0 || rec_len == 0)
            return true;

        // TODO: assumes Libp::getMillis returns uint32_t
        // TODO: deal with 32/64 bit Libp::getMillis
        // if Libp::getMillis returns
        if (!timeout_ms)
            timeout_ms = std::numeric_limits<uint32_t>::max();

        uint32_t start_time = Libp::getMillis();
        //while (port_->SR2 & I2C_SR2_BUSY)
        //    ;

        masterGenerateStart();
        if (!waitForSr1FlagSet(I2C_SR1_SB, start_time, timeout_ms))
            return false;

        masterWriteSlaveAddr(slave_addr, false);
        if (!waitForSr1FlagSet(I2C_SR1_ADDR, start_time, timeout_ms))
            return false;

        clearAddrFlag();
        if (!masterWriteBlock(transmit_buf, transmit_len, start_time, timeout_ms))
            return false;

        // Should be overflow safe
        uint32_t now = Libp::getMillis();
        if ( (now - start_time) >= timeout_ms )
            return false;
        timeout_ms -= (now - start_time);
        return masterReceive(slave_addr, receive_buf, rec_len, timeout_ms);
    }

};

} // namespace LibpStm32::I2c

#endif /* LIB_LIBPEKIN_STM32_SERIAL_I2C_BUS_H_ */
