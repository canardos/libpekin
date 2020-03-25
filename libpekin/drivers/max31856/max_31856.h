/**
 * Driver for the Maxim Integrated MAX31856 Precision
 * Thermocouple to Digital Converter with Linearization.
 *
 * Uses Libpekin library functionality.
 *
 * Largely untested
 *
 * Datasheet: https://www.maximintegrated.com/en/products/sensors/MAX31856.html
 */
#ifndef SRC_DEVICES_MAX_31856_H_
#define SRC_DEVICES_MAX_31856_H_

//#include <error_handler.h>
#include <cstdint>
#include "serial/spi.h"

// Outer namespace to encapsulate while avoiding use of template
// params when referencing below enums/calling decodeTemp etc.
namespace Libp::Max31856 {

enum class Mode : uint8_t {
    one_shot   = 0, // default
    continuous = 1
};

/// Thermocouple voltage conversion averaging Mode
enum class ConversionMode : uint8_t {
    single_sample  = 0b000, //default
    avg_2_samples  = 0b001,
    avg_4_samples  = 0b010,
    avg_8_samples  = 0b011,
    avg_16_samples = 0b100  // 1xx
};

/// Thermocouple type
enum class TcType : uint8_t {
    b_type = 0b0000,
    e_type = 0b0001,
    j_type = 0b0010,
    k_type = 0b0011, // default
    n_type = 0b0100,
    r_type = 0b0101,
    s_type = 0b0110,
    t_type = 0b0111
    // 10xx = Voltage Mode, Gain = 8. Code = 8 x 1.6 x 217 x VIN
    // 11xx = Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN
    // Where Code is 19 bit signed number from TC registers and VIN is thermocouple input voltage
};
enum class NoiseFilter : uint8_t {
    freq_60hz = 0, // default
    freq_50hz = 1
};
enum class OcFaultMode : uint8_t {
    disabled = 0b00, // default
    test_10ms = 0b01,
    test_32ms = 0b10,
    test_100ms = 0b11,
};


/**
 * Driver for the Maxim Integrated MAX31856 Precision
 * Thermocouple to Digital Converter with Linearization.
 *
 * This class handles configuration and temperature reading via SPI.
 * Reading of the FAULT or DDRY GPIO signals should be done externally.
 *
 * Communication with the IC is via an @p SpiRegisterOps object provided at
 * construction.
 *
 * Datasheet: https://www.maximintegrated.com/en/products/sensors/MAX31856.html
 */
template <typename T>
class Max31856 {
public:
    /**
     * Register read addresses. Write address is read address + 0x80.
     */
    struct RegAddr {

        /**
         * The Configuration 0 register selects the conversion mode (automatic or
         * triggered by the 1-shot command), selects opencircuit fault detection
         * timing, enables the cold-junction sensor, clears the fault status
         * register, and selects the filter notch frequencies.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t config_0 = 0x00;

        /**
         * The Configuration 1 register selects the averaging time for the
         * thermocouple voltage conversion averaging mode and also selects the
         * thermocouple type being monitored.
         *
         * Default value: 0x03
         */
        inline static constexpr uint8_t config_1 = 0x01;

        /**
         * The Fault Mask Register allows the user to mask faults from causing the
         * FAULT output from asserting. Masked faults will still result in fault
         * bits being set in the Fault Status register (0Fh). Note that the FAULT
         * output is never asserted by thermocouple and cold-junction out-of-range
         * status.
         *
         * Default value: 0xFF
         */
        inline static constexpr uint8_t fault_mask = 0x02;

        /**
         * Write a temperature limit value to this register. When the measured cold-
         * junction temperature is greater than this value, the CJ High fault status
         * bit will be set and (if not masked) the FAULT output will assert.
         *
         * Default value: 0x7F
         */
        inline static constexpr uint8_t cj_temp_high_threshold = 0x03;

        /**
         * Write a temperature limit value to this register. When the measured cold-
         * junction temperature is less than this value, the CJ Low fault status bit
         * will be set and (if not masked) the FAULT output will assert.
         *
         * Default value: 0xC0
         */
        inline static constexpr uint8_t cj_temp_low_threshold = 0x04;

        /**
         * Write the MSB of the two-byte temperature limit value to this register.
         * When the linearized thermocouple temperature is greater than the two-
         * byte (05h and 06h) limit value, the TC High fault status bit will be set
         * and (if not masked) the FAULT output will assert
         *
         * Default value: 0x7F
         */
        inline static constexpr uint8_t tc_temp_high_thresh_msb = 0x05;

        /**
         * Write the LSB of the two-byte temperature limit value to this register.
         * When the linearized thermocouple temperature is greater than the two-
         * byte (05h and 06h) limit value, the TC High fault status bit will be set
         * and (if not masked) the FAULT output will assert.
         *
         * Default value: 0xFF
         */
        inline static constexpr uint8_t tc_temp_high_thresh_lsb = 0x06;

        /**
         * Write the MSB of the two-byte temperature limit value to this register.
         * When the linearized thermocouple temperature is less than the two-byte
         * (07h and 08h) limit value, the TC Low fault status bit will be set and
         * (if not masked) the FAULT output will assert.
         *
         * Default value: 0x80
         */
        inline static constexpr uint8_t tc_temp_low_thresh_msb = 0x07;

        /**
         * Write the LSB of the two-byte temperature limit value to this register.
         * When the linearized thermocouple temperature is less than the two-byte
         * (07h and 08h) limit value, the TC Low fault status bit will be set and
         * (if not masked) the FAULT output will assert.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t tc_temp_low_thresh_lsb = 0x08;

        /**
         * When the cold-junction temperature sensor is enabled, this register
         * allows an offset temperature to be applied to the measured value
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t cj_offset = 0x09;

        /**
         * This register contains the MSB of the two-byte (0Ah and 0Bh) value used
         * for cold-junction compensation of the thermocouple measurement. When the
         * cold-junction temperature sensor is enabled, this register is read-only
         * and contains the MSB of the measured cold-junction temperature plus the
         * value in the Cold-Junction Offset register. Also when the cold-junction
         * temperature sensor is enabled, a read of this register will reset the
         * DRDY pin high. When the cold-junction temperature sensor is disabled,
         * this register becomes a read-write register that contains the MSB of the
         * most recent cold-junction conversion result until a new value is written
         * into it. This allows writing the results from an external temperature
         * sensor, if desired. The maximum contained in the two cold-junction
         * temperature bytes is clamped at 128°C and the minimum is clamped at
         * -64°C.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t cj_temp_hi_byte = 0x0A;

        /**
         * This register contains the LSB of the two-byte (0Ah and 0Bh) value used
         * for cold-junction compensation of the thermocouple measurement. When the
         * cold-junction temperature sensor is enabled, this register is read-only
         * and contains the LSB of the measured cold-junction temperature plus the
         * value in the Cold-Junction Offset register. Also when the cold-junction
         * temperature sensor is enabled, a read of this register will reset the
         * DRDY pin high. When the cold-junction temperature sensor is disabled,
         * this register becomes a read-write register that contains the LSB of the
         * most recent cold-junction conversion result until a new value is written
         * into it.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t cj_temp_lo_byte = 0x0B;
        // output only regs

        /**
         * This is the high byte of the 19-bit register that contains the
         * linearized and cold-junction-compensated thermocouple temperature value.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t tc_temp_hi_byte = 0x0C;

        /**
         * This is the middle byte of the 19-bit register that contains the
         * linearized and cold-junction-compensated thermocouple temperature value.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t tc_temp_mid_byte = 0x0D;

        /**
         * This is the low byte of the 19-bit register that contains the
         * linearized and cold-junction-compensated thermocouple temperature value.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t tc_temp_lo_byte = 0x0E;

        /**
         * The Fault Status Register contains eight bits that indicate the fault
         * conditions (Thermocouple Out-of-Range, Cold Junction Out-of-Range, Cold
         * Junction High, Cold Junction Low, Thermocouple High Temperature,
         * Thermocouple Low Temperature, Over-Under Voltage, or Open Thermocouple)
         * that have been detected.
         *
         * Default value: 0x00
         */
        inline static constexpr uint8_t fault_status = 0x0F;

        static constexpr uint8_t getWriteAddr(uint8_t read_addr)
        {
            return read_addr | 0x80;
        }
    };

    /// Bit positions for various registers
    struct RegBitPos {

        struct Config0 {
            /// Conversion mode (0 = normally off[default], 1 = auto)
            inline static constexpr uint8_t cmode = 7;
            /// One-shot mode (0 = no conversion req.[default], 1 = req. conversion)
            inline static constexpr uint8_t one_shot = 6;
            /// Open circuit fault detection config
            inline static constexpr uint8_t oc_fault = 5;
            /// 0 = enabled[default], 1 = disabled
            inline static constexpr uint8_t cj_sensor_disable = 3;
            /// 0 = comparator mode[default], 1 = interrupt (i.e. latch) mode
            inline static constexpr uint8_t fault_mode = 2;
            /// In interrupt (i.e. latch) mode, writing 1 clears fault status bits
            inline static constexpr uint8_t fault_status_clear = 1;
            /// 0 = 60Hz[default], 1 = 50 Hz
            inline static constexpr uint8_t noise_filter_freq = 0;
        };

        struct Config1 {
            /// Voltage conversion averaging mode
            inline static constexpr uint8_t conversion_mode = 4;
            // Thermocouple type
            inline static constexpr uint8_t thermocouple_type = 0;
        };

        ///
        struct FaultMask {
            /// Cold-Junction High Fault Threshold Mask
            inline static constexpr uint8_t cj_high =  5;
            /// Cold-Junction Low Fault Threshold Mask
            inline static constexpr uint8_t cj_low = 4;
            /// Thermocouple Temperature High Fault Threshold Mask
            inline static constexpr uint8_t tc_high = 3;
            /// Thermocouple Temperature Low Fault Threshold Mask
            inline static constexpr uint8_t tc_low = 2;
            /// Over-voltage or Undervoltage Input Fault Mask
            inline static constexpr uint8_t ov_uv = 1;
            /// Thermocouple Open-Circuit Fault Mask
            inline static constexpr uint8_t open_fault = 0;
        };

        struct FaultStatus {
            /// Cold junction out of range
            inline static constexpr uint8_t cj_out_of_rng = 7;
            /// Thermocouple out of range
            inline static constexpr uint8_t tc_out_of_rng = 6;
            /// Cold junction high
            inline static constexpr uint8_t cj_high = 5;
            /// Cold junction low
            inline static constexpr uint8_t cj_low = 4;
            /// Thermocouple high
            inline static constexpr uint8_t tc_high = 3;
            /// Thermocouple low
            inline static constexpr uint8_t tc_low = 2;
            /// Over/under voltage input
            inline static constexpr uint8_t ov_uv = 1;
            /// Thermocouple open circuit
            inline static constexpr uint8_t tc_oc = 0;
        };
    };

    /**
     *
     * @param spi Required SPI spec:
     *            - 8-bit
     *            - MSB first
     *            - 5 MHz max
     *            - CPHA bit = 1
     *            - any clock polarity (CPOL)
     *            - Tcwh: (CSinactive) min = 400ns
     *            - Tcc: (CSactive to SCLK) min = 100ns
     */
    Max31856(SpiRegisterOps<T>& spi) : spi_(spi) {}

    /**
     * Set the device configuration.
     *
     * @param mode    Sampling mode
     * @param tc_type Thermocouple type
     * @param c_mode  Voltage conversion averaging mode
     * @param filter  Noise rejection filter frequency
     */
    void configure(Mode mode, TcType tc_type, ConversionMode c_mode, NoiseFilter filter)
    {
        const OcFaultMode oc_fault = OcFaultMode::test_10ms; // pwon default is off
        const bool cj_sensor_disabled = false;
        const bool fault_status_clear = false;
        const bool use_interrupt_mode = false;

        uint8_t config0 =
                enumBaseT(c_mode)   << RegBitPos::Config0::cmode |
                enumBaseT(oc_fault) << RegBitPos::Config0::oc_fault |
                cj_sensor_disabled    << RegBitPos::Config0::cj_sensor_disable |
                use_interrupt_mode    << RegBitPos::Config0::fault_mode |
                fault_status_clear    << RegBitPos::Config0::fault_status_clear |
                // TODO: change notch freq in one-shot mode only
                enumBaseT(filter) << RegBitPos::Config0::noise_filter_freq;

        uint8_t config1 =
                enumBaseT(c_mode)  << RegBitPos::Config1::conversion_mode |
                enumBaseT(tc_type) << RegBitPos::Config1::thermocouple_type;

        // Unclear why, but first write fails sporadically so double up
        spi_.write(RegAddr::getWriteAddr(RegAddr::config_0), config0);
        spi_.write(RegAddr::getWriteAddr(RegAddr::config_0), config0);
        spi_.write(RegAddr::getWriteAddr(RegAddr::config_1), config1);

        //return spi_.read(RegAddr::config_0) == config0 && spi_.read(RegAddr::config_1) == config1;
    }

    /**
     *
     * @param offset MSB=4degC, LSB=0.0625degC
     */
    void setCjOffset(int8_t offset)
    {
        //10 = 0.6/7
        spi_.write(RegAddr::getWriteAddr(RegAddr::cj_offset), offset);
    }

    /**
     * Set the temperatures at which a fault status signal is generated.
     *
     * All values are whole degrees Celcius.
     *
     * @param tc_high Linearized thermocouple max allowed temp.
     * @param tc_low  Linearized thermocouple min allowed temp.
     * @param cj_high Cold junction max allowed temp.
     * @param cj_low  Cold junction min allowed temp.
     */
    void setFaultRanges(uint16_t tc_high, uint16_t tc_low, int8_t cj_high, int8_t cj_low)
    {
        // TODO: can we safely assume always 2's compliment?
        // TC reg uses most significant 12bits for integer portion
        // Least significant 4 bits are fractional portion and are unused here
        spi_.write(RegAddr::getWriteAddr(RegAddr::tc_temp_high_thresh_msb), tc_high >> 4);
        spi_.write(RegAddr::getWriteAddr(RegAddr::tc_temp_high_thresh_lsb), tc_high << 4);
        spi_.write(RegAddr::getWriteAddr(RegAddr::tc_temp_low_thresh_msb),  tc_low  >> 4);
        spi_.write(RegAddr::getWriteAddr(RegAddr::tc_temp_low_thresh_lsb),  tc_low  << 4);
        spi_.write(RegAddr::getWriteAddr(RegAddr::cj_temp_high_threshold),  cj_high);
        spi_.write(RegAddr::getWriteAddr(RegAddr::cj_temp_low_threshold),   cj_low);
    }

    /**
     * Set the value of the fault mask register. The fault mask determines
     * which error conditions will trigger the FAULT output on the device. See
     * datasheet for details.
     *
     * @param mask fault mask
     */
    void setFaultMask(uint8_t mask)
    {
        spi_.write(RegAddr::fault_mask, mask);
    }

    /**
     * Return the value of the fault status register. See data sheet for
     * details
     *
     * @return fault status register value
     */
    uint8_t readFaultStatus()
    {
        return spi_.read(RegAddr::fault_status);
    }

    /**
     * Read linearized thermocouple temperature register.
     *
     * @return encoded temperature in least significant 19-bits.
     *         - Most significant 12-bits = integral component, 2's compliment
     *           value
     *         - Least significant 7-bits = decimal component
     *           = unsigned value / 2^7
     */
    uint32_t readTemp()
    {
        uint8_t buf[3];
        spi_.read(RegAddr::tc_temp_hi_byte, buf, 3);
        return buf[0] << 11 | buf[1] << 3 | buf[2] >> 5;
    }

    /**
     * Read the cold-junction temperature register.
     *
     * Tcelcius = result * 0.015625
     *
     * @return
     */
    uint16_t readCjTemp()
    {
        uint8_t buf[2];
        spi_.read(RegAddr::cj_temp_hi_byte, buf, 2);
        return buf[0] << 6 | buf[1] >> 2;
    }

private:
    SpiRegisterOps<T>& spi_;
};

/**
 * Convert 19-bit result from @p readTemp to tenths of a degree Celcius.
 *
 * @param read_temp 19-bit temperature value as read from device
 *
 * @return temperature in tenths of a degree Celcius
 */
static constexpr int16_t decodeTemp(uint32_t read_temp)
{
    bool is_neg = read_temp & 0x40000;
    int32_t temp = ( (read_temp & 0xBFFFF) >> 7) * 10;
    temp += ((read_temp & 0b0111'1111) * 10) >> 7;
    if (is_neg)
        temp -= 20480;
    return temp;
}

namespace ConversionTests { // hide tests

struct Sample {
    float temp;
    int16_t tenths; // assumes truncated
    uint32_t binary;
};

// Samples from datasheet
constexpr uint8_t sample_size = 10;
constexpr Sample samples[sample_size] {
    { 1600.00,   16000, 0b0110'0100'0000'0000'0000'0000 },
    { 1000.00,   10000, 0b0011'1110'1000'0000'0000'0000 },
    {  100.9375,  1009, 0b0000'0110'0100'1111'0000'0000 },
    {   25.00,     250, 0b0000'0001'1001'0000'0000'0000 },
    {    0.0625,     0, 0b0000'0000'0000'0001'0000'0000 },
    {    0.00,       0, 0b0000'0000'0000'0000'0000'0000 },
    {   -0.0625,    -1, 0b1111'1111'1111'1111'0000'0000 },
    {   -0.25,      -3, 0b1111'1111'1111'1100'0000'0000 },
    {   -1.00,     -10, 0b1111'1111'1111'0000'0000'0000 },
    { -250.00,   -2500, 0b1111'0000'0110'0000'0000'0000 }
};

// Validate decodeTemp func against datasheet samples
constexpr bool validateSamples()
{
    for (uint8_t i = 0; i < sample_size; i++) {
        if ( decodeTemp(samples[i].binary >> 5) != samples[i].tenths )
            return false;
    }
    return true;
}
static_assert(validateSamples());

} // namespace ConversionTests
} // namespace Libp::Max31856

#endif /* SRC_DEVICES_MAX_31856_H_ */
