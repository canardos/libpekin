#ifndef LIB_LIBPEKIN_DRIVERS_INPUT_BME280_H_
#define LIB_LIBPEKIN_DRIVERS_INPUT_BME280_H_

#include <stddef.h>
#include <cstdint>
#include <algorithm>
#include "libpekin.h"
#include "bus/bus_concepts.h"


namespace Libp::Bme280 {

// Keep everything outside the class to avoid template param req.

/// Oversampling mode for humidity/temp/pressure measurements.
enum class OSample : uint8_t {
    none        = 0b000,
    osample_x1  = 0b001,
    osample_x2  = 0b010,
    osample_x4  = 0b011,
    osample_x8  = 0b100,
    osample_x16 = 0b101
};
/// Mode for ctrl_meas register
 enum class Mode : uint8_t {
    sleep = 0b00,
    forced = 0b01,
    normal = 0b11
};
/// Inactive duration Tstandby in normal mode (config register)
enum class TStandby : uint8_t {
    ts_0_5ms = 0b000,
    ts_62_5ms = 0b001,
    ts_125ms = 0b010,
    ts_250ms = 0b011,
    ts_500ms = 0b100,
    ts_1000ms = 0b101,
    ts_10ms = 0b110,
    ts_20ms = 0b111
};
/// Filter setting (config register)
enum class Filter : uint8_t {
    off = 0b000,
    coef_2 = 0b001,
    coef_4 = 0b010,
    coef_8 = 0b011,
    coef_16 = 0b100
};

struct Measurements {
    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;
};

inline constexpr uint8_t i2c_addr_sdo_gnd = 0x76;
inline constexpr uint8_t i2c_addr_sdo_vddio = 0x77;


/**
 * Basic driver for the Bosch BM280 temperature/humidity/pressure sensor.
 *
 * Reading from/writing to the device is done via an object provided on
 * construction that satisfies the `ReadWriteReg8bitAddr` concept.
 *
 * The BME280 supports the I²C and SPI digital interfaces; it acts as a slave
 * for both protocols.
 *
 * The I²C interface supports the Standard, Fast and High Speed modes.
 *
 * The SPI interface supports both SPI mode ‘00’ (CPOL = CPHA = ‘0’) and mode
 * ‘11’ (CPOL = CPHA = ‘1’) in 4-wire and 3-wire configuration.
 *
 * Based on:
 * BME280 - Data sheet
 * Revision 1.6
 * September 2018
 *
 * Warning: Largely untested
 */
template <ReadWriteReg8bitAddr Device>
class Bme280 {

public:
    enum class Reg : uint8_t {
        calib_temp_pr = 0x88, ///< start of temp/pressure calibration data block (0x88-0xa1)
        id = 0xd0,
        sreset = 0xe0,
        calib_hum = 0xe1,    ///< start of humidity calibration data block (0xe1 -> 0xf0)
        ctrl_hum = 0xf2, status, ctrl_meas, config,
        press_msb = 0xf7, press_xlsb, temp_msb, temp_lsb, hum_msb, hum_lsb
    };
    static constexpr uint8_t calib_temp_pr_len = 26;
    static constexpr uint8_t calib_hum_len = 16;

    static constexpr uint8_t reg_status_measuring_pos = 3;
    static constexpr uint8_t reg_status_im_update_pos = 0;
    static constexpr uint8_t reg_ctrl_meas_osrs_t_pos = 5;
    static constexpr uint8_t reg_ctrl_meas_osrs_p_pos = 2;
    static constexpr uint8_t reg_ctrl_meas_mode_pos = 0;
    static constexpr uint8_t reg_cfg_t_sb_pos = 5;
    static constexpr uint8_t reg_cfg_filter_pos = 2;
    static constexpr uint8_t reg_cfg_spi3w_en_pos = 0;
    static constexpr uint8_t reg_press_xlsb_pos = 4;
    static constexpr uint8_t reg_temp_xlsb_pos = 4;

    /**
     *
     * @param Device
     */
    Bme280(const Device& device) : device_(device) { };

    /// See page datasheet 24
    struct CompensationCoefficients {
        uint16_t dig_t1;
        int16_t dig_t2;
        int16_t dig_t3;
        uint16_t dig_p1;
        int16_t dig_p2;
        int16_t dig_p3;
        int16_t dig_p4;
        int16_t dig_p5;
        int16_t dig_p6;
        int16_t dig_p7;
        int16_t dig_p8;
        int16_t dig_p9;
        uint8_t dig_h1;
        int16_t dig_h2;
        uint8_t dig_h3;
        int16_t dig_h4;
        int16_t dig_h5;
        int8_t dig_h6;

        int32_t t_fine;   ///<  Intermediate temperature coefficient
    };

    /**
     * Calculate the max. active measurement time in milliseconds.
     *
     * @return max. active measurement time rounded up to the next whole
     *         millisecond
     */
    static constexpr uint16_t maxMeasureTime(
            bool hum, bool press, bool temp,
            uint8_t os_hum, uint8_t os_press, uint8_t os_temp)
    {
        return 1.25 + temp * (2.3 * os_temp) + press * (2.3 * os_press + 0.575) + hum * (2.3 * os_hum + 0.575) + 1;
    }
    static_assert(maxMeasureTime(true,true,true, 1,1,1) == 10);
    static_assert(maxMeasureTime(false,true,true, 1,4,1) == 14);

    /**
     * Performs a soft reset via the 0xe0 'reset" register. See datasheet for
     * details.
     *
     * @return true on success, false on communication failure or timeout
     *         waiting for reboot.
     */
    bool reset()
    {
        const uint8_t reset_cmd = 0xb6;
        if (!device_.writeReg(enumBaseT(Reg::sreset), &reset_cmd, 1))
            return false;
        // Wait for reboot to complete
        const uint8_t timeout_ms = 50;
        uint8_t waited_ms = 0;
        uint8_t status;
        while (waited_ms < timeout_ms) {
            delayMs(2);
            if (device_.readReg(enumBaseT(Reg::status), &status, 1)) {
                if ( !(status & (1 << reg_status_im_update_pos)) )
                    break;
            }
            waited_ms += 2;
        }
        return waited_ms < timeout_ms;
    }

    /**
     * Confirm communication with the sensor and initialize the compensation
     * data.
     *
     * @return true on success, false on communication failure.
     */
    bool initialize()
    {
        static constexpr uint8_t chip_id = 0x60;
        uint16_t id = readReg(Reg::id);
        if (id != chip_id) {
            return false;
        }
        return updateCalibration();
    }

    /**
     * Return the current operating mode.
     *
     * @param [out] mode
     *
     * @return true on success, false on communication failure.
     */
    bool getMode(Mode& mode)
    {
        uint16_t ctrl_meas = readReg(Reg::ctrl_meas);
        if (ctrl_meas > 255)
            return false;
        const uint8_t mask = 0b11 << reg_ctrl_meas_mode_pos;
        mode = static_cast<Mode>(ctrl_meas & mask);
        return true;
    }

    /**
     * Set the operating mode.
     *
     * @param [in] mode
     *
     * @return true on success, false on communication failure.
     */
    bool setMode(Mode mode)
    {
        // Datasheet page 15 recommended mode transitions suggests the only
        // unsupported transitions are:
        // - forced -> normal
        // - forced -> sleep
        //
        // The Bosch driver saves settings->resets->restores settings to get to
        // sleep mode prior to changing mode. It's not clear if this is
        // required or only done to avoid the above unsupported transitions.
        //
        // For applications polling via forced mode, this is likely to have
        // implications for power consumption given a 2ms startup time.
        //
        // TODO: test power consumption during startup vs this approach
        //
        // We instead check to see if the device is currently in forced mode,
        // and if it is, wait for a return to sleep before proceeding with the
        // mode change.
        //
        // TODO: test

        uint16_t ctrl_meas = readReg(Reg::ctrl_meas);
        if (ctrl_meas > 255)
            return false;

        // If in forced mode, wait for return to sleep
        const uint8_t mask = 0b11 << reg_ctrl_meas_mode_pos;
        while ( (ctrl_meas & mask) == enumBaseT(Mode::forced)) {
            //if (mode == Mode::forced)
            //    return true;
            delayUs(250);
            ctrl_meas = readReg(Reg::ctrl_meas);
            if (ctrl_meas > 255)
                return false;
        }
        // Nothing to do if current mode == requested mode
        if ((ctrl_meas & mask) == enumBaseT(mode))
            return true;
        // Update mode
        ctrl_meas = (ctrl_meas & ~mask) | enumBaseT(mode);
        return writeReg(Reg::ctrl_meas, ctrl_meas);
    }

    /**
     *
     * @param mode
     * @param os_hum
     * @param os_pres
     * @param os_temp
     * @param t_standby
     * @param filter
     * @param en_3wire
     *
     * @return true on success, false on communication failure.
     */
    bool setConfig(
            Mode mode,
            OSample os_hum, OSample os_pres, OSample os_temp,
            TStandby t_standby, Filter filter, bool en_3wire)
    {
        const uint8_t ctrl_hum = enumBaseT(os_hum);
        const uint8_t ctrl_meas_cfg[2] = {
                static_cast<uint8_t>(
                    enumBaseT(os_temp) << reg_ctrl_meas_osrs_t_pos
                  | enumBaseT(os_pres) << reg_ctrl_meas_osrs_p_pos
                  | enumBaseT(mode) << reg_ctrl_meas_mode_pos
                ),
                static_cast<uint8_t>(
                    enumBaseT(t_standby) << reg_cfg_t_sb_pos
                  | enumBaseT(filter) << reg_cfg_filter_pos
                  | en_3wire << reg_cfg_spi3w_en_pos
                )
        };
        // ctrl_hum write must be followed by write
        // to ctrl_meas to become effective.
        return writeReg(Reg::ctrl_hum, ctrl_hum)
            && device_.writeReg(enumBaseT(Reg::ctrl_meas), ctrl_meas_cfg, 2);
    }


    /**
     * Read and return the measurement registers.
     *
     * @param [out] result
     *
     * @return true on success, false on communication failure.
     */
    bool getMeasurements(Measurements& result)
    {
        // TODO: allow individual measurements
        constexpr uint8_t thp_reg_len = 8;
        uint8_t measurements[thp_reg_len];
        if (!device_.readReg(enumBaseT(Reg::press_msb), measurements, thp_reg_len))
            return false;

        RawData raw;
        // TODO; do we need to cast??
        raw.pres = measurements[0] << 12
                 | measurements[1] << 4
                 | measurements[2] >> 4;

        raw.temp = measurements[3] << 12
                 | measurements[4] << 4
                 | measurements[5] >> 4;

        raw.hum = measurements[6] << 8
                | measurements[7];

        doCompensation(raw, result);
        return true;
    }

private:
    struct RawData {
        uint32_t pres;
        uint32_t temp;
        uint32_t hum;
    };

    const Device& device_;
    CompensationCoefficients calib;

    bool updateCalibration()
    {
        constexpr uint8_t max_len = std::max(calib_temp_pr_len, calib_hum_len);
        uint8_t buffer[max_len];

        if (!device_.readReg(enumBaseT(Reg::calib_temp_pr), buffer, calib_temp_pr_len))
            return false;
        updateCalibTempPres(buffer);
        if (!device_.readReg(enumBaseT(Reg::calib_hum), buffer, calib_hum_len))
            return false;
        updateCalibHum(buffer);
        return true;
    }

    uint16_t readReg(Reg reg)
    {
        return device_.readReg(enumBaseT(reg));
    }
    bool writeReg(Reg reg, uint8_t value)
    {
        return device_.writeReg(enumBaseT(reg), &value, 1);
    }
    /*bool updateReg(Reg reg, uint8_t mask, uint8_t value)
    {
        uint16_t old_value = device_.updateReg(enumBaseT(reg), mask, value);
        return old_value <= 255;
    }*/

    void updateCalibTempPres(uint8_t* regs)
    {
        calib.dig_t1 = concat16u(regs[1], regs[0]);
        calib.dig_t2 = concat16(regs[3], regs[2]);
        calib.dig_t3 = concat16(regs[5], regs[4]);
        calib.dig_p1 = concat16u(regs[7], regs[6]);
        calib.dig_p2 = concat16(regs[9], regs[8]);
        calib.dig_p3 = concat16(regs[11], regs[10]);
        calib.dig_p4 = concat16(regs[13], regs[12]);
        calib.dig_p5 = concat16(regs[15], regs[14]);
        calib.dig_p6 = concat16(regs[17], regs[16]);
        calib.dig_p7 = concat16(regs[19], regs[18]);
        calib.dig_p8 = concat16(regs[21], regs[20]);
        calib.dig_p9 = concat16(regs[23], regs[22]);
        calib.dig_h1 = regs[25];
    }

    void updateCalibHum(uint8_t* regs)
    {
        calib.dig_h2 = concat16(regs[1], regs[0]);
        calib.dig_h3 = regs[2];

        int16_t msb8, lsb4;
        // TODO: lsb4 can be 8-bit

        msb8 = static_cast<int16_t>(regs[3] << 4);
        lsb4 = static_cast<int16_t>(regs[4] & 0x0f);
        calib.dig_h4 = msb8 | lsb4;

        msb8 = static_cast<int16_t>(regs[5] << 4);
        lsb4 = static_cast<int16_t>(regs[4] >> 4);
        calib.dig_h5 = msb8 | lsb4;

        calib.dig_h6 = static_cast<int8_t>(regs[6]);
    }


    void doCompensation(RawData& raw, Measurements& result)
    {
        result.temperature = compensateTemperature(raw);
        result.pressure = compensatePressure(raw);
        result.humidity = compensateHumidity(raw);
    }


    /*
     * The below compensation functions are adapted the Bosch BME280 driver
     * version 3.5.0:
     *
     * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
     *
     * BSD-3-Clause
     *
     * Redistribution and use in source and binary forms, with or without
     * modification, are permitted provided that the following conditions are met:
     *
     * 1. Redistributions of source code must retain the above copyright
     *    notice, this list of conditions and the following disclaimer.
     *
     * 2. Redistributions in binary form must reproduce the above copyright
     *    notice, this list of conditions and the following disclaimer in the
     *    documentation and/or other materials provided with the distribution.
     *
     * 3. Neither the name of the copyright holder nor the names of its
     *    contributors may be used to endorse or promote products derived from
     *    this software without specific prior written permission.
     *
     * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
     * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
     * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
     * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
     * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     * POSSIBILITY OF SUCH DAMAGE.
     */

    /**
     * @param raw raw measurements from the device.
     * @return the compensated temperature in integer data type.
     */
    int32_t compensateTemperature(const RawData& raw)
    {
        int32_t var1, var2;
        int32_t temperature;
        constexpr  int32_t temperature_min = -4000;
        constexpr  int32_t temperature_max = 8500;

        var1 = (int32_t)((raw.temp / 8) - ((int32_t)calib.dig_t1 * 2));
        var1 = (var1 * ((int32_t)calib.dig_t2)) / 2048;
        var2 = (int32_t)((raw.temp / 16) - ((int32_t)calib.dig_t1));
        var2 = (((var2 * var2) / 4096) * ((int32_t)calib.dig_t3)) / 16384;
        calib.t_fine = var1 + var2;
        temperature = (calib.t_fine * 5 + 128) / 256;

        if (temperature < temperature_min) {
            temperature = temperature_min;
        }
        else if (temperature > temperature_max) {
            temperature = temperature_max;
        }
        return temperature;
    }

    // 32-bit int version
    /**
     * @param raw raw measurements from the device.
     * @return the compensated pressure in integer data type.
     */
    uint32_t compensatePressure(const RawData& raw)
    {
        int32_t var1, var2, var3, var4;
        uint32_t var5;
        uint32_t pressure;
        constexpr  uint32_t pressure_min = 30000;
        constexpr  uint32_t pressure_max = 110000;

        var1 = (((int32_t)calib.t_fine) / 2) - (int32_t)64000;
        var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib.dig_p6);
        var2 = var2 + ((var1 * ((int32_t)calib.dig_p5)) * 2);
        var2 = (var2 / 4) + (((int32_t)calib.dig_p4) * 65536);
        var3 = (calib.dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
        var4 = (((int32_t)calib.dig_p2) * var1) / 2;
        var1 = (var3 + var4) / 262144;
        var1 = (((32768 + var1)) * ((int32_t)calib.dig_p1)) / 32768;

        // avoid exception caused by division by zero
        if (var1) {
            var5 = (uint32_t)((uint32_t)1048576) - raw.pres;
            pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

            if (pressure < 0x80000000) {
                pressure = (pressure << 1) / ((uint32_t)var1);
            }
            else {
                pressure = (pressure / (uint32_t)var1) * 2;
            }
            var1 = (((int32_t)calib.dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
            var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib.dig_p8)) / 8192;
            pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib.dig_p7) / 16));

            if (pressure < pressure_min) {
                pressure = pressure_min;
            }
            else if (pressure > pressure_max) {
                pressure = pressure_max;
            }
        }
        else {
            pressure = pressure_min;
        }
        return pressure;
    }

    /**
     * @param raw raw measurements from the device.
     * @return the compensated humidity in integer data type.
     */
    uint32_t compensateHumidity(const RawData& raw)
    {
        constexpr uint32_t humidity_max = 102400;
        int32_t var1, var2, var3, var4, var5;
        uint32_t humidity;

        var1 = calib.t_fine - ((int32_t)76800);
        var2 = (int32_t)(raw.hum * 16384);
        var3 = (int32_t)(((int32_t)calib.dig_h4) * 1048576);
        var4 = ((int32_t)calib.dig_h5) * var1;
        var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
        var2 = (var1 * ((int32_t)calib.dig_h6)) / 1024;
        var3 = (var1 * ((int32_t)calib.dig_h3)) / 2048;
        var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
        var2 = ((var4 * ((int32_t)calib.dig_h2)) + 8192) / 16384;
        var3 = var5 * var2;
        var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
        var5 = var3 - ((var4 * ((int32_t)calib.dig_h1)) / 16);
        var5 = (var5 < 0 ? 0 : var5);
        var5 = (var5 > 419430400 ? 419430400 : var5);
        humidity = (uint32_t)(var5 / 4096);

        if (humidity > humidity_max) {
            humidity = humidity_max;
        }
        return humidity;
    }

    static constexpr uint16_t concat16u(uint8_t msb, uint8_t lsb)
    {
        return static_cast<uint16_t>(msb << 8 | lsb);
    }
    static constexpr int16_t concat16(uint8_t msb, uint8_t lsb)
    {
        return static_cast<int16_t>(msb << 8 | lsb);
    }
};

} // namespace Libp::Bme280

#endif /* LIB_LIBPEKIN_DRIVERS_INPUT_BME280_H_ */
