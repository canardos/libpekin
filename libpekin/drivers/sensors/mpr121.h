#ifndef SRC_MPR121_H_
#define SRC_MPR121_H_

#include <cstdint>
#include "libpekin.h"
#include "serial/i2c.h"
#include "bus/bus_concepts.h"

namespace Libp {

/**
 * Basic driver for the MPR121 capacitive touch sensing IC.
 *
 * Provides initialization and basic electrode touch detection functions.
 *
 * Datasheet & Application Notes
 * -----------------------------
 * https://www.nxp.com/products/no-longer-manufactured/proximity-capacitive-touch-sensor-controller:MPR121
 */
template <I2cMaster I2cBus>
class Mpr121 {

public:

    /// Register addresses
    enum class Reg : uint8_t {
        touched_status_0_7 = 0x00,
        touched_status_8_12 = 0x01,

        auto_config_fail_0 = 0x02,
        auto_config_fail_1 = 0x03,

        max_half_delta_rising = 0x2b,
        noise_half_delta_rising = 0x2c,
        noise_count_rising = 0x2d,
        delay_limit_rising = 0x2e,

        max_half_delta_falling = 0x2f,
        noise_half_delta_falling = 0x30,
        noise_count_falling = 0x31,
        delay_limit_falling = 0x32,

        noise_half_delta_touched = 0x33,
        noise_counts_touched = 0x34,
        filter_delay_touched = 0x35,

        prox_max_half_delta_rising = 0x36,
        prox_noise_half_delta_rising = 0x37,
        prox_noise_count_rising = 0x38,
        prox_delay_limit_rising = 0x39,

        prox_max_half_delta_falling = 0x3a,
        prox_noise_half_delta_falling = 0x3b,
        prox_noise_count_falling = 0x3c,
        prox_delay_limit_falling = 0x3d,

        prox_noise_half_delta_touched = 0x3e,
        prox_noise_counts_touched = 0x3f,
        prox_filter_delay_touched = 0x40,

        elec_touch_threshold_0 = 0x41,  ///< + n*2 where n=electrode number [0..11]
        elec_release_threshold_0 = 0x42,///< + n*2 where n=electrode number [0..11]

        prox_touch_threshold = 0x59,
        prox_release_threshold = 0x5a,

        debounce_touch_rel = 0x5b,
        filter_cdc_config = 0x5c,
        filter_cdt_config = 0x5d,
        elec_config = 0x5e,
        elec_current_0 = 0x5f, ///< + n*2 where n=electrode number [1..11, 12 for prox]

        elec_charge_time_0_1 = 0x6c,

        gpio_ctrl_0 = 0x73,
        gpio_ctrl_1 = 0x74,
        gpio_data = 0x75,
        gpio_dir = 0x76,
        gpio_enable = 0x77,
        gpio_data_set = 0x78,
        gpio_data_clear = 0x79,
        gpio_data_toggle = 0x7a,
        auto_cfg_ctrl_0 = 0x7b,
        auto_cfg_ctrl_1 = 0x7c,
        auto_cfg_usl = 0x7d,
        auto_cfg_lsl = 0x7e,
        auto_cfg_tgt_lvl = 0x7f,
        soft_reset = 0x80 ///< doesn't affect I2C.
    };

    /// Overloaded + operator to offset from elec_touch/release_threshold_0
    /// registers.
    /// Performs normal arithmetic addition on the register address.
    friend inline Reg operator+(Reg reg, uint8_t n) {
        return static_cast<Reg>(static_cast<uint8_t>(reg) + n);
    }

    Mpr121(I2cBus& i2c, uint8_t i2c_addr = 0x5c)
            : device_(i2c, i2c_addr) { };

    /**
     * Reset the device and set appropriate defaults.
     *
     * An error return is likely the result of an I2C error, but may also
     * result from an auto configuration failure.
     *
     * @param touch_thresh touch threshold for all electrodes.
     * @param rel_thresh release touch threshold for all electrodes.
     *
     * @return true for a successful initialization, false otherwise.
     */
    bool initialize(uint8_t touch_thresh = 10, uint8_t rel_thresh = 8)
    {
        setReg(Reg::soft_reset, 0x63);
        delayMs(1);
        // Check single reg power on default to confirm comms.
        if (readReg(Reg::filter_cdt_config) != 0x24) {
            return false;
        }

        setReg(Reg::soft_reset, 0x63);
        delayMs(1);
        setReg(Reg::elec_config, 0x00);  //Stop mode so we can configure

        if constexpr (setup_type == SetupType::an600) {
            // touch pad baseline filter
            // rising
            setReg(Reg::max_half_delta_rising, 0x01);
            setReg(Reg::noise_half_delta_rising, 0x01);
            setReg(Reg::noise_count_rising, 0x00);
            setReg(Reg::delay_limit_rising, 0x00);


            // falling
            setReg(Reg::max_half_delta_falling, 0x01);
            setReg(Reg::noise_half_delta_falling, 0x01);
            setReg(Reg::noise_count_falling, 0xFF);
            setReg(Reg::delay_limit_falling, 0x02); //0

            // touched
            setReg(Reg::noise_half_delta_touched, 0x00);
            setReg(Reg::noise_counts_touched, 0x00);
            setReg(Reg::filter_delay_touched, 0x00);

            // Touch pad threshold
            for (uint8_t electrode = 0; electrode < 24; electrode += 2) {
                setReg(Reg::elec_touch_threshold_0 + electrode, touch_thresh);
                setReg(Reg::elec_release_threshold_0 + electrode, rel_thresh);
            }

            //AFE configuration
            setReg(Reg::filter_cdc_config, 0xC0);
            setReg(Reg::filter_cdt_config, 0x00);

            //Auto configuration
            setReg(Reg::auto_cfg_ctrl_0, 0xCB);
            setReg(Reg::auto_cfg_usl, 0xE4);
            setReg(Reg::auto_cfg_lsl, 0x94);
            setReg(Reg::auto_cfg_tgt_lvl, 0xCD);
            //    auto_cfg_ctrl_1 = 0x7c,

            // Enable detection
            setReg(Reg::elec_config, 0x0C);

            // Confirm auto-config success
            bool auto_cfg_failed = readReg(Reg::auto_config_fail_0) & readReg(Reg::auto_config_fail_1);
            if (auto_cfg_failed)
                return false;
        }
        else if constexpr (setup_type == SetupType::design_guide) {
            // Touch pad baseline filter

            // Rising: baseline quick rising
            setReg(Reg::max_half_delta_rising, 0x01);
            setReg(Reg::noise_half_delta_rising, 0x01);
            setReg(Reg::noise_count_rising, 0x00);
            setReg(Reg::delay_limit_rising, 0x00);

            // Falling: baseline slow falling
            setReg(Reg::max_half_delta_falling, 0x01);
            setReg(Reg::noise_half_delta_falling, 0x01);
            setReg(Reg::noise_count_falling, 0xFF);
            setReg(Reg::delay_limit_falling, 0x0);

            // Touched: baseline keep
            setReg(Reg::noise_half_delta_touched, 0x00);
            setReg(Reg::noise_counts_touched, 0x00);
            setReg(Reg::filter_delay_touched, 0x00);

            // Proximity baseline filter

            // Rising: very quick rising
            setReg(Reg::prox_max_half_delta_rising, 0x0f);
            setReg(Reg::prox_noise_half_delta_rising, 0x0f);
            setReg(Reg::prox_noise_count_rising, 0x00);
            setReg(Reg::prox_delay_limit_rising, 0x00);

            // Falling: very slow rising
            setReg(Reg::prox_max_half_delta_falling, 0x01);
            setReg(Reg::prox_noise_half_delta_falling, 0x01);
            setReg(Reg::prox_noise_count_falling, 0xff);
            setReg(Reg::prox_delay_limit_falling, 0xff);

            // Touched
            setReg(Reg::prox_noise_half_delta_touched, 0x00);
            setReg(Reg::prox_noise_counts_touched, 0x00);
            setReg(Reg::prox_filter_delay_touched, 0x00);

            // Touch pad threshold
            for (uint8_t electrode = 0; electrode < 24; electrode += 2) {
                setReg(Reg::elec_touch_threshold_0 + electrode, touch_thresh);
                setReg(Reg::elec_release_threshold_0 + electrode, rel_thresh);
            }

            // Proximity threshold
            constexpr uint8_t prox_touch_thresh =  6;
            constexpr uint8_t prox_rel_thresh =  4;
            setReg(Reg::prox_touch_threshold, prox_touch_thresh);
            setReg(Reg::prox_release_threshold, prox_rel_thresh);

            // Touch and release interrupt debounce
            setReg(Reg::debounce_touch_rel, 0x00); // Not used for polling method, effective for INT mode.

            // AFE and filter configuration
            setReg(Reg::filter_cdc_config, 0x10);  // AFES=6 samples, same as AFES in 0x7B, Global CDC=16uA
            setReg(Reg::filter_cdt_config, 0x24);  // CT=0.5us, TDS=4samples, TDI=16ms
            setReg(Reg::elec_config, 0x80);        // Set baseline calibration enabled,   baseline loading 5MSB
            //Auto Configuration
            setReg(Reg::auto_cfg_ctrl_0, 0x0B);    // AFES=6 samples, same as AFES in 0x5C
                                                   // retry=2b00, no retry,
                                                   // BVA=2b10, load 5MSB after AC,
                                                   // ARE/ACE=2b11, auto configuration enabled
          //setReg(Reg::7C,0x80);                  // Skip charge time search, use setting in 0x5D,
                                                   // OOR, AR, AC IE disabled
                                                   // Not used. Possible Proximity CDC shall over 63uA
                                                   // if only use 0.5uS CDT, the TGL for proximity cannot meet
                                                   // Possible if manually set Register0x72=0x03
                                                   // (Auto configure result) alone.
            setReg(Reg::auto_cfg_usl, 0xc8);       // AC up limit    /C8/BD/C0/9C
            setReg(Reg::auto_cfg_lsl, 0x82);       // AC low limit   /82/7A/7C/65
            setReg(Reg::auto_cfg_tgt_lvl, 0xb4);   // AC target     /B4/AA/AC/8C  target  for /3.0V/2.8V/1.8V
            setReg(Reg::elec_config, 0xBC);        // Run 12 touch + proximity, CL=2b10, load 5MSB to baseline }
        }
        return true;
    }

    /**
     * Return the current touched status for all electrodes.
     *
     * @return content of touch status registers 0 and 1 as a 16-bit word.
     *         Bits 11..0 = touch status of the 12 electrodes.
     *         Bit 12 = touch status of proximity sensor.
     *         Bit 14 = i2c timeout.
     *         Bit 15 = over current flag.
     *         See MPR121 datasheet for more details.
     */
    uint16_t isTouched()
    {
        uint16_t lsb, msb;
        lsb = readReg(Reg::touched_status_0_7);
        msb = readReg(Reg::touched_status_8_12);
        return (lsb > 255 || msb > 255)
                ? 1 << 14
                : msb << 8 | lsb;
    }

private:
    I2cWrapper<I2cBus> device_;
    enum class SetupType : uint8_t {
        an600,        ///< parameters from "AN600: Designing a Touch Panel
                      ///< using the Xtrinsic MPR121 Capacitive Touch Sensor
                      ///< Controller"
        design_guide, ///< parameters from "MPR121 Touch Sensing Design
                      ///< Guidelines"
    };
    static constexpr SetupType setup_type = SetupType::an600;

    uint16_t readReg(Reg reg)
    {
        return device_.readReg(enumBaseT(reg));
    }

    bool setReg(Reg reg, uint8_t value)
    {
        return device_.writeReg(enumBaseT(reg), value);
    }
};

} // namespace Libp

#endif /* SRC_MPR121_H_ */
