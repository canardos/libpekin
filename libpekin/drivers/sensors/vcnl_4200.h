#ifndef SRC_VCNL_4200_H_
#define SRC_VCNL_4200_H_

#include <cstdint>
#include "libpekin.h"
#include "bits.h"
#include "bytes.h"
#include "bus/bus_concepts.h"

namespace Libp::Vcnl4200 {

// NOTE: enums and constants in namespace rather than class scope to avoid
//       needing template args when referencing everything.

/// VCNL4200 configuration registers
struct Vcnl4200Cfg {
    uint8_t registers[16];
};

/// Fixed VCNL4200 I2C slave address
inline constexpr uint8_t i2c_address = 0x51;

enum class Reg : uint8_t {
    // Read/write
    als_conf    = 0x00, ///< ALS integration time, persistence, interrupt, enable/disable
    als_thdh    = 0x01, ///< ALS interrupt threshold high
    als_thdl    = 0x02, ///< ALS interrupt threshold low
    ps_conf1_2  = 0x03, ///<
    ps_conf3_ms = 0x04, ///<
    ps_canc     = 0x05, ///< PS cancellation level setting
    ps_thdl     = 0x06, ///< PS interrupt threshold high
    ps_thdh     = 0x07, ///< PS interrupt threshold low
    // Read-only
    ps_data     = 0x08, ///< PS output data
    als_data    = 0x09, ///< ALS output data
    white_data  = 0x0a, ///< White output data
    int_flag    = 0x0d, ///< ALS/PS interrupt flags
    id          = 0x0e, ///< Device ID
};

/// Register bit positions
namespace Pos {
    // als_conf
    inline constexpr uint8_t als_it = 6;
    inline constexpr uint8_t als_it_switch = 5;
    inline constexpr uint8_t als_pers = 2;
    inline constexpr uint8_t als_int_en = 1;
    inline constexpr uint8_t als_sd = 0;
    // ps_conf1
    inline constexpr uint8_t ps_duty = 6;
    inline constexpr uint8_t ps_pers = 4;
    inline constexpr uint8_t ps_it = 1;
    inline constexpr uint8_t ps_sd = 0;
    // ps_conf2
    inline constexpr uint8_t ps_hd = 3;
    inline constexpr uint8_t ps_int = 0;
    // ps_conf3
    inline constexpr uint8_t ps_mps = 5;
    inline constexpr uint8_t ps_smart_pers = 4;
    inline constexpr uint8_t ps_af = 3;
    inline constexpr uint8_t ps_trig = 2;
    inline constexpr uint8_t ps_sc_adv = 1;
    inline constexpr uint8_t ps_sc_en = 0;
    // ps_ms
    inline constexpr uint8_t ps_ms = 5;
    inline constexpr uint8_t ps_sp = 4;
    inline constexpr uint8_t ps_spo = 3;
    inline constexpr uint8_t led_i = 0;
}

/// INT_Flag register bit positions
namespace IntFlagPos {
    inline constexpr uint8_t ps_upflag = 7;
    inline constexpr uint8_t ps_spflag = 6;
    inline constexpr uint8_t als_if_l = 5;
    inline constexpr uint8_t als_if_h = 4;
    inline constexpr uint8_t ps_if_close = 1;
    inline constexpr uint8_t ps_if_away = 0;
}

/// Integration time for the ambient light sensor. Longer times result in more
/// sensitive readings.
enum class AlsIntegTime : uint8_t {
    t50ms  = 0b00, ///<  50ms, 0.024 lux/step, 1573 lux max
    t100ms = 0b01, ///< 100ms, 0.012 lux/step,  786 lux max
    t200ms = 0b10, ///< 200ms, 0.006 lux/step,  393 lux max
    t400ms = 0b11  ///< 400ms, 0.003 lux/step,  197 lux max
};

/// Values for ALS reading <-> Lux conversion
static inline constexpr uint8_t als_integ_time_mlux_per_step[] = { 24, 12, 6, 3 };

enum class AlsPersistence : uint8_t {
    x1 = 0b00 << Pos::als_pers,
    x2 = 0b01 << Pos::als_pers,
    x4 = 0b10 << Pos::als_pers,
    x8 = 0b11 << Pos::als_pers
};

enum class PsIntTrigger : uint8_t {
    disabled         = 0b00 << Pos::ps_int,
    closing          = 0b01 << Pos::ps_int,
    away             = 0b10 << Pos::ps_int,
    closing_and_away = 0b11 << Pos::ps_int,
};
enum class PsSunCancelMode : uint8_t {
    // bit2 = ps_spo, bit1 = ps_sc_adv, bit0 = ps_sc_en
    disable    = 0b000,
    normal     = 0b001,
    extra      = 0b011,
    extra_plus = 0b111,
};
enum class PsMode : uint8_t {
    normal_with_int = 0,
    logic_output = 1,
};


enum class PsLedDuration : uint8_t {
    it_1t    = 0b000 << Pos::ps_it, ///< 30 uS
    it_1pt5t = 0b001 << Pos::ps_it, ///< 45 uS
    it_2t1   = 0b010 << Pos::ps_it, ///< 60 uS ??
    it_4t    = 0b011 << Pos::ps_it, ///< 120 uS
    it_8t    = 0b100 << Pos::ps_it, ///< 240 uS
    it_9t    = 0b101 << Pos::ps_it, ///< 270 uS
};
enum class PsLedDutyCycle : uint8_t {
    ratio_1_160  = 0b00 << Pos::ps_duty,
    ratio_1_320  = 0b01 << Pos::ps_duty,
    ratio_1_640  = 0b10 << Pos::ps_duty,
    ratio_1_1280 = 0b11 << Pos::ps_duty
};
enum class PsIntPersistence : uint8_t {
    x1 = 0b00 << Pos::ps_pers,
    x2 = 0b01 << Pos::ps_pers,
    x3 = 0b10 << Pos::ps_pers,
    x4 = 0b11 << Pos::ps_pers
};
enum class PsLedCurrent : uint8_t {
    i_50ma = 0,
    i_75ma = 1,
    i_100ma = 2,
    i_120ma = 3,
    i_140ma = 4,
    i_160ma = 5,
    i_180ma = 6,
    i_200ma = 7,
};
enum class PsLedNPulses : uint8_t {
    x1 = 0b00 << Pos::ps_mps,
    x2 = 0b01 << Pos::ps_mps,
    x4 = 0b10 << Pos::ps_mps,
    x8 = 0b11 << Pos::ps_mps
};

/// Proximity sensor LED settings for use with `psEnable` function.
struct LedSettings {
    PsLedDutyCycle duty_cycle;
    PsLedDuration duration;
    PsLedNPulses pulses;
    PsLedCurrent current;
};

static constexpr LedSettings default_led_settings = {
        .duty_cycle = PsLedDutyCycle::ratio_1_1280,
        .duration = PsLedDuration::it_1t,
        .pulses = PsLedNPulses::x1,
        .current = PsLedCurrent::i_50ma
};

/**
 * Convert raw ALS reading to an absolute millilux level.
 *
 * @param als_reading
 * @param integ_time integration time used for the ALS reading
 *
 * @return Light level in millilux
 */
static constexpr uint32_t alsToMilliLux(uint16_t als_reading, AlsIntegTime integ_time)
{
    return als_reading * als_integ_time_mlux_per_step[Libp::enumBaseT(integ_time)];
}


/**
 * Limited driver for the Vishay VCNL4200 proximity and ambient light sensor.
 *
 * Based on the following documents:
 *
 * title    : Datasheet: VCNL4200
 * source   : https://www.vishay.com/docs/84430/vcnl4200.pdf
 * date     : 16-Oct-2019
 * revision : 1.4
 *
 * title    : Application Note: Designing the VCNL4200 Into an Application
 * source   : https://www.vishay.com/docs/84327/designingvcnl4200.pdf
 * date     : 15-Oct-2019
 */
template <ReadWriteReg8bitAddr Device>
class Vcnl4200 {
public:

    Vcnl4200(Device& device) : device_(device) { }

    /**
     * Disable the ambient light sensor.
     *
     * @return true on success, false on communication failure.
     */
    bool alsDisable()
    {
        uint8_t als_conf[2] = { 0x01, 0 };
        return device_.writeReg(Libp::enumBaseT(Reg::als_conf), als_conf, 2);
    }

    /**
     * Enable the ambient light sensor.
     *
     * To easily define the threshold range, multiply the value of the resolution
     * (lx/step) by the threshold level.
     *
     * @param integration_time
     * @param int_en enable interrupts
     * @param thresh_low xxx
     * @param thresh_high xxx
     * @param persistence number of consecutive readings outside threshold required
     *        to trigger interrupt.
     *
     * @return true on success, false on communication failure.
     */
    bool alsEnable(AlsIntegTime integration_time = AlsIntegTime::t50ms,
            bool int_en = false, uint16_t thresh_low = 0, uint16_t thresh_high = 0,
            AlsPersistence persistence = AlsPersistence::x1)
    {
        const uint8_t als_conf_l = Libp::enumBaseT(integration_time)  << Pos::als_it
                                 // TODO: What is white channel interrupt vs ALS interrupt?
                                 //       Datasheet and app note both silent
                                 //| white_ch_int << als_it_switch
                                 | Libp::enumBaseT(persistence)
                                 | int_en << Pos::als_int_en
                                 | 0 << Pos::als_sd; // enable

        uint8_t als_conf[2] = { als_conf_l, 0 };
        uint8_t als_thdh[2] = { Libp::lowByte(thresh_high), Libp::highByte(thresh_high) };
        uint8_t als_thdl[2] = { Libp::lowByte(thresh_low),  Libp::highByte(thresh_low) };

        //__builtin_bswap16

        return device_.writeReg(Libp::enumBaseT(Reg::als_conf), als_conf, 2)
            && device_.writeReg(Libp::enumBaseT(Reg::als_thdh), als_thdh, 2)
            && device_.writeReg(Libp::enumBaseT(Reg::als_thdl), als_thdl, 2);
    }

    /**
     * Disable the proximity sensor.
     *
     * @return true on success, false on communication failure.
     */
    bool psDisable()
    {
        uint8_t ps_conf1[2] = { 0x01, 0 };
        return device_.writeReg(Libp::enumBaseT(Reg::ps_conf1_2), ps_conf1, 2);
    }

    /**
     * Enable the proximity sensor.
     *
     * Defaults:
     * - No sunlight cancellation
     * - No forced mode
     *
     * @param led LED settings.
     * @param int_trigger interrupt trigger type.
     * @param int_persist number of consecutive samples needed to trigger
     *                    interrupt.
     * @param smart_persist enable smart persist mode (faster int trigger
     *                      - see application note)
     * @param logic_mode use INT pin as proximity detect high/low output (can't
     *                   use ALS with INT func. in this mode)
     * @param hd_output 16-bit output (vs 12 default)
     *
     * @return true on success, false on communication failure.
     */
    bool psEnable(
            const LedSettings& led,
            PsIntTrigger int_trigger, PsIntPersistence int_persist,
            bool smart_persist = false, bool logic_mode = false, bool hd_output = false)
    {
        const uint8_t ps_conf1 = Libp::enumBaseT(led.duty_cycle)
                               | Libp::enumBaseT(int_persist)
                               | Libp::enumBaseT(led.duration)
                               | 0 << Pos::ps_sd; // enable

        const uint8_t ps_conf2 = hd_output << Pos::ps_hd
                               | Libp::enumBaseT(int_trigger);

        const uint8_t ps_conf3 = Libp::enumBaseT(led.pulses)
                               | smart_persist << Pos::ps_smart_pers;

        const uint8_t ps_ms = logic_mode << Pos::ps_ms
                            | Libp::enumBaseT(led.current) << Pos::led_i;

        uint8_t ps_conf1_2[2] =  { ps_conf1, ps_conf2 };
        uint8_t ps_conf3_ms[2] = { ps_conf3, ps_ms    };

        return device_.writeReg(Libp::enumBaseT(Reg::ps_conf1_2), ps_conf1_2, 2)
            && device_.writeReg(Libp::enumBaseT(Reg::ps_conf3_ms), ps_conf3_ms, 2);
    }

    /**
     * Enable the proximity sensor with default values.
     * - LED duty 1/1280
     * - LED duration 1T
     * - LED pulses 1
     * - LED current 50ma
     * - Interrupt disabled
     * - 12-bit resolution
     *
     * @return true on success, false on communication failure.
     */
    bool psEnable()
    {
        return psEnable(default_led_settings, PsIntTrigger::disabled, PsIntPersistence::x1);
    }

    /**
     * Set proximity sensor interrupt thresholds.
     *
     * @param low
     * @param high
     */
    bool psSetThresholds(uint16_t low, uint16_t high)
    {
        uint8_t ps_thdh[2] = { Libp::lowByte(high), Libp::highByte(high) };
        uint8_t ps_thdl[2] = { Libp::lowByte(low),  Libp::highByte(low) };
        return device_.writeReg(Libp::enumBaseT(Reg::ps_thdh), ps_thdh, 2)
            && device_.writeReg(Libp::enumBaseT(Reg::ps_thdl), ps_thdl, 2);
    }


    /**
     *
     * @param sun_cancel sunlight cancellation mode.
     * @param sun_hi_out if sunlight cancellation is used, output 0xFF on sun
     *                   protect instead of 0x00.
     * @param level
     *
     * @return
     */
    bool psSetSunCancel(
            PsSunCancelMode sun_cancel, bool sun_hi_out, uint16_t level)
    {
        uint8_t pos_conf3_ms[2];
        bool success = device_.readReg(Libp::enumBaseT(Reg::ps_conf3_ms), pos_conf3_ms, 2);
        if (!success) {
            return false;
        }
        constexpr uint8_t conf3_mask = 0b11;
        constexpr uint8_t ms_mask = 0b110000;

        Libp::Bits::setBits(
                pos_conf3_ms[0],
                conf3_mask,
                (Libp::enumBaseT(sun_cancel) & 0b11) << Pos::ps_sc_en);

        Libp::Bits::setBits(
                pos_conf3_ms[1],
                ms_mask,
                (Libp::enumBaseT(sun_cancel) & 0b100) >> 2 << Pos::ps_sp | sun_hi_out << Pos::ps_spo);

        uint8_t ps_canc[2] = { Libp::lowByte(level), Libp::highByte(level) };

        return device_.writeReg(Libp::enumBaseT(Reg::ps_conf3_ms), pos_conf3_ms, 2)
            && device_.writeReg(Libp::enumBaseT(Reg::ps_canc), ps_canc, 2);
    }

    /**
     *
     * @return 16-bit reading or -1 on error
     */
    int32_t alsRead()
    {
        uint8_t regs[2];
        return device_.readReg(Libp::enumBaseT(Reg::als_data), regs, 2)
                ? (regs[1] << 8) | regs[0]
                : -1;
    }

    /**
     *
     * @return 16-bit reading or -1 on error
     */
    int32_t psRead()
    {
        uint8_t regs[2];
        return device_.readReg(Libp::enumBaseT(Reg::ps_data), regs, 2)
                ? (regs[1] << 8) | regs[0]
                : -1;
    }

    /**
     *
     * @return INT flags or 255 on error
     */
    uint8_t intFlags()
    {
        uint8_t regs[2];
        return device_.readReg(Libp::enumBaseT(Reg::int_flag), regs, 2)
                ? regs[1]
                : 0xff;
    }

    /**
     * Configure all device parameters in one go.
     *
     * It is recommended to use the Vcnl4200CfgBuilder class to populate the
     * register values.
     *
     * @param config the register values
     *
     * @return true on success. false on communication failure.
     */
    bool configure(const Vcnl4200Cfg& config)
    {
        // Device only supports a single 16-bit register write at a time
        uint8_t addr = 0;
        while (addr < 8) {
         if (!device_.writeReg(addr, &config.registers[addr<<1], 2))
             return false;
         addr++;
        }
        return true;
    }

private:
    Device& device_;
};

/**
 * Class to build a `constexpr` Vcnl4200Cfg struct at compile time using typical
 * builder syntax.
 *
 * Using the builder will typically be more efficient in terms of flash usage
 * than calling individual configuration member functions.
 *
 * E.g.:
 *     using namespace Vcnl4200;
 *
 *     static constexpr Vcnl4200Cfg vcnl_cfg = []() {
 *         return Vcnl4200CfgBuilder()
 *              .alsEnable()
 *              .psEnable()
 *              .psLedSetDuty(PsLedDutyCycle::ratio_1_1280)
 *              .psLedSetDuration(PsLedDuration::it_1t)
 *              .psLedSetPulses(PsLedNPulses::x1)
 *              .psIntEnable(PsIntTrigger::closing, 10, 20, PsIntPersistence::x1)
 *              .build();
 *     }();
 *
 *     Vcnl4200 vcnl4200(i2c_wrapper);
 *     vcnl4200.configure(vcnl_cfg);
 *
 */
class Vcnl4200CfgBuilder {
private:
    bool als_en = false;
    AlsIntegTime als_integration_time = AlsIntegTime::t50ms;
    bool als_interrupt_en = false;
    AlsPersistence als_int_persist = AlsPersistence::x1;
    uint16_t als_thresh_low = 0;
    uint16_t als_thresh_high = 0;

    bool ps_en = false;
    PsLedDutyCycle ps_led_duty_cycle = PsLedDutyCycle::ratio_1_1280;
    PsLedDuration ps_led_duration = PsLedDuration::it_1t;
    PsLedNPulses ps_led_pulses = PsLedNPulses::x1;
    PsLedCurrent ps_led_current = PsLedCurrent::i_50ma;

    PsIntTrigger ps_int_trigger = PsIntTrigger::disabled;
    PsIntPersistence ps_int_persist = PsIntPersistence::x1;
    uint16_t ps_thresh_low = 0;
    uint16_t ps_thresh_high = 0;

    bool logic_mode = false;
    bool smart_persist = false;
    bool hd_mode = false;
public:
    constexpr Vcnl4200CfgBuilder& alsEnable(
            AlsIntegTime integration_time = AlsIntegTime::t50ms)
    {
        als_en = true;
        als_integration_time = integration_time;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& alsIntEnable(
            uint16_t thresh_low, uint16_t thresh_high,
            AlsPersistence persistence = AlsPersistence::x1)
    {
        als_en = true;
        als_interrupt_en = true;
        als_thresh_low = thresh_low;
        als_thresh_high = thresh_high;
        als_int_persist = persistence;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psEnable()
    {
        ps_en = true;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psIntEnable(
            PsIntTrigger int_trigger,
            uint16_t thresh_low, uint16_t thresh_high,
            PsIntPersistence int_persist = PsIntPersistence::x1)
    {
        ps_en = true;
        ps_int_trigger = int_trigger;
        ps_thresh_low = thresh_low;
        ps_thresh_high = thresh_high;
        ps_int_persist = int_persist;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psLedSetDuty(PsLedDutyCycle duty_cycle)
    {
        ps_led_duty_cycle = duty_cycle;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psLedSetDuration(PsLedDuration duration)
    {
        ps_led_duration = duration;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psLedSetPulses(PsLedNPulses pulses)
    {
        ps_led_pulses = pulses;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psLedSetCurrent(PsLedCurrent current)
    {
        ps_led_current = current;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psSmartPersistEnable()
    {
        smart_persist = true;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psLogicModeEnable()
    {
        logic_mode = true;
        return *this;
    }
    constexpr Vcnl4200CfgBuilder& psHdModeEnable()
    {
        hd_mode = true;
        return *this;
    }
    constexpr Vcnl4200Cfg build() const
    {
        const uint8_t als_conf_l = Libp::enumBaseT(als_integration_time)  << Pos::als_it
                                 // TODO: What is white channel interrupt vs ALS interrupt?
                                 //       Datasheet and app note both silent
                                 //| white_ch_int << als_it_switch
                                 | Libp::enumBaseT(als_int_persist)
                                 | als_interrupt_en << Pos::als_int_en
                                 | 0 << Pos::als_sd; // enable

        const uint8_t ps_conf1 = Libp::enumBaseT(ps_led_duty_cycle)
                               | Libp::enumBaseT(ps_int_persist)
                               | Libp::enumBaseT(ps_led_duration)
                               | (!ps_en) << Pos::ps_sd; // enable

        const uint8_t ps_conf2 = hd_mode << Pos::ps_hd
                               | Libp::enumBaseT(ps_int_trigger);

        const uint8_t ps_conf3 = Libp::enumBaseT(ps_led_pulses)
                               | smart_persist << Pos::ps_smart_pers;

        const uint8_t ps_ms = logic_mode << Pos::ps_ms
                            | Libp::enumBaseT(ps_led_current) << Pos::led_i;

        const Vcnl4200Cfg regs = { {
                als_conf_l,
                0,
                lowByte(als_thresh_high),
                Libp::highByte(als_thresh_high),
                Libp::lowByte(als_thresh_low),
                Libp::highByte(als_thresh_low),
                ps_conf1,
                ps_conf2,
                ps_conf3,
                ps_ms,
                0, // TODO; sun cancel
                0, // TODO; sun cancel
                Libp::lowByte(ps_thresh_low),
                Libp::highByte(ps_thresh_low),
                Libp::lowByte(ps_thresh_high),
                Libp::highByte(ps_thresh_high)
        } };
        return regs;
    }
};

} // namespace Libp::Vcnl4200

#endif /* SRC_VCNL_4200_H_ */
