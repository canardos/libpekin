#ifndef LIB_LIBPEKIN_DRIVERS_CLOCK_RV8803_H_
#define LIB_LIBPEKIN_DRIVERS_CLOCK_RV8803_H_

#include <cstdint>
#include <cstring>
#include "libpekin.h"
#include "rv8803.h"
#include "bus/bus_concepts.h"
#include "datetime.h"
#include "misc_math.h"
#include "bits.h"
#include "serial/i2c.h"

namespace Libp::Rv8803 {

// Keep everything outside the class to avoid template param req.

#pragma pack(push, 1)
struct TimeData {
    uint8_t hundredths;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;  ///< 2^day_of_week where sun=0, sat=6. Output only.
    uint8_t day_of_month;
    uint8_t month;        ///< month [1.12]
    uint8_t year;         ///< two-digit year [00..99]

    static void bcd2bin(TimeData& time_data)
    {
        time_data.hundredths = Libp::bcd2bin(time_data.hundredths);
        time_data.seconds = Libp::bcd2bin(time_data.seconds);
        time_data.minutes = Libp::bcd2bin(time_data.minutes);
        time_data.hours = Libp::bcd2bin(time_data.hours);
        time_data.day_of_month = Libp::bcd2bin(time_data.day_of_month);
        time_data.month = Libp::bcd2bin(time_data.month);
        time_data.year = Libp::bcd2bin(time_data.year);
    }

    /**
     * @param [in,out] time_data day_of_week not required and will be set by this
     *                           function
     */
    static void bin2bcd(Libp::Rv8803::TimeData& time_data)
    {
        time_data.day_of_week = 1 << dayOfWeek(time_data.year, time_data.month, time_data.day_of_month);
        time_data.hundredths = Libp::bin2bcd(time_data.hundredths);
        time_data.seconds = Libp::bin2bcd(time_data.seconds);
        time_data.minutes = Libp::bin2bcd(time_data.minutes);
        time_data.hours = Libp::bin2bcd(time_data.hours);
        time_data.day_of_month = Libp::bin2bcd(time_data.day_of_month);
        time_data.month = Libp::bin2bcd(time_data.month);
        time_data.year = Libp::bin2bcd(time_data.year);
    }
};
#pragma pack(pop)
// This structure must be convertible to/from a byte array
static_assert(sizeof(TimeData) == 8);

struct Alarm {
    uint8_t minutes;
    uint8_t hours;
    uint8_t weekday;
    uint8_t date;
};

/// Periodic time update interrupt type
enum class PdTimeUpdIntType : uint8_t {
    second_update = 0,
    minute_update = 1
};


/**
 * Driver for the Micro Crystal RV8803 RTC.
 *
 * Based on:
 * Application Manual RV-8803-C7
 * Revision 1.6
 * May 2019
 *
 * @tparam I2cBus device supports <= 400k frequency
 */
template <I2cMaster I2cBus>
class Rv8803 {

public:

    enum class Reg : uint8_t {
        hundredths = 0x10, seconds, minutes, hours, weekday, date, month, year,
        alarm_mins, alarm_hours, alarm_wday_date,
        timer_0, timer_1,
        extensions, flags, control,
        hudredths_cp, seconds_cp,
        offset, event_ctrl
    };

    Rv8803(I2cBus& i2c, uint8_t i2c_addr = (0x64 >> 1)) : device_(i2c, i2c_addr) { };

    /**
     * Reads the complete time from the device.
     *
     * @param [out] time_data each field is in BCD format
     *
     * @return true on success, false on error (i2c error)
     */
    bool readTime(TimeData& time_data)
    {
        // confirm struct packing hasn't messed things up
        static_assert(sizeof(TimeData) == 8);
        if (!device_.readReg(enumBaseT(Reg::hundredths), reinterpret_cast<uint8_t*>(&time_data), 8))
            return false;
        // If we're about to tick over to a new minute, check that it didn't
        // happen while we were reading
        if (bcd2bin(time_data.seconds) == 59) {
            TimeData time_data2;
            if (!device_.readReg(enumBaseT(Reg::hundredths), reinterpret_cast<uint8_t*>(&time_data2), 8))
                return false;
            if (bcd2bin(time_data2.seconds) == 0) {
                memcpy(&time_data, &time_data2, 8);
            }
        }
        return true;
    }

    /**
     * Writes the time to the device. Hundredths are ignored and are set to
     * zero on the device.
     *
     * time_data.weekday will be updated with the correct weekday (0=Sunday)
     * based on the other date fields provided.
     *
     * No verification performed on time_data values.
     *
     * @param [in,out] time_data each field must be in BCD format
     *
     * @return true on success, false on error (i2c error)
     */
    bool writeTime(TimeData& time_data)
    {
        // confirm struct packing hasn't messed things up
        static_assert(sizeof(TimeData) == 8);
        // skip 100ths reg
        return device_.writeReg(enumBaseT(Reg::seconds), reinterpret_cast<uint8_t*>(&time_data) + 1, 7);
    }

    /// Type of alarm
    enum class AlarmType : uint8_t {
        wdays = 0,  ///< restrict alarm to one or more days of the week
        mday  = 1,  ///< restrict alarm to a specific day of the month
        daily = 2,  ///< alarm unrestricted by day
    };

    /**
     * Sets the alarm.
     *
     * Does not enable the alarm interrupt.
     *
     * It is recommended to disable the alarm interrupt prior to calling this
     * function to prevent inadvertent interrupts.
     *
     * There are three types of alarm:
     * - wdays  : restrict alarm to one or more days of the week
     * - mday   : restrict alarm to a specific day of the month
     * - daily  : alarm unrestricted by day
     *
     * @param [in] day_type
     * @param [in] day if day_type==wdays : [0..127] where each bit represents
     *                                      a weekday (bit 0 = sunday). 0 == all
     *                                      weekdays
     *                    day_type==mday  : [1..31] day of month
     *                    day_type==daily : not used
     * @param [in] use_hour restrict alarm to the specified hour
     * @param [in] hour [0..23]
     * @param [in] use_min restric alarm to the specified minute
     * @param [in] min [0..59]
     *
     * @return true on success, false on error (i2c error)
     */
    bool setAlarm(AlarmType day_type, uint8_t day, bool use_hour, uint8_t hour, bool use_min, uint8_t min)
    {
        // Set weekday or date alarm if required
        if (day_type != AlarmType::daily) {
            constexpr uint8_t mask = 1 << ext_wada_pos;
            constexpr uint8_t new_val = enumBaseT(day_type) << ext_wada_pos;
            if (device_.updateReg8(enumBaseT(Reg::extensions), mask, new_val) > 255)
                return false;
        }
        // Set alarm time/day
        uint8_t alarm_data[4];
        alarm_data[0] = enumBaseT(Reg::alarm_mins);
        alarm_data[1] = use_min << 7 | min;
        alarm_data[2] = use_hour << 7 | hour;
        alarm_data[3] = (day_type == AlarmType::daily) << 7 | day;
        return device_.writeReg8(enumBaseT(Reg::alarm_mins), alarm_data, 4);
    }

    /**
     * Set the type of the periodic time update interrupt.
     *
     * Does not enable the period time update interrupt.
     *
     * It is recommended to disable the periodic time update interrupt prior to
     * calling this function to prevent inadvertent interrupts.
     *
     * @param [in] type
     *
     * @return true on success, false on i2c error.
     */
    bool setPdTimeUpdInt(PdTimeUpdIntType type)
    {
        constexpr uint8_t mask = 1 << ext_usel_pos;
        uint8_t value = enumBaseT(type) << ext_usel_pos;
        return device_.updateReg(enumBaseT(Reg::extensions), mask, value) <= 255;
    }

    /// CLKOUT pin frequency
    enum class ClkOutFreq : uint8_t {
        freq_32768hz = 0b00,
        freq_1024hz = 0b01,
        freq_1hz = 0b10,
    };

    /**
     * Set the frequency of the CLKOUT pin.
     *
     * @param [in] freq
     *
     * @return true on success, false on i2c error.
     */
    bool setClkOutFreq(ClkOutFreq freq)
    {
        constexpr uint8_t mask = 0b11 << ext_fd_pos;
        uint8_t value = enumBaseT(freq) << ext_fd_pos;
        return device_.updateReg(enumBaseT(Reg::extensions), mask, value) <= 255;
    }

    /// Period countdown timer source clock
    enum class CntdnTmrClkFreq : uint8_t {
        freq_4096hz,
        freq_64hz,
        freq_1hz,
        freq_1_60hz ///< 1/60 Hz
    };

    /**
     * En/disable the countdown timer and set its period.
     *
     * Does not enable the countdown timer interrupt.
     *
     * It is recommended to disable the countdown timer interrupt prior to
     * calling this function to prevent inadvertent interrupts.
     *
     * Countdown period = tmr_value / clk_freq
     *
     * @param [in] enable
     * @param [in] tmr_value 12-bit value
     * @param [in] clk_freq
     *
     * @return true on success, false on i2c error.
     */
    bool setCountdownTmr(bool enable, uint16_t tmr_value, CntdnTmrClkFreq clk_freq)
    {
        if (   !device_.writeReg8(enumBaseT(Reg::timer_0), static_cast<uint8_t>(tmr_value))
            || !device_.writeReg8(enumBaseT(Reg::timer_1), static_cast<uint8_t>(tmr_value >> 8)) ) {
            return false;
        }
        constexpr uint8_t mask = (1 << ext_te_pos) | (0b11 << ext_td_pos);
        uint8_t value = (enable << ext_te_pos) | (enumBaseT(clk_freq) << ext_td_pos);
        return device_.updateReg(enumBaseT(Reg::extensions), mask, value) <= 255;
    }

    /// CLKOUT pin frequency
    enum class EventEdge : uint8_t {
        high_to_low = 0,
        low_to_high = 1,
    };
    enum class EventFilter : uint8_t {
        none = 0b00,
        sample_pd_3_9ms = 0b01,
        sample_pd_15_6ms = 0b10,
        sample_pd_125ms = 0b11
    };

    /**
     * En/disable external event capture.
     *
     * See datasheet for full details.
     *
     * @param [in] enable
     * @param [in] edge
     * @param [in] filter
     * @param [in] reset_on_evt
     *
     * @return true on success, false on error (i2c error)
     */
    bool setEventCapture(bool enable, EventEdge edge, EventFilter filter, bool reset_on_evt)
    {
        uint8_t reg_val = enable << event_ctrl_ecp_pos
                        | enumBaseT(edge) <<event_ctrl_ehl_pos
                        | enumBaseT(filter) << event_ctrl_et_pos
                        | reset_on_evt << event_ctrl_erst_pos;
        return device_.writeReg(enumBaseT(Reg::event_ctrl), reg_val);
    }

    /**
     *En/disable INT pin interrupt generation for various events.
     *
     * @param [in] pd_upd periodic time update interrupt enable
     * @param [in] cnt_dn_tmr countdown timer interrupt enable
     * @param [in] alarm alarm interrupt enable
     * @param [in] ext_event external event interrupt enable
     *
     * @return true on success, false on error (i2c error)
     */
    bool setInterrupts(bool pd_upd, bool cnt_dn_tmr, bool alarm, bool ext_event)
    {
        uint8_t reg_val = pd_upd << ctrl_uie_pos
                        | cnt_dn_tmr << ctrl_tie_pos
                        | alarm << ctrl_aie_pos
                        | ext_event << ctrl_eie_pos;
        return device_.writeReg(enumBaseT(Reg::control), reg_val);
    }

private:
    // extension register bits positions
    static constexpr uint8_t ext_wada_pos = 6;
    static constexpr uint8_t ext_usel_pos = 5;
    static constexpr uint8_t ext_te_pos = 4;
    static constexpr uint8_t ext_fd_pos = 2;
    static constexpr uint8_t ext_td_pos = 0;

    // flag register bits positions
    static constexpr uint8_t flag_uf_pos = 5;
    static constexpr uint8_t flag_tf_pos = 4;
    static constexpr uint8_t flag_af_pos = 3;
    static constexpr uint8_t flag_evf_pos = 2;
    static constexpr uint8_t flag_v2f_pos = 1;
    static constexpr uint8_t flag_v1f_pos = 0;

    // control register bits positions
    static constexpr uint8_t ctrl_uie_pos = 5;
    static constexpr uint8_t ctrl_tie_pos = 4;
    static constexpr uint8_t ctrl_aie_pos = 3;
    static constexpr uint8_t ctrl_eie_pos = 2;

    // event control register bit positions
    static constexpr uint8_t event_ctrl_ecp_pos = 7;
    static constexpr uint8_t event_ctrl_ehl_pos = 6;
    static constexpr uint8_t event_ctrl_et_pos = 4;
    static constexpr uint8_t event_ctrl_erst_pos = 0;

    I2cWrapper<I2cBus> device_;
};

} // namespace Libp::Rv8803

#endif /* LIB_LIBPEKIN_DRIVERS_CLOCK_RV8803_H_ */
