#ifndef LIBPEKIN_DEVICES_LP_ON_OFF_LEDS_H_
#define LIBPEKIN_DEVICES_LP_ON_OFF_LEDS_H_

#include "3rd_party/sg14/inplace_function.h"
#include "lp_types.h"
#include "lp_logging.h"
#include <cstdint>
#include <concepts>

#define DBG_PREFIX "LED: "

namespace libp::led {

struct BlinkCycle {
    /**
     * LED on time
     */
    uint16_t on_ms;
    /**
     * Off time between flashes if > 1 flash per period
     */
    uint16_t off_ms;
};

namespace blink_speed {
    inline constexpr BlinkCycle  pd_150ms_duty_33 {   50,  100 };
    inline constexpr BlinkCycle  pd_250ms_duty_25 {   50,  200 };
    inline constexpr BlinkCycle  pd_500ms_duty_25 {  125,  375 };
    inline constexpr BlinkCycle  pd_500ms_duty_50 {  250,  250 };
    inline constexpr BlinkCycle pd_1000ms_duty_25 {  250,  750 };
    inline constexpr BlinkCycle pd_1000ms_duty_50 {  500,  500 };
    inline constexpr BlinkCycle pd_2000ms_duty_25 {  500, 1500 };
    inline constexpr BlinkCycle pd_2000ms_duty_50 { 1000, 1000 };
} // blink_speed


/**
 * Callback at end of fixed duration light/blink is finished.
 */
using callback_t = stdext::inplace_function<void(), 16>;

/**
 * Provides on/off/timed on/flashing functionality for one or more LEDs.
 *
 * n_led + 1 timers are required. The index passed to start/stop_timer will be
 * [0 -> n_led].
 *
 * At timer expiration, the client code must call
 * `void processTmrEvent(uint8_t tmr_idx)`
 *
 * IMPORTANT: Not thread-safe. Assumes calls to on/off etc. and processTmrEvent
 * are made in the same context.
 *
 * @tparam n_led number of LEDs
 * @tparam set_led     Sig: `set_led(uint8_t led_idx, bool on)`
 * @tparam start_timer Sig: `start_timer(uint8_t tmr_idx, uint32_t delay_ms)`
 * @tparam stop_timer  Sig: `stop_timer(uint8_t tmr_idx)`
 */
template <
    uint8_t n_led,
    auto set_led,
    auto start_timer,
    auto stop_timer>
requires
    std::invocable<decltype(set_led),     uint8_t /*led_idx*/, bool /*on*/> &&
    std::invocable<decltype(start_timer), uint8_t /*tmr_idx*/, libp::no_narrowing<uint32_t> /*delay_ms*/> &&
    std::invocable<decltype(stop_timer),  uint8_t /*tmr_idx*/>
class OnOffLeds {
public:
    OnOffLeds() = default;

    // Prevent accidental copy/move
    OnOffLeds(const OnOffLeds&) = delete;
    OnOffLeds& operator=(const OnOffLeds&) = delete;
    OnOffLeds(OnOffLeds&&) = delete;
    OnOffLeds& operator=(OnOffLeds&&) = delete;

    /**
     * Set LED(s) on or off
     *
     * @param led 0-based LED index or 0xff for all LEDs
     * @param on  true for on, false for off
     */
    void set(uint8_t led_idx, bool on)
    {
        if (led_idx == 0xff) {
            for (uint8_t i = 0; i < n_led; i++) {
                set_led(i, on);
            }
        }
        else if (led_idx < n_led){
            set_led(led_idx, on);
        }
    }

    void allOff() { off(0xff); }

    /**
     * Turn off LED(s).
     *
     * @param led 0-based LED index or 0xff for all LEDs
     */
    void off(uint8_t led)
    {
        LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"off(%d)", led);
        if (led == 0xff) {
            for (uint8_t i = 0; i < n_led; i++) {
                off(i);
            }
            return;
        }
        if (led >= n_led) {
            return;
        }
        ledOff(led);
        state_[led] = State::off;

        stop_timer(led);

        // Need to stop the blink timer as start blink function uses the
        // timer state to determine if other LEDs are already blinking.
        bool should_stop_timer = true;
        for (uint8_t i = 0; i < n_led; i++) {
            if (state_[i] == State::blinking) {
                should_stop_timer = false;
                break;
            }
        }
        if (should_stop_timer) {
            stop_timer(n_led);
        }
    }

    /**
     * Turn on LED.
     *
     * @param led          0-based LED index or 0xff for all LEDs
     * @param duration_ms  Turn off after `duration_ms` milliseconds. 0 to
     *                     remain on.
     * @param callback     Function to call after `duration_ms` expires.
     */
    void on(uint8_t led, uint32_t duration_ms = 0, const callback_t callback = nullptr)
    {
        LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"on(%d)", led);

        if (led == 0xff) {
            for (uint8_t i = 0; i < n_led; i++) {
                on(i, duration_ms, i == 0 ? callback : nullptr);
                //callback = nullptr; // callback should only be called once
            }
            return;
        }
        if (led >= n_led) {
            return;
        }

        // There is no need to stop any timers here as both
        // off and blink timers are NOOPs if state is off
        //stop_timer(led);
        callback_[led] = callback; // clear any prior uncalled ptr
        if (duration_ms > 0) {
            state_[led] = State::timed_on;
            LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, "LED: starting on timer");
            start_timer(led, duration_ms);
        }
        else {
            state_[led] = State::on;
        }
        ledOn(led);
    }


    /**
     *
     *
     * @param led         0-based LED index or 0xff for all LEDs
     * @param blink_cycle LED on/off cycle durations.
     * @param num_periods The number of times to blink. 0 to blink until turned
     *                    off. 1 blink is a full cycle of on/off
     * @param callback
     */
    void startBlinking(uint8_t led,
                       BlinkCycle blink_cycle,
                       uint16_t num_periods,
                       const callback_t callback)
    {
        startBlinking(led, blink_cycle, num_periods, 0, 1, callback);
    }

    /**
     * Start blinking at the specified cycle/period for the specified duration.
     *
     * Terminology
     * -----------
     * - LED blink on/off is a blink cycle.
     * - There can be one or more blink cycles per period (a blink pattern)
     * - The period is the time until the blink pattern repeats.
     * - Blinking can continue indefinitely or for a set number of periods.
     *
     * For example, 2 quick flashes every 5 seconds (blink pattern) for 20
     * seconds (i.e. 4x 5s periods):
     * - blink_cycle       : { .on_ms = 100, .off_ms = 200 }
     * - num_periods       : 4
     * - cycles_per_period : 2
     * - period_ms         : 5000
     *
     * A single blink pattern is shared by all LEDs
     * --------------------------------------------
     * This call will update the blink settings for all LEDs (blink cycle,
     * blinks per period, period length).
     * If any LEDs are already blinking, then they will follow the new
     * settings, but will retain their number of blinks remaining.
     *
     * IMPORTANT: The remaining duration until the existing blinking LEDs stop
     * is based on the number of blink cycles (on/off) remaining at the time of
     * this call. An example:
     *
     * - Call 1: startBlinking(0, {100, 200}, 10, 5000, 2) // LED0 2x short
     *   blinks every 5s for 50s.
     *   * LED0 will blink a total of 20 times (2 x 10) over 50s.
     *   * LED1 off.
     *
     * - After 10 seconds and (4 blinks):
     *   * LED0 has 16 blinks remaining (40s),
     *   * LED1 off.
     *
     * - Call 2: startBlinking(1, {100, 200},  0, 2000, 4) // LED1 4x short
     *   blinks every 2s.
     *   * LED0 has 16 blinks remaining, which with the new settings (4 blinks
     *     every 2s), will finish in 8s.
     *   * LED1 is now blinking indefinitely with the new settings.
     *
     * Call is a NOOP if parameters are not valid.
     *
     * @param led               0-based LED index or 0xff for all LEDs
     * @param blink_cycle       LED on/off cycle durations.
     * @param num_periods       The number of times to blink. 0 to blink until
     *                          turned off. 1 blink is a full cycle of on/off
     * @param period_ms         See cycles_per_period. Cannot be 0 if
     *                          cycles_per_period > 1.
     *                          If 0, = period.on_ms + period.off_ms
     * @param cycles_per_period e.g. period = on 100ms, off 100ms,
     *                               cycles_per_period = 2, period_ms = 1000
     *                          - on 100ms / off 100ms / on 100ms / off 800ms
     * @param callback          Function to call after `count` blinks.
     */
    void startBlinking(uint8_t led,
                       BlinkCycle blink_cycle,
                       uint16_t num_periods = 0,
                       uint16_t period_ms = 0,
                       uint8_t cycles_per_period = 1,
                       const callback_t callback = nullptr)
    {
        if (led == 0xff) {
            for (uint8_t i = 0; i < n_led; i++) {
                startBlinking(i, blink_cycle, num_periods, period_ms, cycles_per_period, i == 0 ? callback : nullptr);
            }
            return;
        }
        if (led >= n_led) {
            return;
        }

        int32_t wait_ms = period_ms
                ? period_ms - (blink_cycle.on_ms + blink_cycle.off_ms) * cycles_per_period + blink_cycle.off_ms
                : blink_cycle.off_ms;

#define STRICT_PARAM_CHECK
#if defined(STRICT_PARAM_CHECK)
        if ( (period_ms == 0 && cycles_per_period > 1) ||
            led >= n_led || blink_cycle.on_ms == 0 || blink_cycle.off_ms == 0 || wait_ms <= 0 || cycles_per_period == 0) {

            LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"Invalid blink configuration");
            return;
        }
#endif
        ledOff(led);
        state_[led] = State::blinking;
        callback_[led] = callback;

        blink_timing_.on_ms = blink_cycle.on_ms;
        blink_timing_.off_ms = blink_cycle.off_ms;
        blink_timing_.wait_ms = static_cast<uint16_t>(wait_ms);
        blink_timing_.n_blink = cycles_per_period;

        remain_cycles_per_period_ = cycles_per_period;
        remain_half_cycles_[led] = num_periods * 2 * cycles_per_period;

        LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"LED %d start blinking:\r\n"
                "- on/off_ms     = %dms/%dms\r\n"
                "- wait_ms       = %dms\r\n"
                "- n_blink       = %d\r\n"
                "- remain_cycles = %d\r\n",
                led, blink_timing_.on_ms, blink_timing_.off_ms, blink_timing_.wait_ms,  blink_timing_.n_blink, remain_half_cycles_[led]);


        // IF another LED is already blinking, join it - don't reset the timer
        bool already_blinking = false;
        for (uint8_t i = 0; i < n_led; i++) {
            if (i != led && state_[i] == State::blinking) {
                already_blinking = true;
                break;
            }
        }

        if (already_blinking && blink_on_) {
            // Joining the existing pattern on
            ledOn(led);
        }
        else if (remain_half_cycles_[led] > 0) {
            // If we're not already on or are starting fresh:
            // First processBlinkTimerEvent handler call will
            // decrement but the light hasn't been turned
            // on yet, so add 1.
            remain_half_cycles_[led]++;
        }
        // If no LEDs are blinking, start
        if (!already_blinking) {
            blink_on_ = false; // Will be inverted in processBlinkTimerEvent
            processBlinkTimerEvent();
        }
    }

    void processTmrEvent(uint8_t tmr_idx)
    {
        if (tmr_idx == n_led) {
            processBlinkTimerEvent();
        }
        else {
            processOffTimerEvent(tmr_idx);
        }
    }

private:
    __attribute__((always_inline))
    inline void ledOn(uint8_t led_idx)  { set_led(led_idx, true ); }
    __attribute__((always_inline))
    inline void ledOff(uint8_t led_idx) { set_led(led_idx, false); }

    void processBlinkTimerEvent()
    {
        LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"processBlinkTimerEvent()");
        blink_on_ = !blink_on_;
        bool still_blinking = false;

        for (uint8_t i = 0; i < n_led; i++) {
            if (state_[i] == State::blinking) {
                // LED was blinking, but has finished
                if (stopBlinkIfCyclesCompleted(i)) {
                    LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"LED %i has stopped blinking", i);
                }
                // LED is blinking and hasn't finished
                else {
                    LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"LED %d is still blinking", i);
                    still_blinking = true;
                    if (blink_on_) {
                        LP_LOG_RAW_IF(EN_LOG_ONOFFLED, Log::Level::DEBUG, "- Turning on ");
                        ledOn(i);
                    }
                    else {
                        // Need to move this outside the loop
                        //remain_cycles_per_period_--;
                        LP_LOG_RAW_IF(EN_LOG_ONOFFLED, Log::Level::DEBUG, "- Turning off ");
                        ledOff(i);
                    }
                }
            }
            else {
                LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"- not blinking", i);
            }
        }

        if (still_blinking) {
            // Decrement on the off half cycle
            if (!blink_on_) {
                remain_cycles_per_period_--;
            }
            LP_LOG_RAW_IF(EN_LOG_ONOFFLED, Log::Level::DEBUG,
                    "for %d ms\r\n"
                    "- blink_on = %u\r\n"
                    "- remain_cycles_per_period = %u\r\n"
                    "- remain_cycles = %u\r\n",
                    blink_on_ ? blink_timing_.on_ms
                              : remain_cycles_per_period_ > 0 ? blink_timing_.off_ms : blink_timing_.wait_ms,
                    blink_on_,
                    remain_cycles_per_period_,
                    remain_half_cycles_[0]
            );

            uint32_t ms = blink_on_
                    ? blink_timing_.on_ms
                    : remain_cycles_per_period_ > 0 ? blink_timing_.off_ms : blink_timing_.wait_ms;

            LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"Starting blink timer (%u ms)", ms);
            start_timer(n_led, ms);

            if (remain_cycles_per_period_ == 0) {
                remain_cycles_per_period_ = blink_timing_.n_blink;
            }
        }
    }

    void processOffTimerEvent(uint8_t led_idx)
    {
        LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"LED %d duration expired. Turning off", led_idx);
        turnOffAndExecuteCallback(led_idx);
        stop_timer(led_idx); // clear expired flag
    }

    /** @return true if cycles are completed and LED has been turned off */
    bool stopBlinkIfCyclesCompleted(uint8_t led_idx)
    {
        // Stop if enough cycles done
        if (remain_half_cycles_[led_idx] > 0) {
            remain_half_cycles_[led_idx]--;
            if (remain_half_cycles_[led_idx] == 0) {
                turnOffAndExecuteCallback(led_idx);
                return true;
            }
        }
        return false;
    }
    void turnOffAndExecuteCallback(uint8_t led_idx)
    {
        off(led_idx);
        if (callback_[led_idx]) {
            // The callback may set another callback,
            // so reset before call, not after.
            callback_t cb = std::move(callback_[led_idx]);
            callback_[led_idx] = nullptr;
            LP_LOG_DEBUG_IF(EN_LOG_ONOFFLED, DBG_PREFIX"Executing callback");
            cb();
        }
    }

private:
    enum class State : uint8_t {
        off = 0,
        on,
        timed_on,
        blinking
    };

    /**
     * E.g.
     * on_ms   =  100
     * off_ms  =  200
     * wait_ms = 4600
     * n_blink =    2
     *
     * ON 100ms / OFF 200ms / ON 100ms / OFF 4600ms
     */
    struct BlinkTiming {
        uint16_t on_ms;   // time on each blink
        uint16_t off_ms;  // off in between blinks within period
        uint16_t wait_ms; // off time at end of period
        uint8_t n_blink;  // number of blinks per period
    };

    State state_[n_led]{State::off};

    // -- Blinking data

    BlinkTiming blink_timing_{};
    uint8_t remain_cycles_per_period_{0};

    // Total number of pulse/blink cycles remaining (on/off = 2 cycles)
    uint16_t remain_half_cycles_[n_led]{};
    bool blink_on_{false};

    callback_t callback_[n_led];
};

} // namespace libp::led

#undef DBG_PREFIX

#endif /* LIBPEKIN_DEVICES_LP_ON_OFF_LEDS_H_ */
