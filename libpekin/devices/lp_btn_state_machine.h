/*
 * lp_btn_state_machine.h
 */
#ifndef LIBPEKIN_DEVICES_BTN_STATE_MACHINE_H_
#define LIBPEKIN_DEVICES_BTN_STATE_MACHINE_H_

#include "lp_logging.h"
#include <cstdint>
#include <tuple>

#define DBG_PREFIX "BSM: "

namespace libp::btn {

enum class BtnMode: uint8_t {
    /**
     * TAP sent on UP after timeout  waiting for additional taps, or on DOWN if
     * already at maximum number of allowed taps.
     */
    tap,
    /**
     * TAP sent on UP after timeout waiting for additional taps, or on DOWN if
     * already at maximum number of taps. HOLD sent when button is DOWN
     * for >= min_hold_duration_ms. RELEASE is sent on UP after a HOLD event.
     */
    tap_and_hold,
    /**
     * HOLD is sent when button is DOWN for >= min_hold_duration_ms, or on DOWN
     * if already at maximum number of allowed taps RELEASE is sent on UP after
     * a HOLD event.
     */
    moment
};


/**
 * Global config data for all buttons
 */
struct BtnCfg {
    static constexpr uint16_t default_max_time_btn_taps_ms  =  500;
    static constexpr uint16_t default_min_hold_duration_ms  = 1100;

    uint16_t max_time_btn_taps_ms = default_max_time_btn_taps_ms;
    uint16_t min_hold_duration_ms = default_min_hold_duration_ms;
};


/**
 * Single button config data
 */
struct BtnInstanceCfg {
    // Be careful if increasing this. Client code might assume
    // it as a max for arrays etc.
    static constexpr uint8_t max_configurable_btn_taps = 3;

    /**
     * Configured max number of taps for this button.
     * Excludes system functions / menus
     *
     * <b>MUST</b> be >=1 and <= max_configurable_btn_taps
     */
    uint8_t max_taps = 3;

    BtnMode mode[max_configurable_btn_taps] = {}; // [0,0,0,...] = tap mode
};


// outside class given tparams
enum class BtnMenuEvtResult : uint8_t {
    /** Do nothing */
    none,
    /** dispatch the event */
    event,
    /** exit the menu */
    exit,
    /** dispatch the event and exit the menu */
    event_and_exit
};

/**
 * There is no multi-tap in menus.
 */
template<typename EventType>
class BtnMenu {
public:
    virtual ~BtnMenu() = default;
    /**
     * Called by the button state manager when the menu is activated.
     */
    virtual void onMenuEntry(uint8_t btn_idx) = 0;
    /**
     * Called by the button state manager when the menu has exited.
     *
     * If the exit is due to an event_and_exit return from processBtnEvent,
     * this method will be called AFTER the event is sent.
     *
     * @param btn_idx
     */
    virtual void onMenuExit(uint8_t btn_idx) = 0;
    /**
     * The BtnStateMachine instance passes TAP/HOLD/RELEASE events to the menu
     * here.
     *
     * @return {menu result, event type to send (if any), n_tap to send)
     */
    virtual std::tuple<BtnMenuEvtResult, EventType, uint8_t> processBtnEvent(EventType evt) = 0;
};


template<typename EventType>
struct BtnMenuEntry {
    BtnMenu<EventType>& menu;
    /** MUST be > BtnInstanceCfg.max_taps */
    uint8_t taps_to_enter;
    /** Menu is activated by holding on the nth tap */
    bool hold_needed = true;
    /** Menu auto exits after this duration of no input events */
    uint32_t inactive_expiry_ms = 0;
};

template<typename EventType>
struct BtnEvents {
    EventType tap;
    EventType hold;
    EventType release;
};

constexpr uint8_t inactivity_timer_btn_idx = 0xff;

namespace internal {
/**
 * Defined outside of class to ensure single instance even in case of
 * accidental multiple template instantiation.
 *
 * if != 0xff, then only the button with this index will respond
 * to button events sent via `processBtnEvent`.
 *
 * Used to prevent other buttons operating when one button has a menu
 * active.
 */
inline uint8_t cur_excl_mode_btn_id_ = 0xff;
} // namespace internal


/**
 * State machine and event dispatcher for a single button. Provides hold and
 * multi-tap functionality.
 *
 *
 * =======================
 * Theory of operation
 * =======================
 *
 * The configuration is very flexible, at the expense of being somewhat
 * confusing.
 *
 * ------
 * Events
 * ------
 *
 * Input  : DOWN/UP
 * Output : TAP/HOLD/RELEASE (with num_taps = 1/2/3)
 *        : Menu events
 *
 * ---------------
 * Operating modes
 * ---------------
 *
 * There are 3 operating modes: Tap / Tap+Hold / Moment.
 *
 * 1) With multi-tap disabled (i.e. max_taps == 1)
 * -----------------------------------------------
 *
 * With multi-tap support disabled the modes are straightforward:
 *
 * --------------------------------------------------------
 * Mode       Input                                 Output
 * --------------------------------------------------------
 * Tap      : DOWN                                  TAP
 * Tap+Hold : DOWN->UP within min_hold_duration_ms  TAP
 *          : DOWN for >= min_hold_duration_ms      HOLD
 *          : UP after a HOLD event                 RELEASE
 * Moment   : DOWN                                  HOLD
 *          : UP                                    RELEASE
 * --------------------------------------------------------
 *
 *
 * 2) With multi-tap enabled (i.e. max_taps > 1)
 * ---------------------------------------------
 *
 * When multiple taps are enabled (max_taps > 1), the mapping is more complex,
 * as the device must wait after a tap (DOWN->UP) event to determine if more
 * taps are coming. It waits max_time_btn_taps_ms.
 *
 * The exception is when the tap count reaches max_taps, at which point it is no
 * longer necessary to wait for additional taps.
 *
 * For this reason, max_taps == 1 (i.e. multi-tap disabled) provides a more
 * responsive experience.
 *
 * An example helps illustrate this:
 *
 * Settings : max_taps == 3
 *            Tap+hold mode for all taps
 *
 * Single-tap
 * - User presses DOWN->UP.
 * - The device waits max_time_btn_taps_ms to see if further taps are coming.
 * - After max_time_btn_taps_ms has elapsed, it sends a TAP event with
 *   num_taps = 1.
 *
 * Double-tap
 * - User presses DOWN->UP.
 * - The device waits max_time_btn_taps_ms to see if further taps are coming.
 * - The user presses DOWN->UP again before max_time_btn_taps_ms has elapsed.
 * - The device waits max_time_btn_taps_ms to see if further taps are coming.
 * - After max_time_btn_taps_ms has elapsed, it sends a TAP event with
 *   num_taps = 2.
 *
 * Triple-tap
 * - User presses DOWN->UP.
 * - The device waits max_time_btn_taps_ms to see if further taps are coming.
 * - The user presses DOWN->UP again before max_time_btn_taps_ms has elapsed.
 * - The device waits max_time_btn_taps_ms to see if further taps are coming.
 * - The user presses DOWN->UP again before max_time_btn_taps_ms has elapsed.
 * - Since a maximum of 3-taps is supported, the device does NOT wait to see
 *   if further taps are coming, and sends the TAP event with num_taps = 3
 *   immediately[1].
 *
 * [1] If the device was in tap only mode, the TAP event would be sent on the
 *     DOWN event, since there is no possibility of additional taps (already at
 *     max of 3) and no need to wait for hold event.
 *
 * With multi-tap enabled, the input->output mapping is more complex:
 *
 * ----------------------------------------------------------------------------
 * Mode       Event                 # taps preceding UP/DOWN
 * ----------------------------------------------------------------------------
 *                     #taps  < max_taps            #taps == max_taps
 *                     ---------------------------  ---------------------------
 * Tap      : TAP     : DOWN->UP[1]                 DOWN
 *          :         :
 * Tap+hold : TAP     : DOWN->UP within             same as < max_taps
 *          :         : min_hold_duration_ms[1]
 *          : HOLD    : DOWN for                    same as < max_taps
 *          :         : >= min_hold_duration_ms
 *          : RELEASE : UP after a HOLD event
 *          :         :
 * Moment   : HOLD    : DOWN for                    DOWN
 *          :         : >= min_hold_duration_ms
 *          : RELEASE : UP after a HOLD event       UP
 * ------------------  ---------------------------  ---------------------------
 *
 * [1] Event is sent after max_time_btn_taps_ms has expired.
 *
 * The mode can be different for different numbers of taps - e.g. the first tap
 * is tap-only mode, but the second is moment mode.
 *
 *
 * =======================
 * Menus
 * =======================
 *
 * The menu system allows for a multi-tap/hold sequence to "enter" a custom
 * menu, after which all TAP/HOLD/RELEASE events are sent to the BtnMenu
 * object. The menu object processes the events and optionally returns events
 * to be dispatched to the application via on_btn_event.
 *
 * Menus are exclusive and when one button is in menu mode, UP/DOWN input
 * events on other buttons (i.e. BtnStateMachine instances) will be ignored.
 *
 * The menus are provided on construction wrapped in a BtnMenuEntry. This
 * object holds the menu instance as:
 * - taps_to_enter
 * - hold_needed
 * - inactive_expiry_ms
 *
 * e.g.
 * - taps_to_enter = 5, hold_needed = true:
 *   The menu is activated by 4 taps (DOWN->UP within min_hold_duration_ms),
 *   followed by a hold (DOWN for > min_hold_duration_ms).
 *
 * - taps_to_enter = 7, hold_needed = false:
 *   The menu is activated by 7 taps (DOWN->UP within min_hold_duration_ms).
 *
 * Once a menu is active, all OUTPUT events are sent to the menu object via
 * its std::tuple<BtnMenuEvtResult, T> processBtnEvent(T evt) member, and are
 * not sent to the usual on_btn_event callback.
 *
 * When a menu is active, it receives the following events:
 *
 * ------------------------------------------------------
 * Input                                 Output (to menu)
 * ------------------------------------------------------
 * DOWN                                  -
 * DOWN->UP within min_hold_duration_ms  TAP
 * DOWN for >= min_hold_duration_ms      HOLD
 * UP after a Hold event                 RELEASE
 * ------------------------------------------------------
 *
 * There is no multi-tap in menus.
 *
 * Menu exit
 * ---------
 * The menu is exited when:
 * - The menu object returns BtnMenuEvtResult::exit or
 *   BtnMenuEvtResult::event_and_exit from a call to menu_obj.processBtnEvent.
 * - The menu inactivity timer (if present) expires.
 *
 *
 * =======================
 * Usage
 * =======================
 *
 * Input events
 * ------------
 * Button UP/DOWN events must be reported to this class via the
 * processBtnEvent method and the class will dispatch multi-tap/hold etc.
 * events via the on_btn_event function template param.
 *
 * NOTE:
 * - on_btn_event, start_timer and stop_timer should be shared across instances
 *   to prevent multiple template instantiations.
 * - processBtnEvent / processTmrExpiry should be called from the same context
 *   as there is no concurrency protection.
 *
 * @tparam on_btn_event
 *         `on_btn_event(BtnEvent, uint8_t btn_idx, uint8_t num_taps)`
 *         Callback to receive tap/hold/menu etc. events.
 * @tparam start_timer
 *         `start_timer(uint8_t btn_idx, uint32_t delay_ms)`
 *         fn to start a one-shot timer of the specified duration.
 *         At expiration, `processTmrExpiry` must be called.
 *         btn_idx will be EITHER this object's btn_idx OR 0xff.
 *         0xff is for a menu inactivity timer that is shared across all
 *         instances of this class.
 *         Thus a total of num_button_instances + 1 timers are required.
 *         At the expiration of the menu inactivity timer, processTmrExpiry(true)
 *         must be called on all instances.
 *
 * @tparam stop_timer
 *         `stop_timer (uint8_t btn_idx)`
 *         fn to cancel an outstanding timer started by a prior call to
 *         start_timer.
 *         stop_timer must be a NOOP if the timer is not active.
 */
template<
    typename EventType,
    auto on_btn_event,
    auto start_timer,
    auto stop_timer>
requires
requires(EventType event, uint8_t btn_idx, uint8_t num_taps, uint32_t delay_ms) {
    on_btn_event(event, btn_idx, num_taps);
    start_timer(btn_idx, delay_ms);
    stop_timer(btn_idx);
}
class BtnStateMachine {
public:

    /**
     * Construct a BtnStateMachine for a single button.
     *
     * @param btn_idx
     * @param btn_cfg reference retained. Live changes permitted
     * @param btn_evts
     * @param menus reference retained, must remain allocated
     * @param num_menus
     */
    constexpr BtnStateMachine(
            uint8_t btn_idx,
            const BtnInstanceCfg& btn_cfg,
            const BtnCfg& btn_cfg_global,
            const BtnEvents<EventType> btn_evts,
            const BtnMenuEntry<EventType>* menus = nullptr, uint8_t num_menus = 0)
                : btn_idx_(btn_idx),
                  btn_cfg_(btn_cfg),
                  btn_cfg_global_(btn_cfg_global),
                  btn_evts_(btn_evts),
                  menus_(menus), num_menus_(num_menus)
    {
        LP_ASSERT(menus != nullptr || num_menus == 0);
    }

    // Prevent accidental copy/move
    BtnStateMachine(const BtnStateMachine&) = delete;
    BtnStateMachine& operator=(const BtnStateMachine&) = delete;
    BtnStateMachine(BtnStateMachine&&) = delete;
    BtnStateMachine& operator=(BtnStateMachine&&) = delete;

private:

    void stopTimer()
    {
        stop_timer(btn_idx_);
    }
    void startHoldTimer()
    {
        stopTimer();
        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Start hold timer");
        start_timer(btn_idx_, btn_cfg_global_.min_hold_duration_ms);
    }
    void startMultitapTimer()
    {
        stopTimer();
        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Start multi-tap timer");
        start_timer(btn_idx_, btn_cfg_global_.max_time_btn_taps_ms);
    }


    // Menus only handle tap, hold, release for single tap
    void handleBtnEventForMenu(bool down)
    {
        restartMenuInactivityTimer();
        // BUTTON DOWN
        if (down) {
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] down");
            startHoldTimer();
        }
        // BUTTON UP
        else {
            if (is_held_) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] up after hold - triggering release");
                is_held_ = false;
                // is held so timer not active
                sendEvent(btn_evts_.release, tap_count_);
            }
            else {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] up - triggering 1-tap");
                stopTimer();
                sendEvent(btn_evts_.tap, 1);
            }
        }
    }

    void handleBtnEventForSwitch(bool down)
    {
        // BUTTON DOWN
        if (down) {
            tap_count_++;   // for menus
            startHoldTimer();
            bool is_terminal_tap = tap_count_ == btn_cfg_.max_taps;

            if (is_terminal_tap) {
                switch (currentMode()) {
                    case BtnMode::tap:
                        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Max tap down - triggering %d-tap", tap_count_);
                        sendEvent(btn_evts_.tap, tap_count_);
                        break;
                    case BtnMode::moment:
                        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Max tap moment hold - triggering %d-hold", tap_count_);
                        sendEvent(btn_evts_.hold, tap_count_);
                        break;
                    case BtnMode::tap_and_hold:
                        break;
                }
            }
        }
        // BUTTON UP
        else {
            stopTimer();  // stop hold timer;

            // This can happen if reset() was called after a down event
            // e.g. other button entered menu during that time.
            if (tap_count_ == 0) {
                return;
            }

            bool is_terminal_tap = tap_count_ == btn_cfg_.max_taps;

            if (releaseIsPending()) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"release after hold - triggering %d-release", tap_count_);
                sendEvent(btn_evts_.release, tap_count_);
                if (is_held_) {
                    // Hold sequence done - can't have new taps
                    tap_count_ = 0;
                    is_held_ = false;
                    return;
                }
                // else this is moment down on terminal tap, but might have more taps comming for menu entry.
            }
            // Standard tap at max_taps threshold for tap_and_hold
            if (is_terminal_tap && currentMode() == BtnMode::tap_and_hold) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Max tap up- triggering %d-tap", tap_count_);
                sendEvent(btn_evts_.tap, tap_count_);
            }
            is_held_ = false;
            startMultitapTimer();
        }
    }

    void resetState()
    {
        stopTimer();
        tap_count_ = 0;
        is_held_ = false;
    }
public:

    /**
     * Notify the state machine that a button event has occurd.
     *
     * @param down true for button down event, false for button up event.
     */
    void processBtnEvent(bool down)
    {
        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Btn %i %s taps/max = %d/%d", btn_idx_, down ? "DOWN" : "UP", tap_count_, btn_cfg_.max_taps);

        if (down == is_down_) {
            LP_LOG_WARN_IF(EN_LOG_BSM, DBG_PREFIX"Duplicate processBtnEvent(%s) call [btn:%u]", down ? "down" : "up", btn_idx_);
            return;
        }

        is_down_ = down;

        // Ignore events if other button is in exclusive mode (i.e. in config menu)
        // but maintain correct is_down_ state.
        if (otherButtonIsInExclusiveMode()) {
            // Exception is if a hold event has already been sent.
            // In this case, still send the release on up.
            if(!down && releaseIsPending()) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"release after hold - triggering %d-release", tap_count_);
                sendEvent(btn_evts_.release, tap_count_);
            }
            else {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Other btn in excl. mode. Ignoring");
            }
            resetState();
            return;
        }

        if (isInMenu()) {
            handleBtnEventForMenu(down);
        }
        else {
            handleBtnEventForSwitch(down);
        }
    }

    /**
     * Must be called when timer set with tparam start_timer expires.
     */
    void processTmrExpiry(bool is_inactivity_timer)
    {
        if (is_inactivity_timer) {
            if (isInMenu()) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Menu inactivity timer expired [btn:%u]", btn_idx_);
                exitMenu();
            }
            return;
        }

        // Ignore events and reset context if other button is in exclusive mode
        // (i.e. in config menu)
        if (otherButtonIsInExclusiveMode()) {
            if (!releaseIsPending()) {
                resetState();
            }
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"processTmrExpiry: Other btn in excl. mode. Ignoring");
            return;
        }

        uint8_t saved_tap_count = tap_count_;

        // --- This is a hold timer expiry

        if (is_down_) {
            is_held_ = true;
            // If taps_count == max_taps, the hold event was already sent for moment mode
            bool is_terminal_tap = tap_count_ == btn_cfg_.max_taps;
            bool hold_already_sent = is_terminal_tap && currentMode() == BtnMode::moment;

            if ((tap_count_ <= btn_cfg_.max_taps && curModeHasHold() && !hold_already_sent) ||
                 isInMenu()) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"hold timer expired - triggering %d-hold", tap_count_);
                sendEvent(btn_evts_.hold, tap_count_);
            }
        }

        // --- This is a multi tap timer expiry

        else {
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Multi-tap timer expired");
            //
            // If already hit max taps, then event
            // was already sent above on down[tap]/up[tap_and_hold] event,
            // otherwise send the tap event here.
            //
            if (tap_count_ < btn_cfg_.max_taps && currentMode() != BtnMode::moment) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"- triggering %d-tap", tap_count_);
                sendEvent(btn_evts_.tap, tap_count_);
            }
            tap_count_ = 0;
        }

        // Check for menu entry
        if (saved_tap_count > btn_cfg_.max_taps) {
            enterMenuIfNeeded(saved_tap_count, is_held_);
            // If we're in a hold, then no further taps are possible, So if we
            // we didn't enter a menu on hold, clear the hold.
            // Otherwise the next UP will start the multi-tap timer and we'll
            // be back here with hold == false, and the hold will have entered
            // a menu that should be entered with a tap
            if (!isInMenu()) {
                resetState();
            }
        }
    }

    /**
     * Exit any active menu.
     *
     * NOOP if not in a menu.
     */
    void exitActiveMenu()
    {
    	if (cur_menu_) {
    		exitMenu();
    	}
    }


private:

    /**
     * @return if a HOLD event has been sent and the corresponding RELEASE not
     * yet sent.
     */
    bool releaseIsPending()
    {
        bool is_terminal_tap = tap_count_ == btn_cfg_.max_taps;
        return (is_held_ && curModeHasHold()) || (is_terminal_tap && currentMode() == BtnMode::moment);
    }

    void enterMenuIfNeeded(uint8_t n_tap, bool hold)
    {
        for (uint8_t i = 0; i < num_menus_; i++) {
            if (menus_[i].hold_needed == hold && menus_[i].taps_to_enter == n_tap) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX" - entering menu %u", i);
                cur_menu_ = &menus_[i].menu;
                // start inactivty timer
                cur_menu_inactive_timeout_ms_ = menus_[i].inactive_expiry_ms;
                if (cur_menu_inactive_timeout_ms_ > 0) {
                    restartMenuInactivityTimer();
                }
                internal::cur_excl_mode_btn_id_ = btn_idx_;
                menus_[i].menu.onMenuEntry(btn_idx_);
                tap_count_ = 0;
                break;
            }
        }
    }

    void exitMenu()
    {
        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX" - exiting menu");
        stopMenuInactivityTimer();
        if (cur_menu_ != nullptr) {
            cur_menu_->onMenuExit(btn_idx_);
        }
        internal::cur_excl_mode_btn_id_ = 0xff;
        cur_menu_ = nullptr;
        cur_menu_inactive_timeout_ms_ = 0;
        is_held_ = false; // clear in case we exited with hold
    }

    void stopMenuInactivityTimer()
    {
        if (cur_menu_ != nullptr && cur_menu_inactive_timeout_ms_ != 0) {
            stop_timer(inactivity_timer_btn_idx);
        }
    }
    void restartMenuInactivityTimer()
    {
        if (cur_menu_ != nullptr && cur_menu_inactive_timeout_ms_ != 0) {
            stop_timer(inactivity_timer_btn_idx);
            start_timer(inactivity_timer_btn_idx, cur_menu_inactive_timeout_ms_);
        }
    }

    BtnMode currentMode() {
        return (tap_count_ == 0 || tap_count_ > BtnInstanceCfg::max_configurable_btn_taps)
            ? BtnMode::tap
            : btn_cfg_.mode[tap_count_-1];
    }
    bool curModeHasHold() {
        return (tap_count_ == 0 || tap_count_ > BtnInstanceCfg::max_configurable_btn_taps)
            ? false
            : btn_cfg_.mode[tap_count_- 1] != BtnMode::tap;
    }
    bool isInMenu() {
         return cur_menu_ != nullptr;
    }
    bool otherButtonIsInExclusiveMode() {
        return internal::cur_excl_mode_btn_id_ != 0xff &&
               internal::cur_excl_mode_btn_id_ != btn_idx_;
    }


    /**
     * Send event to application, or to current menu if active.
     *
     * @param btn_event
     * @param n_tap
     */
    void sendEvent(EventType btn_event, uint8_t n_tap)
    {
        if (cur_menu_ != nullptr) {
            auto [result, menu_evt, n_tap_from_menu] = cur_menu_->processBtnEvent(btn_event);
            switch (result) {
                case BtnMenuEvtResult::none:
                    break;
                case BtnMenuEvtResult::event:
                    on_btn_event(menu_evt, btn_idx_, n_tap_from_menu);
                    break;
                case BtnMenuEvtResult::event_and_exit:
                    on_btn_event(menu_evt, btn_idx_, n_tap_from_menu);
                    [[fallthrough]];
                case BtnMenuEvtResult::exit:
                    exitMenu();
                    break;
            }
        }
        else {
            on_btn_event(btn_event, btn_idx_, n_tap);
        }
    }

private:
    const uint8_t btn_idx_;
    const BtnInstanceCfg& btn_cfg_;
    const BtnCfg& btn_cfg_global_;

    // Take copy to allow for passing temporary on construction
    const BtnEvents<EventType> btn_evts_;

    const BtnMenuEntry<EventType>* menus_;
    const uint8_t num_menus_;

    BtnMenu<EventType>* cur_menu_ = nullptr;
    uint32_t cur_menu_inactive_timeout_ms_ = 0;

    bool is_down_ = false;

    /**
     * Button has been held > min_hold_period and not yet released
     */
    bool is_held_ = false;

    uint8_t tap_count_ = 0;
};

} // namespace libp::btn

#undef DBG_PREFIX

#endif // LIBPEKIN_DEVICES_BTN_STATE_MACHINE_H_
