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

/**
 * Global config data for all buttons
 */
struct BtnCfg {
    static constexpr uint16_t default_max_time_between_taps_ms  = 500;
    static constexpr uint16_t default_min_time_for_hold_ms      = 1100;

    uint16_t max_time_between_taps_ms= default_max_time_between_taps_ms;
    uint16_t min_time_for_hold_ms = default_min_time_for_hold_ms;
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

    /**
     * True means momentary action for this number of taps.
     *
     * - Regular action sends a tap event on up
     * - Momentary action sends a hold event on down and release event on up.
     */
    bool is_moment[max_configurable_btn_taps] = {}; // [0,0,0,...]
};

/**
 *
 * No multi-tap in menus.
 */
template<typename EventType>
class BtnMenu {
public:
    enum class MenuEvtResult : uint8_t { none, event, exit, event_and_exit };
    virtual ~BtnMenu() = default;
    /**
     * Called by the button state manager when the menu is activated.
     */
    virtual void onMenuEntry() = 0;
    /**
     * The button state manager passes button tap, hold, and release events
     * to the menu here.
     *
     * Only tap, hold, release will be passed.
     */
    virtual std::tuple<MenuEvtResult, EventType> processBtnEvent(EventType evt) = 0;
};


template<typename EventType>
struct BtnMenuEntry {
    BtnMenu<EventType>& menu;
    uint8_t n_taps_to_activate;
    /** Menu is activated by holding on the nth tap */
    bool hold_needed;
};

template<typename EventType>
struct BtnEvents {
    EventType tap;
    EventType hold;
    EventType release;
};

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
 * Button up/down events must be reported to this class via the
 * processBtnEvent method and the class will dispatch multi-tap/hold etc.
 * events via the on_btn_event function template param.
 *
 * This class also hosts button menus and will dispatch various events generated
 * by them also.
 *
 * NOTE:
 * - on_btn_event, start_timer and stop_timer should be shared across instances
 *   to prevent multiple template instantiations.
 * - processBtnEvent / processTmrExpiry should be called from the same context
 *   as there is no concurrency protection.
 *
 * @tparam on_btn_event Callback to receive tap/hold/menu etc. events.
 *                      Sig: `on_btn_event(BtnEvent, uint8_t btn_idx, uint8_t num_taps)`
 * @tparam start_timer  fn to start a one-shot timer of the specified duration.
 *                      At expiration, `processTmrExpiry` must be called.
 *                      Sig: `start_timer(uint8_t btn_idx, uint32_t delay_ms)`
 * @tparam stop_timer   fn to cancel any outstanding timer started by a prior
 *                      call to start_timer.
 *                      Sig: `stop_timer (uint8_t btn_idx)`
 */
template<
    typename EventType,
    auto on_btn_event, // on_btn_event(BtnEvent, uint8_t btn_idx, uint8_t num_taps)
    auto start_timer,  // start_timer(uint8_t btn_idx, uint32_t delay_ms)
    auto stop_timer>   // stop_timer (uint8_t btn_idx)
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
     * @param menus no copy taken, must remain allocated
     */
    constexpr BtnStateMachine(
            uint8_t btn_idx,
            const BtnInstanceCfg& btn_cfg,
            const BtnCfg& btn_cfg_global,
            const BtnEvents<EventType>& btn_evts,
            const BtnMenuEntry<EventType>* menus, uint8_t num_menus)
                : btn_idx_(btn_idx),
                  btn_cfg_(btn_cfg),
                  btn_cfg_global_(btn_cfg_global),
                  btn_evts_(btn_evts),
                  menus_(menus), num_menus_(num_menus)
    { }

    // Prevent accidental copy/move
    BtnStateMachine(const BtnStateMachine&) = delete;
    BtnStateMachine& operator=(const BtnStateMachine&) = delete;
    BtnStateMachine(BtnStateMachine&&) = delete;
    BtnStateMachine& operator=(BtnStateMachine&&) = delete;

private:

    void startHoldTimer()
    {
        stop_timer(btn_idx_);
        start_timer(btn_idx_, btn_cfg_global_.min_time_for_hold_ms);
    }
    void startMultitapTimer()
    {
        stop_timer(btn_idx_);
        start_timer(btn_idx_, btn_cfg_global_.max_time_between_taps_ms);
    }


    // Menus only handle tap, hold, release for single tap
    void handleBtnEventForMenu(bool down)
    {
        // BUTTON DOWN
        if (down) {
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] down - starting hold timer");
            startHoldTimer();
        }
        // BUTTON UP
        else {
            if (is_held_) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] up after hold - triggering release");
                // is held so timer not active
                is_held_ = false;
                sendEvent(btn_evts_.release, tap_count_);
            }
            else {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"[Menu] up - triggering 1-tap");
                stop_timer(btn_idx_);
                sendEvent(btn_evts_.tap, 1);
            }
        }
    }

    void handleBtnEventForSwitch(bool down)
    {
        // -------------------------------------------------------------------
        // 1 | Down when new tap_count == max allowed taps
        // 1 | Don't need to wait for release or timed hold - send event immediately
        // -------------------------------------------------------------------

#ifdef DO_RAPID_TAP // i.e. switch on / off during multi-tap to menu etc.
        if ( down && ((tap_count_ == (btn_cfg_.max_taps - 1)) || (btn_cfg_.max_taps == 1)) ) {
#else
        if ( down && tap_count_ == (btn_cfg_.max_taps - 1) ) {
#endif
            tap_count_++;
            // stop mult-tap timer
            stop_timer(btn_idx_);

            if (btn_cfg_.is_moment[tap_count_ - 1]) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Max tap moment hold - triggering %d-hold\r\n", tap_count_);
                is_max_tap_held_ = true;
                sendEvent(btn_evts_.hold, tap_count_);

                // The user may still keep tapping for a system menu.
                // We need to time the down duration to confirm it's actually
                // a hold before we reset the tap count on release.
                startHoldTimer();
            }
            else {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Max tap down - triggering %d-tap\r\n", tap_count_);
#ifdef DO_RAPID_TAP
                sendEvent(btn_evts_.tap, btn_cfg_.max_taps == 1 ? 1 : tap_count_);
#else
                sendEvent(btn_evts_.tap, tap_count_);
#endif
            }
        }
        // -------------------------------------------------------------------
        // 2 | Still possible there could be more taps.
        // 2 | Could be part of a multi-tap so need to wait
        // -------------------------------------------------------------------
        else {
            if (down) {
                tap_count_++;
                // Stop multi-tap timer
                stop_timer(btn_idx_);
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Down (%d taps)\r\n", tap_count_);

                // start the hold timer for momentary OR this many taps enters a menu
                if ( (tap_count_ <= BtnInstanceCfg::max_configurable_btn_taps && btn_cfg_.is_moment[tap_count_ - 1])
                   || tapCountEntersMenu(tap_count_)) {
                    startHoldTimer();
                }
            }
            else {
                if (is_held_) {
                    // don't send event on hold release for tap count > max
                    if (tap_count_ <= btn_cfg_.max_taps) {
                        sendEvent(btn_evts_.release, tap_count_);
                    }
                    tap_count_ = 0;
                    is_held_ = false;
                    is_max_tap_held_ = false;
                }
                else {
                    if (is_max_tap_held_) {
                        sendEvent(btn_evts_.release, tap_count_);
                        is_max_tap_held_ = false;
                    }
                    // Start multi tap timer
                    LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Up (%d taps) - start multi-tap timer\r\n", tap_count_);
                    startMultitapTimer();
                }
            }
        }
    }

    /**
     * @return true if a menu is present that needs this many taps to enter
     */
    bool tapCountEntersMenu(uint8_t tap_count)
    {
        for (uint8_t i = 0; i < num_menus_; i++) {
            if (menus_[i].n_taps_to_activate == tap_count) {
                return true;
            }
        }
        return false;
    }

    void resetState()
    {
        tap_count_ = 0;
        is_held_ = false;
        is_max_tap_held_ = false;
        // Don't reset down state or we could lose consistency.
        //is_down_ = false;
    }
public:

    /**
     *
     */
    void processBtnEvent(bool down)
    {
        LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Btn %i %s taps/max = %d/%d\r\n", btn_idx_, down ? "DOWN" : "UP", tap_count_, btn_cfg_.max_taps);

        if (down == is_down_) {
            LP_LOG_WARN_IF(EN_LOG_BSM, DBG_PREFIX"Duplicate processBtnEvent(%s) call for btn %u", down ? "down" : "up", btn_idx_);
            return;
        }

        is_down_ = down;

        // Ignore events if other button is in exclusive mode (i.e. in config menu)
        if (otherButtonIsInExclusiveMode()) {
            resetState();
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Other btn in excl. mode. Ignoring");
            return;
        }

        if (inMenu()) {
            handleBtnEventForMenu(down);
        }
        else {
            handleBtnEventForSwitch(down);
        }
    }

    void processTmrExpiry()
    {
        // Ignore events and reset context if other button is in exclusive mode
        // (i.e. in config menu)
        if (otherButtonIsInExclusiveMode()) {
            resetState();
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"processTmrExpiry: Other btn in excl. mode. Ignoring");
            return;
        }

        uint8_t saved_tap_count = tap_count_;
        // --- This is a hold
        if (is_down_) {
            // If taps_count == max_taps, the hold event was already sent
            if (tap_count_ < btn_cfg_.max_taps || inMenu()) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"hold timer expired - triggering %d-hold\r\n", tap_count_);
                sendEvent(btn_evts_.hold, tap_count_);
            }
            is_held_ = true;
        }

        // --- This is expiry of a multi tap timer
        else {
            LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX"Multi-tap timer expired");
            //
            // If already hit max taps, then event
            // was already sent above on down event,
            // otherwise send the event here.
            //
            if (tap_count_ < btn_cfg_.max_taps && !btn_cfg_.is_moment[tap_count_ - 1]) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX" - triggering %d-tap\r\n", tap_count_);
                sendEvent(btn_evts_.tap, tap_count_);
            }
            tap_count_ = 0;
        }

        // Check for menu entry
        if (saved_tap_count > btn_cfg_.max_taps) {
            enterMenuIfNeeded(saved_tap_count, is_held_);
        }
    }

private:

    void enterMenuIfNeeded(uint8_t n_tap, bool hold)
    {
        for (uint8_t i = 0; i < num_menus_; i++) {
            if (menus_[i].hold_needed == hold && menus_[i].n_taps_to_activate == n_tap) {
                LP_LOG_DEBUG_IF(EN_LOG_BSM, DBG_PREFIX" - entering menu %u\r\n", i);
                cur_menu_ = &menus_[i].menu;
                internal::cur_excl_mode_btn_id_ = btn_idx_;
                menus_[i].menu.onMenuEntry();
                tap_count_ = 0;
                break;
            }
        }
    }

    void exitMenu(bool show_confirm_led = false)
    {
        internal::cur_excl_mode_btn_id_ = 0xff;
        cur_menu_ = nullptr;
        //stopMenuInactivityTimer();
    }

    /*void stopMenuInactivityTimer()
    {
        //g_menu_timer.clear();
    }
    void restartMenuInactivityTimer()
    {
        // TODO: could use static exitMenus function from button file
        //g_menu_timer.set([this](){ exitMenu(false); }, cfg::menu_inactivity_timeout_ms);
    }*/

    inline bool inMenu()                       { return cur_menu_ != nullptr; }
    inline bool otherButtonIsInExclusiveMode() { return internal::cur_excl_mode_btn_id_ != 0xff && internal::cur_excl_mode_btn_id_ != btn_idx_; }


    /**
     * Send event to application, or to current menu if active.
     *
     * @param btn_event
     * @param n_tap
     */
    void sendEvent(EventType btn_event, uint8_t n_tap)
    {
        if (cur_menu_ != nullptr) {
            auto [result, menu_evt] = cur_menu_->processBtnEvent(btn_event);
            switch (result) {
                case BtnMenu<EventType>::MenuEvtResult::none:
                    break;
                case BtnMenu<EventType>::MenuEvtResult::event:
                    on_btn_event(menu_evt, btn_idx_, 0);
                    break;
                case BtnMenu<EventType>::MenuEvtResult::event_and_exit:
                    on_btn_event(menu_evt, btn_idx_, 0);
                    [[fallthrough]];
                case BtnMenu<EventType>::MenuEvtResult::exit:
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

    const BtnEvents<EventType>& btn_evts_;

    const BtnMenuEntry<EventType>* menus_;
    const uint8_t num_menus_;

    BtnMenu<EventType>* cur_menu_ = nullptr;

    bool is_down_ = false;
    /**
     * Button has been held > min_hold_period
     */
    bool is_held_ = false;
    /**
     * Button is momentary && has been pressed && tap_count_ == max-taps
     */
    bool is_max_tap_held_ = false;

    uint8_t tap_count_ = 0;
};

} // namespace libp::btn

#undef DBG_PREFIX

#endif // LIBPEKIN_DEVICES_BTN_STATE_MACHINE_H_
