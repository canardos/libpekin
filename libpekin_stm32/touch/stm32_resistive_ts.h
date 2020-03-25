#ifndef LIB_LIBPEKIN_STM32_TOUCH_RESISTIVE_TOUCH_STM32_H_
#define LIB_LIBPEKIN_STM32_TOUCH_RESISTIVE_TOUCH_STM32_H_

#include <cstdint>

#include "../adc_stm32f1xx.h"
#include "../pins_stm32f1xx.h"
#include "libpekin_stm32_hal.h"
#include "touch/resistive_touch.h"

namespace LibpStm32 {

// TODO: pass pins as Pin template params

// *** UPDATE AS REQUIRED ****
static constexpr uint32_t adc_base_addr = ADC1_BASE;
static constexpr uint32_t pin_port_addr = GPIOA_BASE;
static constexpr uint8_t pin_xneg_no = 3;
static constexpr uint8_t pin_xpos_no = 6;
static constexpr uint8_t pin_yneg_no = 5;
static constexpr uint8_t pin_ypos_no = 7;
static constexpr Adc::Channel  xpos_adc_ch = Adc::Channel::ch6;
static constexpr Adc::Channel  ypos_adc_ch = Adc::Channel::ch7;
// ***************************

/**
 * `ITouchScreen` implementation for 4-pin resistive touchscreen on stm32f1xx
 * platform.
 */
class ResistiveTs : public Libp::ResistiveTouch::ITouchScreen {

public:
    /**
     * It is assumed that the appropriate GPIO and ADC clocks have been enabled
     * and that the ADC instance has been configured.
     *
     * ADC channel setup will be performed by this class.
     */
    ResistiveTs() : interrupt_mode_(false) { }

    /**
     * Interrupt will be trigger on yneg_pin_
     *
     * Ensure handler is implemented and clears pending reg to prevent hanging.
     */
    bool start(bool use_interrupt);
    void stop();
    bool isTouched();
    bool readPos(Libp::ResistiveTouch::Point* p);

private:
    enum class State : uint8_t {
        touch_detect, read_x, read_y, uninitialized
    };

    static const Adc::AdcDevice<adc_base_addr> adc_;
    static const LibpStm32::Pin<pin_port_addr, pin_xneg_no> xneg_pin_;
    static const LibpStm32::Pin<pin_port_addr, pin_xpos_no> xpos_pin_;
    static const LibpStm32::Pin<pin_port_addr, pin_yneg_no> yneg_pin_;
    static const LibpStm32::Pin<pin_port_addr, pin_ypos_no> ypos_pin_;

    // Must be low enough to avoid uint32_t overflow (SAMPLES * ADC max val)
    static constexpr uint8_t SAMPLES = 5;

    State state_ = State::uninitialized;
    //ADC_HandleTypeDef* adc_handle_; // can't be const

    bool interrupt_mode_;

    void configurePinsReadX();
    void configurePinsReadY();
    void configurePinsTouchDetect();
};

} // namespace LibpStm32

#endif /* LIB_LIBPEKIN_STM32_TOUCH_RESISTIVE_TOUCH_STM32_H_ */
