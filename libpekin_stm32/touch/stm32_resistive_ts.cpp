#include "stm32_resistive_ts.h"
#include "adc_stm32f1xx.h"

using namespace Libp;
using namespace LibpStm32;

bool ResistiveTs::start(bool use_interrupt)
{
    interrupt_mode_ = use_interrupt;
    if (!interrupt_mode_)
        // May have previously been running in interrupt mode
        yneg_pin_.disableIrq();
    if (state_ != State::touch_detect)
        configurePinsTouchDetect();
    return true;
}

void ResistiveTs::stop()
{
    if (interrupt_mode_) {
        yneg_pin_.disableIrq();
        yneg_pin_.clearAsInputIrq();
    }
}

bool ResistiveTs::isTouched()
{
    if (state_ != State::touch_detect)
        configurePinsTouchDetect();
    return !yneg_pin_.read();
}

bool ResistiveTs::readPos(Libp::ResistiveTouch::Point* p)
{
    configurePinsReadX();
    uint32_t x = 0;
    for (uint8_t i = 0; i < SAMPLES; i++)
        x += Adc::AdcDevice<ADC1_BASE>::poll();
    p->x = x / SAMPLES;

    configurePinsReadY();
    uint32_t y = 0;
    for (uint8_t i = 0; i < SAMPLES; i++)
        y += Adc::AdcDevice<ADC1_BASE>::poll();
    p->y = y / SAMPLES;

    return isTouched();
}

void ResistiveTs::configurePinsReadX()
{
    // disable touch detection
    if (interrupt_mode_)
        yneg_pin_.disableIrq();

    xneg_pin_.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
    xpos_pin_.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
    yneg_pin_.setAsInput(InputMode::floating);
    adc_.setup(ypos_adc_ch, Adc::SampleTime::cycles_1pt5);
    ypos_pin_.setAsInput(InputMode::analog);

    xneg_pin_.set();
    xpos_pin_.clear();

    state_ = State::read_x;
    delayUs(100); // Wait for Ypos/Yneg to settle.
                  // Maybe not required, but little downside
}

void ResistiveTs::configurePinsReadY()
{
    // disable touch detection
    if (interrupt_mode_)
        // TODO: maybe not enabled - how expensive is this?
        yneg_pin_.disableIrq();

    yneg_pin_.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
    ypos_pin_.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
    xneg_pin_.setAsInput(InputMode::floating);
    adc_.setup(xpos_adc_ch, Adc::SampleTime::cycles_1pt5);
    xpos_pin_.setAsInput(InputMode::analog);

    ypos_pin_.set();
    yneg_pin_.clear();

    state_ = State::read_y;
    delayUs(100); // Wait for Ypos/Yneg to settle.
                  // Maybe not required, but little downside
}

void ResistiveTs::configurePinsTouchDetect()
{
    xpos_pin_.setAsOutput(OutputMode::pushpull, OutputSpeed::low);
    xneg_pin_.setAsInput(InputMode::floating);
    ypos_pin_.setAsInput(InputMode::floating);

    if (interrupt_mode_) {
        // TODO: do we care about priority?
        yneg_pin_.setAsInputIrq(InputMode::pullup, false, true);
        yneg_pin_.enableIrq();
    }
    else {
        yneg_pin_.setAsInput(InputMode::pullup);
    }
    state_= State::touch_detect;
}
