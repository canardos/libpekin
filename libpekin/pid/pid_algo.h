#ifndef LIB_LIBPEKIN_PID_PID_ALGO_H_
#define LIB_LIBPEKIN_PID_PID_ALGO_H_

#include <error_handler.h>
#include <cstdint>
#include "misc_math.h"

namespace Libp {

/**
 * Very basic PID implementation. Customized for use in a specific reflow
 * oven application.
 */
class PidAlgo {
public:
	using GetMillisFunc = uint64_t (*)();
	enum class Mode : uint8_t {
		normal, gradient
	};

    PidAlgo(float kp, float ki, float kd, float min_output, float max_output, GetMillisFunc get_millis)
            : kp_(kp), ki_(ki), kd_(kd), min_(min_output), max_(max_output), get_millis_(get_millis) { }

    /**
     * Initialize the algorithm.
     *
     * Must be called prior to calling \p compute.
     */
    void init(float measured_state, uint16_t sampling_period_ms, Mode mode)
    {
    	mode_ = mode;
        sampling_period_ms_ = sampling_period_ms;
        integral_ = 0;
        prev_process_state_ = measured_state;
        getErrHndlr().report("PID=%d,%d,%d\r\n",
                (int)kp_, (int)ki_, (int)kd_);
    }

    /**
     *
     * @param desired_state Target process state. In gradient mode, this is the
     *        desired gradient of the measured state variable.
     * @param measured_state Measured process state (e.g. temperature). This
     *                       should not differ depending on mode. The algo will
     *                       calculate the measured gradient in gradient mode.
     * @param output_var Pointer to variable to receive the new output level
     *
     * @return true if sufficient time has elapsed and a new output level was
     *         computed.
     */
    bool compute(float desired_state, float measured_state, float* output_var) {

    	uint64_t now = get_millis_();
    	uint32_t time_elapsed_ms = (now - last_instant_);
        if (time_elapsed_ms < sampling_period_ms_)
            return false;

        float test_state = mode_ == Mode::gradient
        		? (measured_state - prev_process_state_) / time_elapsed_ms
				: measured_state;

        float error = desired_state - test_state;

        //uint16_t integral_max_sum = measured_state / 30;
        // TODO: constants
        constexpr uint16_t integral_max_sum = 25;
        integral_ += ki_ * (error * time_elapsed_ms / 1000);
        integral_ = constrain(integral_, 0, integral_max_sum * output_scale);
        // TODO: why no negative integral?

        float derivative = (error - previous_error_) / time_elapsed_ms * 1000;

        *output_var = constrain(
                (kp_ * error + integral_ + kd_ * derivative) / output_scale,
                min_, max_ );

        // TODO: Need to remove this
        getErrHndlr().report("tgt=%d, act=%d, err=%d (P=%d, I=%d, D=%d) out=%d(%d)\r\n",
                (int)desired_state, (int)measured_state, (int)error,
				(int)(kp_ * error), (int)integral_, (int)(kd_ * derivative),
				(int)*output_var, (int)(kp_ * error + integral_ + kd_ * derivative));

        prev_process_state_ = measured_state;
        previous_error_ = error;

        return true;
    }

    void setOutputLimits(float min, float max)
    {
        min_ = min;
        max_ = max;
    }

    void setPidParams(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    // TODO: need inline?
    inline static constexpr uint16_t default_sampling_period = 1000;
    inline static constexpr uint16_t output_scale = 1;

    uint32_t sampling_period_ms_ = default_sampling_period;

    float previous_error_ = 0;
    float prev_process_state_ = 0;
    float integral_ = 0;

    Mode mode_ = Mode::normal;

    float kp_;
    float ki_;
    float kd_;

    float min_;
    float max_;

    GetMillisFunc get_millis_;
    uint64_t last_instant_ = 0;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_PID_PID_ALGO_H_ */
