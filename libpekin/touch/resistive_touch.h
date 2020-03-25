/**
 * Class to manage calibration of resistive touch screens.
 *
 * Calibration functions adapted from sample by Carlos E. Vidales:
 *
 * Source: https://www.embedded.com/design/system-integration/4023968/How-To-Calibrate-Touch-Screens
 * Author: Carlos E. Vidales (CVidales.ee77@gtalumni.org)
 * Date: 2002
 * Original Copyright Notice:
 *
 * Copyright (c) 2001, Carlos E. Vidales. All rights reserved.
 *
 * This sample program was written and put in the public domain
 * by Carlos E. Vidales.  The program is provided "as is"
 * without warranty of any kind, either expressed or implied.
 *
 * If you choose to use the program within your own products
 * you do so at your own risk, and assume the responsibility
 * for servicing, repairing or correcting the program should
 * it prove defective in any manner.
 *
 * You may copy and distribute the program's source code in any
 * medium, provided that you also include in each copy an
 * appropriate copyright notice and disclaimer of warranty.
 *
 * You may also modify this program and distribute copies of
 * it provided that you include prominent notices stating
 * that you changed the file(s) and the date of any change,
 * and that you do not charge any royalties or licenses for
 * its use.
 */
#ifndef SRC_RESISTIVE_TOUCH_H_
#define SRC_RESISTIVE_TOUCH_H_

#include <cstdint>
#include "i_touch_screen.h"

namespace Libp::ResistiveTouch {

struct CalibrationMatrix {
    // int32 is adequate for 10-bit ADC
    int32_t An; /* A = An/Divider */
    int32_t Bn; /* B = Bn/Divider */
    int32_t Cn; /* C = Cn/Divider */
    int32_t Dn; /* D = Dn/Divider */
    int32_t En; /* E = En/Divider */
    int32_t Fn; /* F = Fn/Divider */
    int32_t Divider;
    bool valid() { return Divider != 0; }
};

static constexpr uint8_t num_calib_pts = 3;

using CalibrationPoints = Point[num_calib_pts];

/**
 * ITouchScreen wrapper to provide calibration for resistive touch screens.
 *
 * `begin` must be called prior to using an instance of this class.
 *
 * Basic calibration is performed on construction assuming an ideal panel and
 * ADC. Further calibration will likely be required to achieve satisfactory
 * results.
 *
 * The typical calibration procedure is:
 * 1) Get the 3 ideal test points via `getCalibrationPoints`.
 * 2) Have the user click on these 3 points and record the touch points via
 *    @p getTouchPositionRaw.
 * 3) Call @p updateCalibration with the detected points.
 *
 * Once calibration has been completed, `getTouchPosition` should return touch
 * positions in accurate screen coordinates (i.e. based on the x/y resolution
 * provided on construction).
 */
class Screen {

public:
    /// Max supported resolution (higher will overflow)
    /// Resolutions above this will be downsampled automatically
    static constexpr uint8_t max_adc_res = 10;

    /**
     * @param ts interface to the touch screen hardware
     * @param x_res the screen x resolution (i.e. width)
     * @param y_res the screen y resolution (i.e. height)
     */
    Screen(ITouchScreen& ts, uint16_t x_res, uint16_t y_res);

    /**
     * Initializes the GPIO for touch detection.
     *
     * This function must be called prior to using an instance of the class.
     *
     * May be called multiple times if change of interrupt mode is required.
     *
     * If interrupt mode is specified, the `Pins::yneg` pin is used and so an
     * appropriate handler must be implemented. A typical implementation might
     * look like this:
     *
     * ````
     * volatile bool touched = false;
     *
     * main_program_loop() {
     *     Point pos;
     *     ...
     *     while(true) {
     *         ...
     *         if (touched) {
     *             if (touchscreen.getTouchPosition(&pos)) {
     *             // handle touch
     *             }
     *             ...
     *             touched = false;
     *         }
     *         ...
     *     }
     * }
     *
     * extern "C" void EXTIXXX_IRQHandler(void) {
     *     touched = true;
     *     __HAL_GPIO_EXTI_CLEAR_IT(YNEG_PIN);
     * }
     * ````
     *
     * If significant time may pass between the touch and handling the
     * detection in the main loop, recording the touch position in the IRQ
     * handler may provide a more responsive experience.
     *
     * @param use_interrupt true to trigger ext interrupts on touch detection,
     *                      false to poll `isTouched` instead.
     */
    //TODO: handle interrupt callback
    bool begin(bool use_interrupt);

    /**
     * Stops touch detection.
     *
     * If interrupt mode was not specified in the call to @p begin then this
     * function does nothing.
     */
    void end();

    /**
     * Performs a touch detection and returns the result.
     *
     * @return true if a touch is currently detected, false otherwise
     */
    bool isTouched();

    /**
     * Performs touch detection and returns the currently touched position in
     * raw (ADC) unadjusted coordinates. No calibration is performed.
     *
     * This function is typically only used during the calibration process.
     *
     * Note that the resolution of the coordinates will be adjusted downwards
     * to `TouchScreen::max_adc_res` if the ADC resolution provided on
     * construction exceeds `TouchScreen::max_adc_res`.
     *
     * @param[out] pos point to receive the detected coordinates
     *
     * @return true if the detection was successful, false if no touch was
     *         detected.
     */
    bool getTouchPositionRaw(Point* pos);

    /**
     * Performs touch detection and returns the currently touched position in
     * screen coordinates including any required calibration adjustments.
     *
     * @param[out] pos point to receive the detected coordinates
     *
     * @return true if the detection was successful, false if no touch was
     *         detected.
     */
    bool getTouchPosition(Point* pos);

    /**
     * Returns a reference to an array of 3 ideal test points (in screen
     * coordinates) to be used for calibration. The reference is valid for the
     * life of this object.
     *
     * The typical calibration procedure is:
     * 1) Get the 3 ideal test points via @p getCalibrationPoints.
     * 2) Have the user click on these 3 points and record the touch points via
     *    @p getTouchPositionRaw.
     * 3) Call @p updateCalibration with the detected points.
     *
     * @return pointer Points in screen coordinates
     */
    const CalibrationPoints& getCalibrationPoints();

    /**
     * Update the calibration parameters used to map between detected points
     * and screen coordinates.
     *
     * This function assumes the 3 points returned by @p getCalibrationPoints
     * have been used to acquire the detected points.
     *
     * The typical calibration procedure is:
     * 1) Get the 3 ideal test points via @p getCalibrationPoints.
     * 2) Have the user click on these 3 points and record the touch points via
     *    @p getTouchPositionRaw.
     * 3) Call @p updateCalibration with the detected points.
     *
     * @param detected_points points reported via @p getTouchPositionRaw
     * @return true if calibration was successful, false otherwise. Calibration
     *         will only fail if there was no space between the detected
     *         points.
     */
    bool updateCalibration(const CalibrationPoints& detect_points);

    /**
     * Update the calibration matrix directly.
     *
     * @param matrix
     */
    void updateCalibration(const CalibrationMatrix& matrix);

    /**
     * Return current calibration parameters.
     *
     * @return
     */
    const CalibrationMatrix& getCalibration();

private:
    ITouchScreen& ts_;

    /// Ideal calibration points in screen coordinates
    const CalibrationPoints screen_calib_pnts_;

    /// Calibration matrix in use
    CalibrationMatrix calibration_mtx_;

    // TODO: should this be exposed?
    /// Update calibration matrix using point test
    bool updateCalibration(const CalibrationPoints& expect_points, const CalibrationPoints& detect_points);

    /// Convert point from digitizer location to screen location using current
    /// calibration matrix.
    bool calibratePoint(Point* p);


};

} // namespace Libp::ResistiveTouch

#endif /* SRC_RESISTIVE_TOUCH_H_ */
