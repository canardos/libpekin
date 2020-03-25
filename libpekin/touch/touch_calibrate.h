#ifndef SRC_TOUCH_CALIBRATE_H_
#define SRC_TOUCH_CALIBRATE_H_

#include <error_handler.h>

#include "graphics/idrawing_surface.h"
#include "graphics/colors.h"
#include "touch/resistive_touch.h"
#include "libpekin.h"

namespace Libp::ResistiveTouch {

/**
 * Perform a 3-point calibration.
 *
 * @param display
 * @param touchscreen
 *
 * @return true on success
 */
template <typename T>
inline
bool calibrateTouch(Libp::IDrawingSurface<T>& display, ResistiveTouch::Screen& touchscreen) {

    static constexpr uint16_t bg_color = Rgb565Colors::white;
    static constexpr uint8_t target_size = 4;

    const ResistiveTouch::Point* screen_points = touchscreen.getCalibrationPoints();
    ResistiveTouch::CalibrationPoints measured_points;
    display.fillScreen(bg_color);

    ResistiveTouch::Point pos;
    uint8_t i = 0;
    while (i < ResistiveTouch::num_calib_pts) {
        display.fillRect(screen_points[i].x - 2, screen_points[i].y - 2, target_size, target_size, Rgb565Colors::blue);
        // wait for prior release
        while (touchscreen.isTouched())
            ;
        while (!touchscreen.isTouched())
            ;
        if (touchscreen.getTouchPositionRaw(&pos)) {
            // Save touch point and delete prior target
            measured_points[i].x = pos.x;
            measured_points[i].y = pos.y;
            display.fillRect(screen_points[i].x - 2, screen_points[i].y - 2, target_size, target_size, bg_color);
            i++;
            delayMs(150); // avoid double touch
        }
    }
    return touchscreen.updateCalibration(measured_points);
}

} // namespace Libp

#endif /* SRC_TOUCH_CALIBRATE_H_ */
