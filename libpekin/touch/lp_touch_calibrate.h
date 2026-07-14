#ifndef SRC_TOUCH_CALIBRATE_H_
#define SRC_TOUCH_CALIBRATE_H_

#include "graphics/lp_colors.h"
#include "graphics/lp_idrawing_surface.h"
#include "touch/lp_resistive_touch.h"

#include "libpekin.h"

namespace libp::resist_touch {

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
bool calibrateTouch(libp::IDrawingSurface<T>& display, resist_touch::Screen& touchscreen) {

    static constexpr uint16_t bg_color = rgb565_clr::white;
    static constexpr uint8_t target_size = 4;

    const resist_touch::Point* screen_points = touchscreen.getCalibrationPoints();
    resist_touch::CalibrationPoints measured_points;
    display.fillScreen(bg_color);

    resist_touch::Point pos;
    uint8_t i = 0;
    while (i < resist_touch::num_calib_pts) {
        display.fillRect(screen_points[i].x - 2, screen_points[i].y - 2, target_size, target_size, rgb565_clr::blue);
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

} // namespace libp

#endif /* SRC_TOUCH_CALIBRATE_H_ */
