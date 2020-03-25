#include "touch/resistive_touch.h"
#include "libpekin.h"

namespace Libp {

using namespace ResistiveTouch;

Screen::Screen(ITouchScreen& ts, uint16_t x_res, uint16_t y_res) :
        ts_(ts),
        screen_calib_pnts_{
            { static_cast<uint16_t>(15u * x_res / 100u), static_cast<uint16_t>(15u * y_res / 100u) },
            { static_cast<uint16_t>(50u * x_res / 100u), static_cast<uint16_t>(85u * y_res / 100u) },
            { static_cast<uint16_t>(85u * x_res / 100u), static_cast<uint16_t>(55u * y_res / 100u) }
        }
{ }

const CalibrationPoints& Screen::getCalibrationPoints()
{
    return screen_calib_pnts_;
}

bool Screen::isTouched()
{
    return ts_.isTouched();
}

bool Screen::getTouchPositionRaw(Point* pos)
{
    if (!isTouched())
        return false;
    ts_.readPos(pos);
    return isTouched();
}

bool Screen::getTouchPosition(Point* pos)
{
    if (!getTouchPositionRaw(pos))
        return false;
    calibratePoint(pos);
    return true;
}

bool Screen::updateCalibration(const CalibrationPoints& detect_points)
{
    return updateCalibration(screen_calib_pnts_, detect_points);
}

bool Screen::updateCalibration(const CalibrationPoints& expect_points, const CalibrationPoints& detect_points)
{
    calibration_mtx_.Divider = ((detect_points[0].x - detect_points[2].x) * (detect_points[1].y - detect_points[2].y)) -
               ((detect_points[1].x - detect_points[2].x) * (detect_points[0].y - detect_points[2].y)) ;

    if( calibration_mtx_.Divider == 0 ) {
        return false;
    }
    calibration_mtx_.An = ((expect_points[0].x - expect_points[2].x) * (detect_points[1].y - detect_points[2].y)) -
              ((expect_points[1].x - expect_points[2].x) * (detect_points[0].y - detect_points[2].y)) ;

    calibration_mtx_.Bn = ((detect_points[0].x - detect_points[2].x) * (expect_points[1].x - expect_points[2].x)) -
              ((expect_points[0].x - expect_points[2].x) * (detect_points[1].x - detect_points[2].x)) ;

    calibration_mtx_.Cn = (detect_points[2].x * expect_points[1].x - detect_points[1].x * expect_points[2].x) * detect_points[0].y +
              (detect_points[0].x * expect_points[2].x - detect_points[2].x * expect_points[0].x) * detect_points[1].y +
              (detect_points[1].x * expect_points[0].x - detect_points[0].x * expect_points[1].x) * detect_points[2].y ;

    calibration_mtx_.Dn = ((expect_points[0].y - expect_points[2].y) * (detect_points[1].y - detect_points[2].y)) -
              ((expect_points[1].y - expect_points[2].y) * (detect_points[0].y - detect_points[2].y)) ;

    calibration_mtx_.En = ((detect_points[0].x - detect_points[2].x) * (expect_points[1].y - expect_points[2].y)) -
              ((expect_points[0].y - expect_points[2].y) * (detect_points[1].x - detect_points[2].x)) ;

    calibration_mtx_.Fn = (detect_points[2].x * expect_points[1].y - detect_points[1].x * expect_points[2].y) * detect_points[0].y +
              (detect_points[0].x * expect_points[2].y - detect_points[2].x * expect_points[0].y) * detect_points[1].y +
              (detect_points[1].x * expect_points[0].y - detect_points[0].x * expect_points[1].y) * detect_points[2].y ;

    return true;
}

void Screen::updateCalibration(const CalibrationMatrix& matrix)
{
    calibration_mtx_ = matrix;
}

const CalibrationMatrix& Screen::getCalibration()
{
    return calibration_mtx_;
}

bool Screen::calibratePoint(Point* p)
{
    if(!calibration_mtx_.valid())
        return false;

    // int32 is adequate for 10-bit ADC
    int32_t x = p->x;
    int32_t y = p->y;

    p->x = ( calibration_mtx_.An * x + calibration_mtx_.Bn * y + calibration_mtx_.Cn )
            / calibration_mtx_.Divider ;

    p->y = ( calibration_mtx_.Dn * x + calibration_mtx_.En * y + calibration_mtx_.Fn )
            / calibration_mtx_.Divider ;

    return true;
}

} // namespace Libp
