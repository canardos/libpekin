#ifndef GRAPHICS_GRAPHICS_H_
#define GRAPHICS_GRAPHICS_H_

//#include <type_traits>

namespace Libp {

/// Bits per pixel.
///
/// Pixel order:
/// \code
///          |===============|===============|
/// bytes    |    byte 0     |    byte 1     |
///          |---------------|---------------|
/// bits     |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
///          |===============|===============|
/// msb_1bpp |0 1 3 4 5 6 7 8|9...           |
///          |---------------|---------------|
/// msb_2bpp | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
///          |---------------|---------------|
/// msb_3bpp |  0  |  1  | 2   |  3  |  4  |..
///          |---------------|---------------|
/// msb_4bpp |   0   |   1   |    2  |   3   |
///          |---------------|---------------|
///    _8bpp |       0       |       1       |
///          |===============|===============|
/// \endcode
enum class Bpp : uint8_t {
    msb_1bpp,
    msb_2bpp,
    msb_3bpp,
    msb_4bpp,
    msb_6bpp,
    _8bpp,
};

/// The position on an objects bounding box to align to the destination
/// coordinate.
///
/// \code
/// e.g. * = destination coordinate:
///
/// middle_right
/// +------+
/// |      |
/// |      *
/// |      |
/// +------+
///
/// top_left
/// *------+
/// |      |
/// |      |
/// |      |
/// +------+
/// \endcode
enum class Align : uint8_t {
    top_left      = 0x00,///< top_left
    top_center    = 0x01,///< top_center
    top_right     = 0x02,///< top_right
    middle_left   = 0x10,///< middle_left
    middle_center = 0x11,///< middle_center
    middle_right  = 0x12,///< middle_right
    bottom_left   = 0x20,///< bottom_left
    bottom_center = 0x21,///< bottom_center
    bottom_right  = 0x22 ///< bottom_right
};

/// Data for a 2D image.
struct Image2d {
    const uint16_t width;
    const uint16_t height;
    const Bpp bpp;
    const uint8_t* data;
};

/// 2d point (unsigned)
struct Point2dU {
    uint16_t x;
    uint16_t y;
};

/// 2d point (signed)
struct Point2dS {
    int16_t x;
    int16_t y;
};

struct Rect16 {
    uint16_t width;
    uint16_t height;
};

/**
 * Adjusts the top-left coordinate of a rectangular area to account for various
 * alignments.
 *
 * e.g.
 *
 * x,y  dims     align         = result
 * -------------------------------------
 * 5,5  100,100  top_left      =   5,  5
 * 5,5  100,100  top_right     = -95,  5
 * 5,5  100,100  bottom_center = -45,-95
 *
 * @param [in,out] x
 * @param [in,out] y
 * @param [in]     dims
 * @param [in]     align
 */
void adjustPoint(int16_t& x, int16_t& y, Rect16& dims, Align align);

} // namespace Libp

#endif /* GRAPHICS_GRAPHICS_H_ */
