#ifndef IDRAWING_SURFACE_H_
#define IDRAWING_SURFACE_H_

#include <cstdint>

namespace Libp {

enum class Orientation : uint8_t {
    portrait = 0, landscape, portrait_rev, landscape_rev
};

/**
 * Basic functions to draw on a writable bitmap device.
 *
 * @tparam T type for color values
 */
template <typename T>
class IDrawingSurface {
public:
    virtual ~IDrawingSurface() = default;

    virtual void setPixel(uint16_t x, uint16_t y, T color) = 0;

    virtual void fillLine(uint16_t x, uint16_t y, uint16_t length, T color) = 0;

    /**
     * Draw a solid rectangle.
     *
     *
     * @param x
     * @param y
     * @param w
     * @param h
     * @param color
     */
    virtual void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, T color) = 0;

    // TODO: document out of bounds behaviour and enforce in implementations
    /**
     * Copy an array of color values to the screen.
     *
     * @param x top-left corner x-coordinate. May be off screen
     * @param y top- right corner x-coordinate. May be off screen
     * @param w width.
     * @param h height.
     * @param color array of color values to copy to the screen.
     */
    virtual void copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const T* color) = 0;

    /**
     * Fill the entire screen with a single color.
     *
     * @param color
     */
    virtual void fillScreen(T color) = 0;

    /**
     * Set the display color inversion.
     *
     * @param inverted if true, colors will be inverted.
     */
    virtual void setInverted(bool inverted) = 0;

    /**
     * Set the display orientation (i.e. which edge is the 'top').
     *
     * @param orientation
     */
    virtual void setOrientation(Orientation orientation) = 0;

    // TODO: document if this is before/after orientation change

    virtual uint16_t getWidth() = 0;

    virtual uint16_t getHeight() = 0;

//    virtual void flush() = 0;
};

} // namespace Libp

#endif /* IDRAWING_SURFACE_H_ */


