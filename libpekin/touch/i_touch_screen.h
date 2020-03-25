#ifndef LIB_LIBPEKIN_TOUCH_I_TOUCH_SCREEN_H_
#define LIB_LIBPEKIN_TOUCH_I_TOUCH_SCREEN_H_

#include <cstdint>

namespace Libp::ResistiveTouch {

struct Point {
    uint16_t x;
    uint16_t y;
};

/// Resistive touch screen interface
class ITouchScreen {
public:
    virtual ~ITouchScreen() = 0;

    virtual bool isTouched() = 0;

    /**
     * Maximum supported resolution = 10-bit (i.e. 0->4095)
     */
    virtual bool readPos(Point* p) = 0;
};

} // namespace Libp::ResistiveTouch


#endif /* LIB_LIBPEKIN_TOUCH_I_TOUCH_SCREEN_H_ */
