#include <cstdint>
#include "graphics.h"

namespace Libp {

void adjustPoint(int16_t& x, int16_t& y, Rect16& dimensions, Align align)
{
    switch (align) {
    case Align::top_center:
        x = x - dimensions.width / 2;
        break;
    case Align::top_right:
        x = x - dimensions.width;
        break;
    case Align::top_left:
        break;
    case Align::middle_left:
        y = y - dimensions.height / 2;
        break;
    case Align::middle_center:
        y = y - dimensions.height / 2;
        x = x - dimensions.width / 2;
        break;
    case Align::middle_right:
        y = y - dimensions.height / 2;
        x = x - dimensions.width;
        break;
    case Align::bottom_left:
        y = y - dimensions.height;
        break;
    case Align::bottom_center:
        y = y - dimensions.height;
        x = x - dimensions.width / 2;
        break;
    case Align::bottom_right:
        y = y - dimensions.height;
        x = x - dimensions.width;
        break;
    }
}

} // namespace Libp
