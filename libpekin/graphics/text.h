#ifndef LIB_LIBPEKIN_GRAPHICS_TEXT_H_
#define LIB_LIBPEKIN_GRAPHICS_TEXT_H_

#include "graphics.h"
#include <cstdint>
#if __GNUC__ >= 10
#include <concepts>
#endif

namespace Libp {

/// Metadata (position, size etc.) for a single bitmap character in a font
/// bitmap image.
struct CharMeta {
    /// Position in the font bitmap image.
    const Point2dU pos;
    /// Size in the font bitmap image.
    const Point2dU size;
    /// Drawing offset.
    const Point2dS offs;
    /// Number of x pixels to advance to draw the next character.
    const uint16_t x_adv;
};

/// Raster font interface
template<typename T>
concept RasterFontConcept = requires (T font, uint8_t ch) {
#if __GNUC__ >= 10
    { font.validChar(ch) } -> std::same_as<bool>;
    { font.charMeta(ch)  } -> std::same_as<const CharMeta&>;
    { font.image()       } -> std::same_as<const Image2d&>;
#else
    { font.validChar(ch) } -> bool;
    { font.charMeta(ch)  } -> const CharMeta&;
    { font.image()       } -> const Image2d&;
#endif

};

} // namespace Libp

#endif /* LIB_LIBPEKIN_GRAPHICS_TEXT_H_ */
