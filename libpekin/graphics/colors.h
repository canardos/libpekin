#ifndef SRC_GRAPHICS_COLORS_H_
#define SRC_GRAPHICS_COLORS_H_

#include <cstdint>

/**
 * RGB565 HTML colors
 */
namespace Libp::Rgb565Colors {
    inline constexpr uint16_t white    = 0xFFFF;
    inline constexpr uint16_t silver   = 0xBDF7;
    inline constexpr uint16_t gray     = 0x7BEF;
    inline constexpr uint16_t black    = 0x0000;
    inline constexpr uint16_t red      = 0xF800;
    inline constexpr uint16_t maroon   = 0x7800;
    inline constexpr uint16_t yellow   = 0xFFE0;
    inline constexpr uint16_t olive    = 0x7BE0;
    inline constexpr uint16_t lime     = 0x07E0;
    inline constexpr uint16_t green    = 0x03E0;
    inline constexpr uint16_t aqua     = 0x07FF;
    inline constexpr uint16_t teal     = 0x03EF;
    inline constexpr uint16_t blue     = 0x001F;
    inline constexpr uint16_t navy     = 0x000F;
    inline constexpr uint16_t fuchsia  = 0xF81F;
    inline constexpr uint16_t purple   = 0x780F;
}

#endif /* SRC_GRAPHICS_COLORS_H_ */
