#ifndef TEXT_RENDER_H_
#define TEXT_RENDER_H_

#include <cstdio>
#include <cstring>
#include <algorithm>
#include "idrawing_surface.h"
#include "raster_font.h"

namespace Libp {

/**
 * Renders text on an IDrawingSurface.
 *
 * Supports bitmap fonts that implement the `IRasterFont` interface.
 *
 * Caution
 * -------
 * This code was developed for a specific application and has only been tested
 * to the the extent of that application's functionality. In other words it is
 * largely untested, likely contains minor bugs and inefficiencies and should
 * be tested carefully in any application you choose to use it in.
 *
 * @tparam T type of color used by the IDrawingSurface passed in the
 *           constructor #PrimitivesRender
 */
template <typename T>
class TextRender {
public:

    TextRender(IDrawingSurface<T>& surface) : surface_(surface) { }

    /**
     * Render a single ASCII character.
     *
     * @param x_pos x position of top left corner
     * @param y_pos y position of top left corner
     * @param font 1/2/4/8 bpp supported
     * @param character ASCII character code
     * @param color array of 2^bpp color elements
     *
     * @return 0 on error
     */
    template <RasterFontConcept Font>
    uint16_t drawChar(uint16_t x_pos, uint16_t y_pos, const Font& font, uint8_t character, const T color[])
    {
		if (!font.validChar(character))
			return 0;

		CharMeta meta = font.charMeta(character);
		const Image2d& image = font.image();
		uint16_t y_cursor = y_pos + meta.offs.y;;
		const uint16_t x_cursor_start = x_pos + meta.offs.x;

		// TODO: Optimize below loops
		switch(image.bpp) {
		case Bpp::_8bpp:
            for (uint16_t y = meta.pos.y; y < (meta.pos.y + meta.size.y); y++) {

				uint32_t pixel_idx = y * image.width + meta.pos.x;
				uint16_t x_cursor = x_cursor_start;

				for (uint16_t x = 0; x < meta.size.x; x++) {
					uint8_t byte = image.data[pixel_idx++];
					if (byte) {
						surface_.setPixel(x_cursor, y_cursor, color[byte]);
					}
					x_cursor++;
				}
                y_cursor++;
			}
			break;
		case Bpp::msb_4bpp:
            for (uint16_t y = meta.pos.y; y < (meta.pos.y + meta.size.y); y++) {

				uint32_t pixel_idx = y * image.width + meta.pos.x;
				uint16_t x_cursor = x_cursor_start;
	            const bool odd_pixel = pixel_idx & 0b1;

                for (uint16_t x = 0; x < meta.size.x; x++) {
                    uint32_t byte_idx = pixel_idx / 2;
                    uint8_t byte = image.data[byte_idx];
                    byte = odd_pixel
                            ? byte & 0b00001111
                            : byte >> 4;
                    if (byte) {
                        surface_.setPixel(x_cursor, y_cursor, color[byte]);
                    }
                    x_cursor++;
                    pixel_idx++;
                }
                y_cursor++;
			}
			break;
		case Bpp::msb_2bpp:
			for (uint16_t y = meta.pos.y; y < (meta.pos.y + meta.size.y); y++) {

			    uint32_t pixel_idx = y * image.width + meta.pos.x;
			    uint16_t x_cursor = x_cursor_start;

                for (uint16_t x = 0; x < meta.size.x; x++) {
					uint32_t byte_idx = pixel_idx / 4;
					uint8_t byte = image.data[byte_idx];
                    uint8_t shifts = 6 - ((pixel_idx % 4) << 1);
					uint8_t pixel_val = (byte >> shifts) & 0b00000011;
					if (pixel_val) {
						surface_.setPixel(x_cursor, y_cursor, color[pixel_val]);
					}
					x_cursor++;
					pixel_idx++;
				}
                y_cursor++;
			}
			break;
		case Bpp::msb_1bpp:
            for (uint16_t y =  meta.pos.y; y < (meta.pos.y + meta.size.y); y++) {

                uint32_t pixel_idx = y * image.width + meta.pos.x;
                uint16_t x_cursor = x_cursor_start;

                for (uint16_t x = 0; x < meta.size.x; x++) {
					uint32_t byte_idx = pixel_idx / 8;
					uint8_t byte = image.data[byte_idx];
					if (byte & (1 << (7 - (pixel_idx % 8)))) {
						surface_.setPixel(x_cursor, y_cursor, color[0]);
					}
	                x_cursor++;
					pixel_idx++;
				}
                y_cursor++;
			}
			break;
		default:
		    return 0;
		    // todo: unsupported error
		    break;
		}
		return meta.x_adv;
	}

    // TODO: update interface to return cursor x pos vs width
    template <RasterFontConcept Font>
    uint16_t drawText(int16_t x, int16_t y, const Font& font, const char* text, Align align, const T color[])
    {
    	if (align != Align::top_left) {
    	    Rect16 r = getTextBounds(font, text);
	        adjustPoint(x, y, r, align);
    	}
		uint16_t i = 0;
		while (text[i] != 0) {
			x += drawChar(x, y, font, text[i++], color);
		}
		return x;
    }

	/**
	 * Calculate the rendering dimensions for a piece of text in a specific
	 * font.
	 *
     * @param font
     * @param text null terminated string
     * @return
     */
    template <RasterFontConcept Font>
    static Rect16 getTextBounds(const Font& font, const char* text) {
		uint16_t i = 0;
		uint16_t width = 0;
		uint16_t max_height = 0;
		int16_t min_y_offs = 32000;
		// TODO: this is not exact, should use x_adv between and char size on last
		while (text[i] != 0) {
		    CharMeta meta = font.charMeta(text[i]);
			width += meta.x_adv;
			uint16_t total_height = meta.size.y + meta.offs.y;
            max_height = std::max(max_height, total_height);
            min_y_offs = std::min(min_y_offs, meta.offs.y);
			i++;
		}
		// TODO: document limits
		Rect16 r = { width, static_cast<uint16_t>(max_height + min_y_offs) };
		return r;
	}


	/**
	 * Trims a C-style string as required to fit within a defined width.
	 *
	 * If the string is trimmed, the final 3 (visible) characters will be
	 * overwritten with "..." and a null-terminator will be inserted.
	 * If there is no room for even "...", then a null-terminator will be
	 * inserted at index 0.
	 *
	 * @param font
	 * @param text [in,out] null terminated string
	 * @param max_width maximum width in pixels
	 */
    template <RasterFontConcept Font>
    static void trimText(const Font& font, char text[], uint16_t max_width)
	{
	    uint8_t text_len = strlen(text);
	    uint16_t render_width = getTextBounds(font, text).width;
	    if (render_width > max_width) {
	        constexpr char ellipsis[] = "...";
	        uint16_t ellipsis_width = getTextBounds(font, ellipsis).width;
	        if (ellipsis_width > max_width || text_len < 3) {
	            text[0] = '\0';
	            return;
	        }
	        // TODO: check!!
            text_len -= 2;
	        uint16_t allowed_text_width = max_width - ellipsis_width;
	        while (render_width > allowed_text_width) {
	            text[--text_len] = '\0';
	            render_width = getTextBounds(font, text).width;
	        }
	        while (text_len > 0 && text[--text_len] == ' ')
	            ;
	        // We are sure to have space as a minimum 3 chars were trimmed
	        text[++text_len] = '.';
	        text[++text_len] = '.';
	        text[++text_len] = '.';
	        text[++text_len] = '\0';
	    }
	}

private:
    IDrawingSurface<T>& surface_;
};

} // namespace Libp

#endif /* TEXT_RENDER_H_ */
