/**
 * IDrawingSurface implementations for 2D display buffers of varying bit
 * depths.
 *
 * Not optimized. Intended for testing of embedded rendering code on PCs.
 */
#ifndef DISPLAY_BUFFER_H_
#define DISPLAY_BUFFER_H_

#include <cstdint>
#include <cstring>
#include "idrawing_surface.h"

namespace Libp {

/**
 *
 * @tparam width
 * @tparam height
 */
template <uint16_t width, uint16_t height>
class DisplayBuf1bpp : public IDrawingSurface<uint8_t> {
public:
    void fillScreen(uint8_t value) override
    {
        uint8_t byte_val = value ? 0xff : 0;
        memset(buffer_, byte_val, len);
    }
    inline __attribute__((always_inline)) void setPixel(uint16_t x, uint16_t y, uint8_t value) override
    {
        if (x >= width || y >= height)
            return;
		uint8_t mask = 1 << (7 - (x % 8));
		if (value)
			buffer_[y][x / 8] |= mask;
		else
			buffer_[y][x / 8] &= (~mask);
    }
    inline __attribute__((always_inline)) void fillLine(uint16_t x, uint16_t y, uint16_t length, uint8_t value) override
    {
    	uint8_t leading_bits = 8 - x % 8;
    	while (length-- > 0 && leading_bits-- > 0 ) {
    		setPixel(x++, y, value);
    	}
    	length++;
    	uint32_t x_offs = x / 8;
		memset(&buffer_[y][x_offs], value ? 0xff : 0, length / 8);
		uint8_t trailing_bits = length % 8;
		x = x + length - trailing_bits;
		while (trailing_bits-- > 0) {
			setPixel(x++, y, value);
		}
    }
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color) override {}
    void copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* color) override {}
    void setOrientation(Orientation orientation) override {}
    void setInverted(bool inverted) override {}
    uint16_t getWidth() override { return width;  }
    uint16_t getHeight()override { return height; }
    uint8_t* buffer() {
        return static_cast<uint8_t*>(&buffer_[0][0]);
    }
private:
    static_assert(!(width%2), "Width must be a multiple of 8");
    static constexpr uint32_t len = width * height / 8;
    uint8_t buffer_[height][width / 8];
};


/**
 * MSB = left pixel
 *
 * @tparam width must be a multiple of 2
 * @tparam height
 */
template <uint16_t width, uint16_t height>
class DisplayBuf4bpp : public IDrawingSurface<uint8_t> {
public:
    void fillScreen(uint8_t value) override
    {
        uint8_t byte_val = value << 4 | value;
        memset(buffer_, byte_val, len);
    }
    inline __attribute__((always_inline))
    void setPixel(uint16_t x, uint16_t y, uint8_t value) override
    {
        if (x >= width || y >= height)
            return;
        if (x & 0b1)
            buffer_[y][x >> 1] = (buffer_[y][x >> 1] & 0xf0) | value;
        else
            buffer_[y][x >> 1] = (buffer_[y][x >> 1] & 0x0f) | value << 4;
    }
    void fillLine(uint16_t x, uint16_t y, uint16_t length, uint8_t color) override
    {
    	// TODO: TEST
    	// TODO: bounds checking
    	if (length == 0)
    		return;
    	if (x >= width || y >= height)
    	    return;
    	uint16_t x_offs = x >> 1;
    	// Set leading nibble if present
    	if (x & 0b1) {
    		buffer_[y][x_offs] = (buffer_[y][x_offs] & 0xf0) | color;
			x_offs++;
			length--;
    	}
    	// Set trailing nibble if present
    	if (length & 0b1) {
    		buffer_[y][x_offs + length/2] = (buffer_[y][x_offs + length/2] & 0x0f) | color << 4;
    		length--;
    	}
    	if (length) {
            uint8_t byte_val = color << 4 | color;
    		memset(&buffer_[y][x_offs], byte_val, length / 2);
    	}
    }
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color) override {
        if (w ==0 || h == 0 || x >= width || y >= height)
            return;
        if (y + h > height)
            h = height - y;
        if (x + w > width)
            w = width - x;
        // TODO; optimize
        uint8_t byte_val = color << 4 | color;
        for (uint16_t y_idx = y; y_idx < y + h; y_idx++) {
            uint16_t remaining_w = w;
            uint16_t x_offs = x >> 1;
            // Set leading nibble if present
            if (x & 0b1) {
                buffer_[y_idx][x_offs] = (buffer_[y_idx][x_offs] & 0xf0) | color;
                x_offs++;
                remaining_w--;
            }
            // Set trailing nibble if present
            if (remaining_w & 0b1) {
                buffer_[y_idx][x_offs + remaining_w/2] = (buffer_[y_idx][x_offs + remaining_w/2] & 0x0f) | color << 4;
                remaining_w--;
            }
            if (remaining_w) {
                memset(&buffer_[y_idx][x_offs], byte_val, remaining_w / 2);
            }
        }
    }
    /// Currently requires byte aligned output (i.e. x%2 == 0)
    void copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* color) override
    {
        // doesn't support non aligned output
        if ( (x & 0b1) || (w & 0b1) )
            return;

        uint32_t arr_idx = 0;
        uint16_t x_idx = x >> 1;

        uint16_t len = w >> 1;
        uint16_t onscreen_len = x + w > width ? (width - x) >> 1 : len;
        uint16_t last_row = y + h > height ? height : y + h;

        for (uint16_t row = y; row < last_row; row++) {
            memcpy(&buffer_[row][x_idx], &color[arr_idx], onscreen_len);
            arr_idx += len;
        }
    }
    void copyRect2(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* color)
    {
        /*
         * even array, even x -> normal copyRect
         * even array,  odd x
         *  odd array, even x
         *  odd array,  odd x
         *
         *
         *
         */



        uint32_t arr_idx = 0;
        // TODO: deal with negative X/Y
        uint16_t x_offs = x >> 1;

        uint16_t width_bytes = w >> 1;

        // Constrain to onscreen portion
        uint16_t onscreen_len = x + w > width ? (width - x) >> 1 : width_bytes;
        uint16_t last_visible_row = y + h > height ? height : y + h;


        for (uint16_t row = y; row < last_visible_row; row++) {

            if (x & 0b1) {
                buffer_[row][x_offs] = (buffer_[row][x_offs] & 0x0f) | color[arr_idx] << 4;
            }
            x_offs++;




            memcpy(&buffer_[row][x_offs], &color[arr_idx], onscreen_len);
            arr_idx += width_bytes;



        }




    }
    void setOrientation(Orientation orientation) override {}
    void setInverted(bool inverted) override {}
    uint16_t getWidth() override  { return width;  }
    uint16_t getHeight()override  { return height; }
    uint8_t* buffer() {
        return static_cast<uint8_t*>(&buffer_[0][0]);
    }
private:
    static_assert(!(width%2), "Width must be a multiple of 2");
    static constexpr uint32_t len = width * height / 2;
    uint8_t buffer_[height][width / 2];
};


template <uint16_t width, uint16_t height>
class DisplayBuf6bpp : public IDrawingSurface<uint8_t> {
public:
    void fillScreen(uint8_t value) override
    {
        if (value == 0)
            memset(buffer_, 0, len);
        else {
            uint8_t* buf = (uint8_t*) buffer_;
            uint8_t vals[3];
            vals[0] = value << 2 | value >> 4;
            vals[1] = value << 4 | value >> 2;
            vals[2] = value << 6 | value;
            for (uint32_t i = 0; i < len; i += 3) {
                buf[i] = vals[0];
                buf[i+1] = vals[1];
                buf[i+2] = vals[2];
            }
        }
    }
    inline __attribute__((always_inline)) void setPixel(uint16_t x, uint16_t y, uint8_t value) override
    {
        if (x >= width || y >= height)
            return;
        uint32_t offs = x * 3 / 4;
        switch (x % 4) {
        case 0:
            buffer_[y][offs] = (buffer_[y][offs] & 0b00000011) | value << 2;
            break;
        case 1:
            buffer_[y][offs  ] = (buffer_[y][offs  ] & 0b11111100) | value >> 4;
            buffer_[y][offs+1] = (buffer_[y][offs+1] & 0b00001111) | value << 4;
            break;
        case 2:
            buffer_[y][offs  ] = (buffer_[y][offs  ] & 0b11110000) | value >> 2;
            buffer_[y][offs+1] = (buffer_[y][offs+1] & 0b00111111) | value << 6;
            break;
        case 3:
            buffer_[y][offs] = (buffer_[y][offs] & 0b11000000) | value;
            break;
        }
    }
    inline __attribute__((always_inline)) void fillLine(uint16_t x, uint16_t y, uint16_t length, uint8_t value) override
    {
        // TODO: TEST
        // TODO: bounds checking
        if (length == 0)
            return;
        // TODO get repeating sequence and copy
        while (length-- > 0)
            setPixel(x++, y, value);
/*        uint32_t x_offs = x >> 1;
        // Set leading nibble if present
        if (x & 0b1) {
            buffer_[y][x_offs] = (buffer_[y][x_offs] & 0xf0) | value;
            x_offs++;
            length--;
        }
        // Set trailing nibble if present
        if (length & 0b1) {
            buffer_[y][x_offs + length/2] = (buffer_[y][x_offs + length/2] & 0x0f) | value << 4;
            length--;
        }
        if (length) {
            uint8_t byte_val = value << 4 | value;
            memset(&buffer_[y][x_offs], byte_val, length / 2);
        }*/
    }
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color) override {}
    void copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* color) override {}
    void setOrientation(Orientation orientation) override {}
    void setInverted(bool inverted) override {}
    uint16_t getWidth() override  { return width;  }
    uint16_t getHeight()override  { return height; }
    uint8_t* buffer() {
        return static_cast<uint8_t*>(&buffer_[0][0]);
    }
private:
    static_assert(!(width%3), "Width must be a multiple of 3");
    static constexpr uint32_t len = width * height * 3 / 4;
    uint8_t buffer_[height][width * 3 / 4];
};

} // namespace Libp

#endif /* DISPLAY_BUFFER_H_ */
