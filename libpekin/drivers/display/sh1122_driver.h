#ifndef SRC_SH1122_DRIVER_H_
#define SRC_SH1122_DRIVER_H_

#include <cstdint>
#include <algorithm>
#include "libpekin.h"
#include "bus/bus_concepts.h"
#include "graphics/idrawing_surface.h"

namespace Libp::Sh1122 {

namespace Cmd {
    static constexpr uint8_t set_contrast =          0x81; ///< followed by 1-byte (00 -> ff, low -> high)
    static constexpr uint8_t set_col_addr_normal =   0xa0; ///< note: will change addr counter
    static constexpr uint8_t set_col_addr_rev =      0xa1; ///< note: will change addr counter
    static constexpr uint8_t force_display_on_set =  0xa5; ///< force all display on regardless of RAM content
    static constexpr uint8_t force_display_on_clr =  0xa4; ///< restore display to normal (show RAM) mode
    static constexpr uint8_t set_inverse_on =        0xa7;
    static constexpr uint8_t set_inverse_off =       0xa6;
    static constexpr uint8_t set_multiplex_ratio =   0xa8; ///< followed by 1-byte (00 -> 3f), see datasheet
    static constexpr uint8_t set_dcdc_mode =         0xad; ///< followed by 1-byte, see datasheet (0x80=off)
    static constexpr uint8_t set_display_on =        0xaf;
    static constexpr uint8_t set_display_off =       0xae;
    static constexpr uint8_t set_row_addr =          0xb0; ///< followed by 1-byte (00 -> 3f) row number
    static constexpr uint8_t set_scan_dir_normal =   0xc0;
    static constexpr uint8_t set_scan_dir_rev =      0xc8;
    static constexpr uint8_t set_disp_offs =         0xd3; ///< followed by 1-byte (00 -> 3f) row number
    static constexpr uint8_t set_osc_and_disp_clk =  0xd5; ///< followed by 1-byte (00 -> ff), see datasheet
    static constexpr uint8_t set_pre_dis_chrg_pd =   0xd9; ///< followed by 1-byte (00 -> ff), see datasheet
    static constexpr uint8_t set_vcom_desel_lvl =    0xdb; ///< followed by 1-byte (00 -> ff), see datasheet
    static constexpr uint8_t set_vsegm_prechrg_lvl = 0xdc; ///< followed by 1-byte (00 -> ff), see datasheet
    static constexpr uint8_t read_mod_write =        0xe0; ///< Start Read-Modify-Write mode
    static constexpr uint8_t read_mod_write_end =    0xee; /**< Cancel Read-Modify-Write mode and returns column
                                                                address to the address before the mode was set */
    static constexpr uint8_t noop =                  0xe3;
}

/**
 * Driver for the SH1122 display IC.
 *
 * - 256x64x4-bit SRAM buffer
 * - 8-bit SPI/I2C serial or 6800/8080 parallel
 *
 * Designed for device-independent SPI, but should be easily adaptable to the
 * other supported interfaces.
 *
 * Limitations:
 * ------------
 * The driver is intended for situations where a full buffer is available. Read
 * operations are not supported by the serial interfaces and are not supported
 * by this driver. Since each pixel is 4-bits, but write operations are 8-bit,
 * all writes must occur on even horizontal pixel boundaries and operate on an
 * even number of horizontal pixels.
 *
 * The integrated DC-DC converter is not supported.
 *
 * Tested on a 72 MHz system. At higher speeds, delays may be necessary to
 * satisfy SH1122 bus timing requirements. Max SPI frequency ~2 MHz. See
 * datasheet for details.
 *
 * @tparam DataBus
 * @tparam U reset pin
 * @tparam V cs pin
 * @tparam X a0 pin
 */
template <Bus8BitWritable DataBus, GpioPinSettable U, GpioPinSettable V, GpioPinSettable X>
class Sh1122Driver : public IDrawingSurface<uint8_t> {
public:
    /**
     * @param bus
     * @param reset Reset pin
     * @param cs Chip select pin
     * @param a0 A0 pin
     */
    Sh1122Driver(DataBus& bus, U& reset, V& cs, X& a0, uint16_t x_res, uint16_t y_res)
        : bus_(bus), reset_(reset), cs_(cs), a0_(a0), width_(x_res), height_(y_res) {}

    static constexpr uint8_t init_sequence[] = {
            Cmd::set_display_off,
            Cmd::set_contrast,
            0x80,
            Cmd::set_col_addr_normal,
            Cmd::set_scan_dir_normal,
            0x40, // default display start line
            Cmd::set_inverse_off,
            // default
            Cmd::set_multiplex_ratio,
            0x3f,
            // No internal DCDC
            Cmd::set_dcdc_mode,
            0x80,
            // No display offset
            Cmd::set_disp_offs,
            0x00,
            // default
            Cmd::set_osc_and_disp_clk,
            0x50,
            // default
            Cmd::set_pre_dis_chrg_pd,
            0x22,
            // default
            Cmd::set_vcom_desel_lvl,
            0x35,
            // default
            Cmd::set_vsegm_prechrg_lvl,
            0x35,
            0x30, // default VSL discharge level
    };

    void initDisplay()
    {
        reset_.set();
        delayMs(1);
        reset_.clear();
        delayMs(10);
        reset_.set();
        delayMs(20);

        cs_.clear();
        bus_.write(init_sequence, sizeof(init_sequence));

        // Clear memory
        bus_.write8(Cmd::read_mod_write);
        bus_.read8(); // dummy read - why? // TODO: how is this working with writable bus concept..?
        a0_.set();
        for (uint16_t i = 0; i < 128 * 64; i++)
            bus_.write8(0x00);
        a0_.clear();
        bus_.write8(Cmd::read_mod_write_end);
        bus_.write8(Cmd::set_display_on);

        cs_.set();
        delayMs(100);
    }

    void setPower(bool on)
    {
        cs_.clear();
        bus_.write8(on ? Cmd::set_display_on : Cmd::set_display_off);
        cs_.set();
    }

    /// 0 -> 0xFF, not very linear
    void setBrightness(uint8_t brightness)
    {
        cs_.clear();
        bus_.write8(Cmd::set_contrast);
        bus_.write8(brightness);
        bus_.write8(Cmd::set_vsegm_prechrg_lvl);
        bus_.write8(brightness < 0xc0 ? 0 : brightness - 0xc0);
        cs_.set();
        // equal brightness
        // precharge 0x00, contrast 0xff
        // precharge 0x35, contrast 0x7c
    }

    /// color must be 4-bit value
    /// Sets 2 pixels
    void setPixel(uint16_t x, uint16_t y, uint8_t color) override
    {
        color = color << 4 | color;
        cs_.clear();
        setCursor(x >> 1, y);
        bus_.write8(Cmd::read_mod_write);
        a0_.set();
        bus_.write8(color);
        a0_.clear();
        bus_.write8(Cmd::read_mod_write_end);
        cs_.set();
    }

    /// color must be 4-bit value
    void fillLine(uint16_t x, uint16_t y, uint16_t length, uint8_t color) override
    {
        if (x >= width_ || y >= height_) // offscreen
            return;
        // Since we can't read pixels, we can't write half bytes
        if ( (x & 0b1) || (length & 0b1) )
            return;

        if (x + length > width_) // skip offscreen portion
            length = width_ - x;
        color = color << 4 | color;
        cs_.clear();
        setCursor(x >> 1, y);
        bus_.write8(Cmd::read_mod_write);
        a0_.set();
        for (uint16_t i = 0; i < (length >> 1); i++)
            bus_.write8(color);
        a0_.clear();
        bus_.write8(Cmd::read_mod_write_end);
        cs_.set();
    }


    /// color must be 4-bit value
    void fillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t color) override
    {
        if (x >= width_ || y >= height_) // offscreen
            return;
        // Since we can't read pixels, we can't write half bytes
        if ( (x & 0b1) || (width & 0b1) )
            return;
        color = color << 4 | color;
        cs_.clear();
        for (uint8_t y_idx = y; y_idx < y + height; y_idx++) {
            setCursor(x >> 1, y_idx);
            bus_.write8(Cmd::read_mod_write);
            a0_.set();
            for (uint16_t i = 0; i < (width >> 1); i++)
                bus_.write8(color);
            a0_.clear();
            bus_.write8(Cmd::read_mod_write_end);
        }
        cs_.set();
    }

    /// color must be 4-bit value
    void fillScreen(uint8_t color) override
    {
        color = color << 4 | color;
        cs_.clear();

        //setCursor(0, 0);
        bus_.write8(Cmd::set_row_addr);
        bus_.write8(0);
        bus_.write8(0);
        bus_.write8(0x10);

        bus_.write8(Cmd::read_mod_write);
        a0_.set();
        for (uint16_t i = 0; i < (width_ * height_ / 2); i++)
            bus_.write8(color);
        a0_.clear();
        bus_.write8(Cmd::read_mod_write_end);
        cs_.set();
    }


    void copyRect(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* data) override
    {
        if (x >= width_ || y >= height_ || x < -width || y < -height) // offscreen
            return;
        // Since we can't read pixels, we can't write half bytes
        if ( (x & 0b1) || (width & 0b1) )
            return;

        cs_.clear();
        uint32_t arr_idx = 0;

        // Skip vertical offscreen portions
        if (y < 0) {
            height += y;
            arr_idx += (width << 1) * -y;
            y = 0;
        }
        if (y + height > height_) {
            height = height_ - y;
        }

        if (width == width_ && x == 0) {
            setCursor(0, y);
            bus_.write8(Cmd::read_mod_write);
            a0_.set();

            //uint32_t len = width * height / 2;
            //while (arr_idx < len)
             //   bus_.write8(data[arr_idx++]);
            const uint32_t len = width * height / 2;
            bus_.write(&data[arr_idx], len);
            a0_.clear();
            bus_.write8(Cmd::read_mod_write_end);
        }
        else {
            int16_t offscreen_bytes = ((x + width) - width_) >> 1;
            if (offscreen_bytes > 0)
                width = width_ - x;
            else
                offscreen_bytes = 0;
            /*int16_t offscreen_bytes = ((x + width) - width_) >> 1;
            offscreen_bytes = std::max(static_cast<int16_t>(0), offscreen_bytes);
            if (offscreen_bytes)
                width = width_ - x;*/
            uint16_t x_idx = x >> 1;
            for (uint16_t y_idx = y; y_idx < y + height; y++) {
                setCursor(x_idx, y_idx);
                bus_.write8(Cmd::read_mod_write);
                a0_.set();
                //for (uint16_t i = width; i > 0; i-=2)
                //    bus_.write8(data[arr_idx++]);
                bus_.write(&data[arr_idx], width / 2);
                arr_idx += offscreen_bytes;
                a0_.clear();
                bus_.write8(Cmd::read_mod_write_end);
            }
        }
        cs_.set();
    }

    void setOrientation(Orientation orientation) override
    {
        if (orientation == Orientation::portrait || orientation == Orientation::portrait_rev)
            return;

        uint8_t col_addr_cmd, scan_dir_cmd, row_offs;

        if (orientation == Orientation::landscape) {
            col_addr_cmd = Cmd::set_col_addr_normal;
            scan_dir_cmd = Cmd::set_scan_dir_normal;
            row_offs = 0x40;
        }
        else {
            col_addr_cmd = Cmd::set_col_addr_rev;
            scan_dir_cmd = Cmd::set_scan_dir_rev;
            row_offs = 0x60;
        }
        cs_.clear();
        bus_.write8(Cmd::set_display_off);
        bus_.write8(col_addr_cmd); // col flip
        bus_.write8(scan_dir_cmd); // row flip
        bus_.write8(row_offs);
        bus_.write8(Cmd::set_display_on);
        cs_.set();
    }

    void setInverted(bool inverted) override
    {
        cs_.clear();
        bus_.write8(inverted ? Cmd::set_inverse_on : Cmd::set_inverse_off);
        cs_.set();
    }

    uint16_t getWidth()  override { return width_;  }
    uint16_t getHeight() override { return height_; }
    //void flush() override { }

private:
    const DataBus& bus_;
    const U& reset_;
    const V& cs_;
    const X& a0_;

    // Raw unchanging native width/height
    // Portrait is not supported
    const uint16_t width_;
    const uint16_t height_;

    inline
    void setCursor(uint16_t x, uint16_t y)
    {
        bus_.write8(Cmd::set_row_addr);
        bus_.write8(y);
        bus_.write8(15 & x);
        bus_.write8(0x10 | (x >> 4) );
    }
};

} // namespace Libp::Sh1122

#endif /* SRC_SH1122_DRIVER_H_ */
