#ifndef SRC_SSD1362_DRIVER_H_
#define SRC_SSD1362_DRIVER_H_

#include <cstdint>
#include <algorithm>
#include "libpekin.h"
#include "bus/bus_concepts.h"
#include "graphics/idrawing_surface.h"

namespace Libp::Ssd1362 {

inline constexpr uint16_t x_res = 256;
inline constexpr uint16_t y_res = 64;

namespace Cmd {
    static constexpr uint8_t set_col_addr        = 0x15; ///< followed by 2-bytes, see datasheet
    static constexpr uint8_t set_row_addr        = 0x75; ///< followed by 2-bytes, see datasheet
    static constexpr uint8_t set_contrast        = 0x81; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_remap           = 0xa0; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_start_line      = 0xa1; ///< followed by 1-byte (0 -> 63), see datasheet
    static constexpr uint8_t set_vert_offs       = 0xa2; ///< followed by 1-byte (0 -> 63), see datasheet
    static constexpr uint8_t set_vert_scrl_area  = 0xa3; ///< followed by 2-bytes, see datasheet
    static constexpr uint8_t set_mode_normal     = 0xa4;
    static constexpr uint8_t set_mode_allon      = 0xa5;
    static constexpr uint8_t set_mode_alloff     = 0xa6;
    static constexpr uint8_t set_mode_inverse    = 0xa7;
    static constexpr uint8_t set_mux_ratio       = 0xa8; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_vdd_reg_src     = 0xab; ///< followed by 1-byte (0 = ext. Vdd, 1 = int. Vdd)
    static constexpr uint8_t set_iref_src        = 0xad; ///< followed by 1-byte (0x8E = ext. Iref, 0x9E = int. Iref)
    static constexpr uint8_t set_display_off     = 0xae;
    static constexpr uint8_t set_display_on      = 0xaf;
    static constexpr uint8_t set_phase_len       = 0xb1; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_clk_div         = 0xb3; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_gpio            = 0xb5; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_2nd_prechrg_pd  = 0xb6; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_greyscale_lut   = 0xb8; ///< followed by 15-bytes, see datasheet
    static constexpr uint8_t set_default_lut     = 0xb9; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_prechrg_lvl     = 0xbc; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_prechrg_cap     = 0xbd; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_com_desel_lvl   = 0xbe; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_cmd_lock        = 0xfd; ///< followed by 1-byte, see datasheet
    static constexpr uint8_t set_fade_blink_mode = 0x23; ///< followed by 1-byte, see datasheet
}

/**
 * Driver for the Solomon Systech SSD1362 OLED ID.
 * - 256x64x4-bit SRAM buffer
 * - 8-bit SPI/I2C serial or 6800/8080 parallel
 *
 * Designed for device-independent SPI, but should be easily adaptable to the
 * other supported interfaces.
 *
 * Portrait modes not currently working - need to handle nibble issue.
 *
 * Limitations:
 * ------------
 * The driver is intended for situations where a full buffer is available. Read
 * operations are not supported by the serial interfaces and are not supported
 * by this driver. Since each pixel is 4-bits, but write operations are 8-bit,
 * all writes must occur on even horizontal pixel boundaries and operate on an
 * even number of horizontal pixels.
 *
 * Tested on a 72 MHz system. At higher speeds, delays may be necessary to
 * satisfy SSD1362 bus timing requirements. Max SPI frequency ~10 MHz. See
 * datasheet for details.
 *
 * @tparam DataBus
 * @tparam U reset pin type
 * @tparam V cs pin type
 * @tparam X dc pint type
 */
template <Bus8BitWritable DataBus, GpioPinSettable U, GpioPinSettable V, GpioPinSettable X>
class Ssd1362Driver : public IDrawingSurface<uint8_t> {
public:
    /**
     *
     * @param bus
     * @param reset Reset pin
     * @param cs Chip select pin
     * @param dc D/C# pin
     */
    Ssd1362Driver(DataBus& bus, U& reset, V& cs, X& dc, uint16_t x_res, uint16_t y_res)
        : bus_(bus), reset_(reset), cs_(cs), data_mode_(dc), width_(x_res), height_(y_res) {}

    void initDisplay(Orientation orientation = Orientation::landscape)
    {
        reset_.set();
        delayMs(1);
        reset_.clear();
        delayMs(1);
        reset_.set();
        delayMs(50);
        // defaults
        // external Iref, internal Vdd, no external pre-charge cap on Vp,
        cs_.clear();
        bus_.write(init_sequence, sizeof(init_sequence));
        cs_.set();
        setOrientation(orientation);
        fillScreen(0x00);
        delayMs(100);
    }

    void setPower(bool on)
    {
        cs_.clear();
        bus_.write8(on ? Cmd::set_display_on : Cmd::set_display_off);
        // TODO: disable internal reg if off
        cs_.set();
    }

    /// 0 -> 0xFF, not very linear
    void setBrightness(uint8_t brightness)
    {
        cs_.clear();
        bus_.write8(Cmd::set_contrast);
        bus_.write8(brightness);
        //bus_.write8(Cmd::set_vsegm_prechrg_lvl);
        //bus_.write8(brightness < 0xc0 ? 0 : brightness - 0xc0);
        cs_.set();
    }

    /// color must be 4-bit value
    /// Sets 2 pixels
    void setPixel(uint16_t x, uint16_t y, uint8_t color) override
    {
        color = color << 4 | color;
        cs_.clear();
        setCursor(x >> 1, y);
        data_mode_.set();
        bus_.write8(color);
        data_mode_.clear();
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
        data_mode_.set();
        for (uint16_t i = 0; i < (length >> 1); i++)
            bus_.write8(color);
        data_mode_.clear();
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
            // TODO: We'll wrap around to start position if it overflows
            setCursor(x >> 1, y_idx);
            data_mode_.set();
            for (uint16_t i = 0; i < (width >> 1); i++)
                bus_.write8(color);
            data_mode_.clear();
        }
        cs_.set();
    }

    /// color must be 4-bit value
    void fillScreen(uint8_t color) override
    {
        color = color << 4 | color;
        cs_.clear();
        setCursor(0, 0);
        data_mode_.set();
        for (uint16_t i = 0; i < (width_ * height_ / 2); i++)
            bus_.write8(color);
        data_mode_.clear();
        cs_.set();
    }


    void copyRect(int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t* data) override
    {
        if (x >= width_ || y >= height_ || x < -width || y < -height) // offscreen
            return;
        // Since we can't read pixels, we can't write half bytes
        if ( (x & 0b1) || (width & 0b1) )
            return;

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

        cs_.clear();
        if (width == width_ && x == 0) {
            setCursor(0, y);
            data_mode_.set();
            const uint32_t len = width * height / 2;
            bus_.write(&data[arr_idx], len);
            data_mode_.clear();
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
                data_mode_.set();
                bus_.write(&data[arr_idx], width / 2);
                //for (uint16_t i = width; i > 0; i-=2)
                //    bus_.write8(data[arr_idx++]);
                arr_idx += offscreen_bytes;
                data_mode_.clear();
            }
        }
        cs_.set();
    }

    // TODO: private
    // TODO: consteval
    constexpr uint8_t calcRemap(bool portrait, bool long_dim_rev, bool short_dim_rev)
    {
        constexpr uint8_t col_addr_remap_pos = 0;
        constexpr uint8_t nibble_remap_pos = 1;
        constexpr uint8_t horiz_addr_inc_pos = 2;
        constexpr uint8_t com_remap_pos = 4;
        constexpr uint8_t seg_split_odd_even = 6;
        //constexpr uint8_t seg_left_right_remap = 7;

        return  1 << seg_split_odd_even
              | short_dim_rev << com_remap_pos
              | portrait << horiz_addr_inc_pos
              | long_dim_rev << col_addr_remap_pos
              | short_dim_rev << nibble_remap_pos;
    }

    void setOrientation(Orientation orientation) override
    {
        uint8_t remap = 0;
        switch (orientation) {
        case Orientation::landscape:
            remap = calcRemap(false, true, false);
            break;
        case Orientation::landscape_rev:
            remap = calcRemap(false, false, true);
            break;
        case Orientation::portrait:
            remap = calcRemap(true, false, false);
            break;
        case Orientation::portrait_rev:
            remap = calcRemap(true, true, true);
            break;
        }
        cs_.clear();
        //bus_.write8(Cmd::display_off);
        bus_.write8(Cmd::set_remap);
        bus_.write8(remap);
        //bus_.write8(Cmd::display_on);
        cs_.set();
    }

    // Will turn set display to normal from all on or all off
    void setInverted(bool inverted) override
    {
        cs_.clear();
        bus_.write8(inverted ? Cmd::set_mode_inverse : Cmd::set_mode_normal);
        cs_.set();
    }

    uint16_t getWidth()  override { return width_;  }
    uint16_t getHeight() override { return height_; }
    //void flush() override { }

private:
    const DataBus& bus_;
    const U& reset_;
    const V& cs_;
    const X& data_mode_;

    static constexpr uint8_t init_sequence[] = {
            Cmd::set_display_off,
            Cmd::set_contrast,
            0x80,
            Cmd::set_display_on,
    };

    // Raw unchanging native width/height
    // Portrait is not supported
    // TODO: yes it is
    const uint16_t width_;
    const uint16_t height_;

    /// cs must be active
    inline
    void setCursor(uint16_t x, uint16_t y)
    {
        bus_.write8(Cmd::set_col_addr);
        bus_.write8(x);    // start addr
        bus_.write8(0x7f); // end addr
        bus_.write8(Cmd::set_row_addr);
        bus_.write8(y);    // start addr
        bus_.write8(0x3f); // end addr
    }
};

} // namespace Libp::Ssd1362

#endif /* SRC_SSD1362_DRIVER_H_ */
