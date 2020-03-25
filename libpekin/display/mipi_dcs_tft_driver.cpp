#include "display/mipi_dcs_tft_driver.h"
#include "libpekin.h"

namespace Libp {

MipiDcsTftDriver::MipiDcsTftDriver(IBasicBus16& bus, uint16_t x_res, uint16_t y_res) :
        bus_(bus), panel_x_res_(x_res), panel_y_res_(y_res),
        width_(x_res), height_(y_res)
{

}

void MipiDcsTftDriver::init(
        const CmdSequence init_commands, uint16_t init_commands_len,
        Orientation orientation, bool inverted)
{
    if (init_commands != nullptr) {
        execCmdSequence(init_commands, init_commands_len);
    }
    setOrientation(orientation);
    setInverted(inverted);
}

void MipiDcsTftDriver::writeCmd(uint8_t cmd, const uint8_t * params, int8_t n_param)
{
    bus_.write8Cmd(cmd);
    while (n_param-- > 0) {
        bus_.write8(*params++);
    }
}

void MipiDcsTftDriver::execCmdSequence(const CmdSequence sequence, int16_t len)
{
    int16_t bytes_to_read = len;

    const uint8_t * seq_ptr = static_cast<const uint8_t*>(sequence);
    const uint8_t max_params = *seq_ptr++;
    bytes_to_read--;
    // Warning: C99 VLA array is not valid C++
    // Confirm support if not using GCC
    uint8_t params[max_params];

    while (bytes_to_read > 0) {
        uint8_t cmd = *seq_ptr++;
        uint8_t n_param = *seq_ptr++;
        if (cmd == cust_cmd_delay) {
            delayMs(n_param);
            bytes_to_read -= 2;
        }
        else {
            for (uint8_t i = 0; i < n_param; i++) {
                params[i] = *seq_ptr++;
            }
            writeCmd(cmd, params, n_param);
            bytes_to_read -= 2 + n_param;
        }
    }
}

void MipiDcsTftDriver::setPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= width_ || y >= height_)
        return;
    // TODO: optimize/test
    setMemoryWindow(x, y, x, y);
    bus_.write8Cmd(MipiDcs::Cmd::write_memory_start);

}

void MipiDcsTftDriver::fillLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
    // TODO: optimize/test
    fillRect(x, y, length, 1, color);
    bus_.write16(color);
}

void MipiDcsTftDriver::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Deal with negative width/height by shifting x/y
/*    if (w < 0) {
        x += w;
        w = -w;
    }
    if (h < 0) {
        y += h;
        y -= h;
    }*/
    uint16_t x1 = x + w;
    uint16_t y1 = y + h;

    // Ignore call if everything is outside screen area
    if (/*x1 < 0 || */x >= width_ || /*y1 < 0 || */y >= height_)
        return;

    // Clip portion outside screen area
    /*if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;*/
    if (x1 > width_)
        x1 = width_;
    if (y1 > height_)
        y1 = height_;

    setMemoryWindow(x, y, x1 - 1, y1 - 1);
    bus_.write8Cmd(MipiDcs::Cmd::write_memory_start);
    bus_.write16Repeat(color, static_cast<uint32_t>(h) * w);
}

void MipiDcsTftDriver::fillScreen(uint16_t color)
{
    fillRect(0, 0, width_, height_, color);
}

void MipiDcsTftDriver::copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint16_t* color)
{
    // TODO: negative x/y
    setMemoryWindow(x, y, x + w - 1, y + h - 1);
    bus_.write8Cmd(MipiDcs::Cmd::write_memory_start);
    bus_.write16Array(color, static_cast<uint32_t>(h) * w);
}

void MipiDcsTftDriver::setMemoryWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint8_t cmd_params[4] = {
        static_cast<uint8_t>(x0 >> 8),
        static_cast<uint8_t>(x0),
        static_cast<uint8_t>(x1 >> 8),
        static_cast<uint8_t>(x1)
    };
    writeCmd(MipiDcs::Cmd::set_column_address, cmd_params, 4);

    const uint8_t cmd_params2[4] = {
        static_cast<uint8_t>(y0 >> 8),
        static_cast<uint8_t>(y0),
        static_cast<uint8_t>(y1 >> 8),
        static_cast<uint8_t>(y1)
    };
    writeCmd(MipiDcs::Cmd::set_page_address, cmd_params2, 4);
}

void MipiDcsTftDriver::setOrientation(Orientation orientation)
{
    // Bits[7:5] Host<->frame memory transfer order
    // - bit 7: Page address order (0=top->bottom, 1=bottom->top)
    // - bit 6: Column address order (0-left->right, 1=right->left)
    // - bit 5: Page/column addressing order (0=normal, 1=reversed)
    //
    // Bits[4:0] Frame memory<->display transfer order
    // - bit 4: Display device line refresh order (0=top->bottom line,
    //          1=bottom->top line)
    // - bit 3: RGB/BGR order (0=RGB, 1=BGR)
    // - bit 2: Display data latch data order (0=left->right,
    //          1=right->left)
    // - bit 1: Flip horizontal (0=normal, 1=flipped)
    // - bit 0: Flip vertical (0=normal, 1=flipped)

    //                                         7 - Page address order
    //                                         | 5 - Page/col swap
    //                                         | | 3 - RGB/BGR order
    //                                         | | | 1 - Flip horizontal
    //                                         | | | |
    static constexpr uint8_t portrait      = 0b01001000;
    static constexpr uint8_t landscape     = 0b00101000;
    static constexpr uint8_t portrait_rev  = 0b01011011;
    static constexpr uint8_t landscape_rev = 0b00111011;
    //static constexpr uint8_t landscape_rev = 0b11111011;
    //                                          | | | |
    //                                          | | | 0 - Flip vertical
    //                                          | | 2 - Data latch data order
    //                                          | 4 - Device line refresh order
    //                                          6 - Col address order
    uint8_t param;
    switch (orientation) {
    case Orientation::portrait:
        width_ = panel_x_res_;
        height_ = panel_y_res_;
        param = portrait;
        break;
    case Orientation::landscape:
        width_ = panel_y_res_;
        height_ = panel_x_res_;
        param = landscape;
        break;
    case Orientation::portrait_rev:
        width_ = panel_x_res_;
        height_ = panel_y_res_;
        param = portrait_rev;
        break;
    case Orientation::landscape_rev:
        width_ = panel_y_res_;
        height_ = panel_x_res_;
        param = landscape_rev;
        break;
    }
    writeCmd(MipiDcs::Cmd::set_address_mode, &param, 1);
    // Reset memory window and disable scrolling
    setMemoryWindow(0, 0, width_ - 1, height_ - 1);
    vscroll(0, 0, 0);
}


void MipiDcsTftDriver::vscroll(int16_t start_line, int16_t num_lines, int16_t offs_lines)
{
    // wrap offset if it exceeds the scroll area height
    offs_lines = offs_lines % num_lines;

    if (offs_lines == 0) {
        // Disable scroll
        writeCmd(MipiDcs::Cmd::enter_normal_mode, nullptr, 0);
        return;
    }

    // BFA + VSA + TFA must == y resolution
    int16_t bfa = panel_y_res_ - start_line - num_lines;

    int16_t vsp = start_line + offs_lines; // vertical start position
    if (offs_lines < 0)
        vsp += num_lines;          //keep in unsigned range

    // MSB first
    const uint8_t params[6] = {
            static_cast<uint8_t>(start_line >> 8),
            static_cast<uint8_t>(start_line),
            static_cast<uint8_t>(num_lines >> 8),
            static_cast<uint8_t>(num_lines),
            static_cast<uint8_t>(bfa >> 8),
            static_cast<uint8_t>(bfa)
    };
    writeCmd(MipiDcs::Cmd::set_scroll_area, params, 6);
    // MSB first
    const uint8_t params2[2] = {
            static_cast<uint8_t>(vsp >> 8),
            static_cast<uint8_t>(vsp)
    };
    writeCmd(MipiDcs::Cmd::set_scroll_start, params2, 2);
}

void MipiDcsTftDriver::setInverted(bool inverted)
{
    writeCmd(
            inverted ? MipiDcs::Cmd::enter_invert_mode : MipiDcs::Cmd::exit_invert_mode,
            nullptr, 0);
}

} // namespace Libp
