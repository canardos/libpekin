#ifndef SRC_TFTLIB_MIPI_DCS_TFT_DRIVER_H_
#define SRC_TFTLIB_MIPI_DCS_TFT_DRIVER_H_

#include <cstdint>

#include "bus/i_basic_bus_16.h"
#include "display/tft_cmd_defs.h"
#include "graphics/idrawing_surface.h"

namespace Libp {

/**
 * IDrawingSurface implementation for devices conforming to the MIPI DCS V1
 * specification.
 *
 * Currently only supports 16-bits/pixel format.
 *
 * Tested on:
 * - ILI9481 with STM32F1 8-bit GPIO and 16-bit FSMC buses.
 */
class MipiDcsTftDriver : public IDrawingSurface<uint16_t> {
public:

    /**
     * Construct a MPIP DCS driver object. No initialization is performed by
     * this constructor. `init` must be called to initialize the attached
     * device.
     *
     * @param lcd_bus bus object to communicate with the display.
     * @param x_res native x resolution of the display (independent of
     *              orientation)
     * @param y_res native y resolution of the display (independent of
     *              orientation)
     */
    MipiDcsTftDriver(IBasicBus16& bus, uint16_t x_res, uint16_t y_res);
    virtual ~MipiDcsTftDriver() { }

    /**
     * Initialize the display. This function must be called prior to any other
     * functions. Most display devices will require a hardware reset prior to
     * initialization.
     *
     * The initialization sequence is:
     * - Execute provided command sequence
     * - Set orientation
     * - Set inversion
     *
     * The command sequence should be used to configure any settings required
     * for operation (e.g. power control, gamma, bus/memory settings).
     *
     * @param init_commands command sequence to send to the device after
     *                      resetting and prior to setting
     *                      orientation/inversion.
     * @param orientation the device orientation (i.e. which side is "up")
     * @param inverted if true, colors will be inverted.
     */
    void init(const CmdSequence init_commands, uint16_t init_commands_len,
            Orientation orientation, bool inverted);

    /**
     * Vertically scrolls a horizontal band of the display.
     *
     * The vertical scroll direction is with respect to the display's native
     * orientation.
     *
     * @param start_line Start of the scroll area in number of lines from the
     *                   top of the display.
     * @param num_lines  Height of the scroll area.
     * @param offs_lines Number of lines to shift the scroll area. May be
     *                   negative.
     */
    void vscroll(int16_t start_line, int16_t num_lines, int16_t offs_lines);

    // --- IDrawingSurface functions ---

    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) override;
    void copyRect(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint16_t* color) override;
    void fillScreen(uint16_t color) override;
    void setOrientation(Orientation orientation) override;
    void setInverted(bool inverted) override;


    void setPixel(uint16_t x, uint16_t y, uint16_t color) override;
    void fillLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) override;
    uint16_t getWidth() override
    {
        return width_;
    }
    uint16_t getHeight() override
    {
        return height_;
    }


    //uint16_t getWidth()  override { return width_;  };
    //uint16_t getHeight() override { return height_; };

private:
    IBasicBus16& bus_;

    // Raw unchanging native width/height
    const int16_t panel_x_res_;
    const int16_t panel_y_res_;

    // Width/height post orientation changes
    uint16_t width_;
    uint16_t height_;

    void writeCmd(uint8_t cmd);

    void writeCmd(uint8_t cmd, const uint8_t * params, int8_t n_param);

    void execCmdSequence(const CmdSequence sequence, int16_t len);

    /**
     * Set the output window for write operations.
     *
     * @param x0 inclusive
     * @param y0 inclusive
     * @param x1 inclusive
     * @param y1 inclusive
     */
    void setMemoryWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

};

} // namespace Libp

#endif /* SRC_TFTLIB_MIPI_DCS_TFT_DRIVER_H_ */
