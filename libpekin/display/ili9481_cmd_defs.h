/**
 * ILI9481 display controller commands and initialization sequence.
 */
#ifndef SRC_TFTLIB_ILI9481_CMD_DEFS_H_
#define SRC_TFTLIB_ILI9481_CMD_DEFS_H_

#include <cstdint>
#include "display/tft_cmd_defs.h"

namespace Libp::Ili9481 {
    inline constexpr uint16_t device_width = 320;
    inline constexpr uint16_t device_height = 480;
    inline constexpr uint16_t device_id = 0x9481;

/**
 * Subset of ILI9481 Native Commands
 */
namespace Cmd {
    /**
     * 6 bytes returned:
     * - 1 Dummy read
     * - 2 MIPI alliance code
     * - 3 MIPI alliance code
     * - 4 Device ID code of ILI9481
     * - 5 Device ID code of ILI9481
     * - 6 Exit code (0xFF)
     * Defaults: 02,04,94,81,FF
     */
    inline constexpr uint8_t device_read_code = 0xBF;
    inline constexpr uint8_t access_protect = 0xB0;
    inline constexpr uint8_t mem_access_and_iface_set = 0xB3;
    inline constexpr uint8_t disp_mode_and_mem_write_mode_set = 0xB4;
    inline constexpr uint8_t panel_driving_set = 0xC0;
    inline constexpr uint8_t display_timing_set_normal_mode = 0xC1;
    inline constexpr uint8_t display_timing_set_partial_mode = 0xC2;
    inline constexpr uint8_t display_timing_set_idle_mode = 0xC3;
    inline constexpr uint8_t frame_rate_and_inversion_control = 0xC5;
    inline constexpr uint8_t iface_ctrl = 0xC6;
    inline constexpr uint8_t gamma_set = 0xC8;
    /// Power setting
    inline constexpr uint8_t power_set = 0xD0;
    inline constexpr uint8_t vcom_ctrl = 0xD1;
    inline constexpr uint8_t power_set_normal_mode = 0xD2;
    inline constexpr uint8_t power_set_partial_mode = 0xD3;
    inline constexpr uint8_t power_set_idle_mode = 0xD4;

 } // namespace Cmd

/*
 * ILI9481 application notes:
 *
 * - Power on reset
 * - Wait 10ms minimum
 * - Set power registers
 *   - BT[2:0] VC[2:0] VRH[3:0], PON, VCIRE
 *   - SELVCM, VCM[5:0], VDV[4:0]
 *   - Apn[2:0], DCnn[2:0]
 * - Set exit sleep mode (0x11)
 * - Wait 80ms minimum
 * - Set display registers
 *   - normal/partial mode
 *   - line/fame inversion
 *   - interface pixel format
 *   - idle mode on/off
 *   - row/column direction
 *   - row/column address
 *   - etc.
 * - Minimum 120ms since power on reset
 * - Write memory data
 * - Set display on (0x29)
 */
/**
 * ILI9481 initialization command sequence.
 */
inline constexpr CmdSequence init_cmd_seq = {
        12, // max params (gamma)

        // Wait 10ms in case we're following an immediate hard reset
        cust_cmd_delay,
            10,

        // *** Set power registers ***

        // Enable manufacturer commands B1->DF, E0->EF, F0->FF (PWON default)
        Cmd::access_protect,
            1,
            0x00,
        /*
         *
         * Voltages (PWON defaults in parenthesis)
         * ---------------------------------------
         * Vci                         =  3.3V [3.3V max]
         * Vci1     =  Vci  * 1 (0.95) =  3.3V
         * VGH      =  Vci1 * 6 (5)    = 19.8V [ 18.0V max]
         * VGL      = -Vci1 * 3 (5)    = -9.9V [-12.5V max]
         * DDVDH    =  Vci1 * 2 (2)    =  6.6V [  6.0V max]
         * VCL      = -Vci1 * 1 (1)    = -3.3V [ -3.0V max]
         * VCIRE    = internal ref.    =  2.5V
         * VREG1OUT = 2.5V x 1.60      =  4.0V [VREG1OUT <= (DDVDH - 0.25)V.]
         */
        Cmd::power_set,
            3,
            0x07, 0x42, 0x18,
        /*
         *
         * VCOMH      = VREG1OUT * 0.720 (0.685) = 1.80V
         * VCOM ampl. = VREG1OUT * 1.020 (0.700) = 2.55V
         */
        Cmd::vcom_ctrl,
            3,
            0x00, 0x07, 0x10,
        /*
         *
         * Gamma driver amp. current  = 1.0 (1.0)
         * Source driver amp. current = 1.0 (1.0)
         *
         * Charge-pump freq. of circuit1/2:
         * fDCDC2                     = Fosc/16 (Fosc/64)
         * fDCDC1                     = Fosc/4  (Fosc/4)
         */
        Cmd::power_set_normal_mode,
            2,
            0x01, 0x02,
        /*
         *
         * Gamma driver amp. current  = 1.0 (1.0)
         * Source driver amp. current = 1.0 (1.0)
         *
         * Charge-pump freq. of circuit1/2:
         * fDCDC2                     = Fosc/16 (Fosc/64)
         * fDCDC1                     = Fosc/4  (Fosc/4)
         */
        Cmd::power_set_partial_mode,
            2,
            0x01, 0x02,
        /*
         *
         * Gamma driver amp. current  = 1.0 (1.0)
         * Source driver amp. current = 1.0 (1.0)
         *
         * Charge-pump freq. of circuit1/2:
         * fDCDC2                     = Fosc/16 (Fosc/64)
         * fDCDC1                     = Fosc/4  (Fosc/4)
         */
        Cmd::power_set_idle_mode,
            2,
            0x01, 0x02,
        /*
         *
         */
        MipiDcs::Cmd::exit_sleep_mode, 0,
        /*
         *
         */
        cust_cmd_delay, 80,

        // *** Set display registers ***

        /*
         *
         * Memory write control      = wrap on overflow (default)
         * TE signal output interval = 1 frame (default)
         * GRAM write cycle period   = 1 frame (default)
         * GRAM 16->18bit data fmt   = 0 used for LSB (default)
         * DFM ?                     = (default)
         */
        Cmd::mem_access_and_iface_set,
            4,
            0x02, 0x00, 0x00, 0x00,
        /*
         *
         * GRAM interface   = DBI/system interface (default)
         * Display op. mode = internal system clk. (default)
         *
         */
        Cmd::disp_mode_and_mem_write_mode_set,
            1,
            0x00,
        /*
         *
         * Grayscale inv. (REV)        = 1 (default)
         * Gate output seq. (SM/GS)    = 0/0 (default)  = G1..G480
         * Number of lines (NL)        = 0x3B (default) = 480 lines
         * Scanning start pos. (SCN)   = 0 (default)    = G1
         * Non-display area:
         * - source output level (NDL) = 0 (default)
         * - drive period (PTS)        = 2 (default)
         * - scan mode (PTG)           = 1 (default)
         * - scan cycle (ISC)          = 1 (default)
         */
        Cmd::panel_driving_set,
            5,
            0x12/* TODO: low nibble 2 is invalid */, 0x3B, 0x00, 0x02, 0x11,
        /*
         *
         * VCOM waveform (BC0)       = 1 (default)    = frame inversion
         * Clk division ratio (DIV0) = 0 (default)    = 1/1
         * Line period (RTN0)        = 0x10 (default) = 16 clocks
         * Front porch period (FP0)  = 8 (default)    = 8 lines
         * Back porch period (BP0)   = 8 (default)    = 8 lines
         */
        // TODO: why not partial/idle mode settings too?? Why set this at all given all defaults
        Cmd::display_timing_set_normal_mode,
            3,
            0x10, 0x10, 0x88,
        /*
         *
         * Frame frequency (FRA) = 3 (default) = 72 Hz
         */
        Cmd::frame_rate_and_inversion_control,
            1,
            0x03,
        /*
         *
         * DBI type C iface (SDA_EN) = 0 (default) = use DIN/DOUT pins
         * VSYNC pin polarity (VSPL) = 0 (default) = low active
         * HSYNC pin polarity (HSPL) = 0 (default) = low active
         * ENABLE pin polarity (EPL) = 1 (default) = data written on enable=1
         * PCLK pin polarity (DPL)   = 0 (default) = input on rising edge
         */
        Cmd::iface_ctrl,
            1,
            0x02,
        /*
         *
         * Gamma settings (PWON default = 0 for all)
         * -----------------------------------------
         * Fine adjust for pos. polarity (KP0/1/2/3/4/5) = 0/0/2/3/6/3
         * Gradient adjust for pos. polarity (RP0/1)     = 5/4
         * Amplitude adjust for pos. polarity (VRP0/1)   = 6/0x16
         * Fine adjust for neg. polarity (KN0/1/2/3/4/5) = 7/3/5/7/7/7
         * Gradient adjust for neg. polarity (RN0/1)     = 4/5
         * Amplitude adjust for neg. polarity (VRN0/1)   = 0xC/0
         */
        Cmd::gamma_set,
            12,
            0x00, 0x32, 0x36, 0x45, 0x06, 0x16, 0x37, 0x75, 0x77, 0x54, 0x0C, 0x00,
        /*
         *
         * Pixel format
         */
        MipiDcs::Cmd::pixel_format,
            1,
            MipiDcs::PixelFmt::dcs_16bpp<<4 | MipiDcs::PixelFmt::dcs_16bpp,

        cust_cmd_delay,
            (120 - 80 - 10),

        MipiDcs::Cmd::set_display_on,
            0,

};

} // namespace Libp::Ili9481

#endif /* SRC_TFTLIB_ILI9481_CMD_DEFS_H_ */

