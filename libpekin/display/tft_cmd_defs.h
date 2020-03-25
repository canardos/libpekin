#ifndef SRC_TFTLIB_TFT_CMD_DEFS_H_
#define SRC_TFTLIB_TFT_CMD_DEFS_H_

#include <cstdint>

namespace Libp {

/**
 * Sequence of device commands in the following format:
 *
 * - uint8_t : Maximum number of parameters for any single command in this data
 *             structure.
 * - An array of one or more command/parameter sets with each set in the
 *   following format:
 *   - uint8_t   8-bit command
 *   - uint8_t   number of parameters for this command (may be zero)
 *   - uint8_t[] 0 or more parameters
 */
typedef uint8_t CmdSequence[];

/**
 * Command code for a fixed delay.
 * Single unsigned byte follows giving ms to delay.
 */
inline constexpr uint8_t cust_cmd_delay = 0xFF;

/**
 * Subset of the MIPI DCS V1 Command Set
 */
namespace MipiDcs {

    namespace Cmd {
        inline constexpr uint8_t soft_reset = 0x01;
        inline constexpr uint8_t exit_sleep_mode = 0x11;

        /// Partial and scroll mode off
        inline constexpr uint8_t enter_normal_mode = 0x13;
        inline constexpr uint8_t set_display_off = 0x28;
        inline constexpr uint8_t exit_invert_mode = 0x20;
        inline constexpr uint8_t enter_invert_mode = 0x21;
        inline constexpr uint8_t set_display_on = 0x29;

        /// 4 bytes:
        /// - bytes 0-1: SC
        /// - bytes 2-3: EC
        inline constexpr uint8_t set_column_address = 0x2A;
        inline constexpr uint8_t set_page_address = 0x2B;

        /// Transfer data to location specified by set_column/page_adress.
        /// This command should follow a set_column_address and
        /// set_page_address commands.
        /// Data following the command is assumed to be pixel data.
        inline constexpr uint8_t write_memory_start = 0x2C;

        /// Define the vertical scrolling area
        /// 6 bytes:
        /// - bytes 0-1: Top Fixed Area (TFA)
        /// - bytes 2-3: Height of Vertical Scrolling Area (VSA)
        /// - bytes 4-5: Bottom Fixed Area (BFA)
        ///
        /// All values are in number of lines from the bottom of frame memory.
        /// Valid values are 0x0000 -> 0x1E0
        /// TFA + VSA + BFA = y resolution
        inline constexpr uint8_t set_scroll_area = 0x33;

        /// Set the data order for transfers to frame memory (i.e. top/bottom
        /// and left/right inversion and portrait/landscape rotation)
        /// 1 byte:
        ///
        /// Bits[7:5] Host<->frame memory transfer order
        /// - bit 7: Page address order (0=top->bottom, 1=bottom->top)
        /// - bit 6: Column address order (0-left->right, 1=right->left)
        /// - bit 5: Page/column addressing order (0=normal, 1=reversed)
        ///
        /// Bits[4:0] Frame memory<->display transfer order
        /// - bit 4: Display device line refresh order (0=top->bottom line,
        ///          1=bottom->top line)
        /// - bit 3: RGB/BGR order (0=RGB, 1=BGR)
        /// - bit 2: Display data latch  data order (0=left->right,
        ///          1=right->left)
        /// - bit 1: Flip horizontal (0=normal, 1=flipped)
        /// - bit 0: Flip vertical (0=normal, 1=flipped)
        inline constexpr uint8_t set_address_mode = 0x36;

        /// Set start of scroll area in frame mem.
        /// 2 bytes: First line of vertical scroll area
        inline constexpr uint8_t set_scroll_start = 0x37;

        inline constexpr uint8_t pixel_format = 0x3A; // [6:4]DPI / [2:0]DBI
    }

    namespace PixelFmt {
        inline constexpr uint8_t dcs_3bpp = 1;
        inline constexpr uint8_t dcs_8bpp = 2;
        inline constexpr uint8_t dcs_12bpp = 3;
        inline constexpr uint8_t dcs_16bpp = 5;
        inline constexpr uint8_t dcs_18bpp = 6;
        inline constexpr uint8_t dcs_24bpp = 7;
    }
}

} // namespace Libp

#endif /* SRC_TFTLIB_TFT_CMD_DEFS_H_ */
