#include <cstdint>

namespace Libp {

inline
static void writeRgb(uint8_t (&color)[3], uint8_t out[])
{
    out[0] = color[0];
    out[1] = color[1];
    out[2] = color[2];
}

/// MSB first
void upsample1bppIndexed(uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[2][3])
{
    uint32_t src_len = width * height / 8;
    uint32_t src_idx = 0;
    uint32_t dst_idx = 0;
    while (src_idx < src_len) {
        uint8_t val = in[src_idx++];
        uint8_t color_idx[8];
        color_idx[0] =  val >> 7;
        color_idx[1] = (val & 0b01000000) >> 6;
        color_idx[2] = (val & 0b00100000) >> 5;
        color_idx[3] = (val & 0b00010000) >> 4;
        color_idx[4] = (val & 0b00001000) >> 3;
        color_idx[5] = (val & 0b00000100) >> 2;
        color_idx[6] = (val & 0b00000010) >> 1;
        color_idx[7] =  val & 0b00000001;

        for (uint8_t i = 0; i < 8; i++) {
            writeRgb(colors[color_idx[i]], &out[dst_idx]);
            dst_idx += 3;
        }
    }
}

/// MSB first
void upsample2bppIndexed(uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[4][3])
{
    uint32_t src_len = width * height / 4;
    uint32_t src_idx = 0;
    uint32_t dst_idx = 0;
    while (src_idx < src_len) {
        uint8_t val = in[src_idx++];
        uint8_t color_idx[4];
        color_idx[0] =  val >> 6;
        color_idx[1] = (val & 0b00110000) >> 4;
        color_idx[2] = (val & 0b00001100) >> 2;
        color_idx[3] =  val & 0b00000011;

        for (uint8_t i = 0; i < 4; i++) {
            writeRgb(colors[color_idx[i]], &out[dst_idx]);
            dst_idx += 3;
        }
    }
}

/// MSB first
void upsample3bppIndexed(uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[8][3])
{
    uint32_t src_len = width * height * 3 / 8;
    uint32_t src_idx = 0;
    uint32_t dst_idx = 0;
    while (src_idx < src_len) {

        uint32_t val = in[src_idx] << 16 | in[src_idx+1] << 8 | in[src_idx+2];
        src_idx += 3;

        uint8_t color_idx[8];
        color_idx[0] =  val >> 21;
        color_idx[1] = (val & 0b00011100'00000000'00000000) >> 18;
        color_idx[2] = (val & 0b00000011'10000000'00000000) >> 15;
        color_idx[3] = (val & 0b00000000'01110000'00000000) >> 12;
        color_idx[4] = (val & 0b00000000'00001110'00000000) >> 9;
        color_idx[5] = (val & 0b00000000'00000001'11000000) >> 6;
        color_idx[6] = (val & 0b00000000'00000000'00111000) >> 3;
        color_idx[7] =  val & 0b00000000'00000000'00000111;

        for (uint8_t i = 0; i < 8; i++) {
            writeRgb(colors[color_idx[i]], &out[dst_idx]);
            dst_idx += 3;
        }
    }
}

/// MSB first
void upsample4bppIndexed(uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[16][3])
{
    uint32_t src_len = width * height / 2; // document
    uint32_t src_idx = 0;
    uint32_t dst_idx = 0;
    while (src_idx < src_len) {
        uint8_t val = in[src_idx++];
        uint8_t color_idx[2];
        color_idx[0] = val >> 4;
        color_idx[1] = val & 0b00001111;

        for (uint8_t i = 0; i < 2; i++) {
            writeRgb(colors[color_idx[i]], &out[dst_idx]);
            dst_idx += 3;
        }
    }
}

/// MSB first
void upsample6bppIndexed(uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[64][3])
{
    uint32_t src_len = width * height * 3 / 4;
    uint32_t src_idx = 0;
    uint32_t dst_idx = 0;
    while (src_idx < src_len) {

        uint32_t val = in[src_idx] << 16 | in[src_idx+1] << 8 | in[src_idx+2];
        src_idx += 3;

        uint8_t color_idx[4];
        color_idx[0] =  val >> 18;
        color_idx[1] = (val & 0b00000011'11110000'00000000) >> 12;
        color_idx[2] = (val & 0b00000000'00001111'11000000) >> 6;
        color_idx[3] =  val & 0b00000000'00000000'00111111;

        for (uint8_t i = 0; i < 4; i++) {
            writeRgb(colors[color_idx[i]], &out[dst_idx]);
            dst_idx += 3;
        }
    }
}

} // namespace Libp
