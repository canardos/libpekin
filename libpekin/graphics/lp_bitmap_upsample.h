/**
 * Functions to upsample 1/2/3/4/8bpp packed images to 24-bit RGB color.
 *
 * e.g. 4 x 2 1-bit image -> black and red 24-bit image
 *
 *     uint8_t in[] = { 0b11110000 };
 *     uint8_t out[4 * 2 * 3];
 *     uint8_t colors[2][3] = { { 0x00, 0x00, 0x00 }, { 0xff, 0x00, 0x00 } };
 *
 *     upsample1bppIndexed(in, out, 4, 2, colors);
 *
 *     // out = { 0xff,0x00,0x00 , 0xff,0x00,0x00 , 0xff,0x00,0x00 , 0xff,0x00,0x00
 *     //         0x00,0x00,0x00 , 0x00,0x00,0x00 , 0x00,0x00,0x00 , 0x00,0x00,0x00 }
 */
#ifndef BITMAP_UPSAMPLE_H_
#define BITMAP_UPSAMPLE_H_

#include <cstdint>

namespace libp {

/**
 * Covert packed 1-bit indexed image to RGBA
 *
 * MSB first (i.e. byte 0, bit 7 = first pixel)
 *
 * `(width * height) % 8 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 2 32-bit RGBA color values
 */
void upsample1bppIndexedRGBA(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, const uint8_t (&colors)[2][4]);

/**
 * Covert packed 1-bit indexed image to RGB24.
 *
 * MSB first (i.e. byte 0, bit 7 = first pixel)
 *
 * `(width * height) % 8 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 2 24-bit RGB color values
 */
void upsample1bppIndexed(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[2][3]);


/**
 * Covert packed 2-bit indexed image to RGBA
 *
 * MSB first (i.e. byte 0, bits 7,6 = first pixel)
 *
 * `(width * height) % 4 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 4 32-bit RGBA color values
 */
void upsample2bppIndexedRGBA(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, const uint8_t (&colors)[4][4]);


/**
 * Covert packed 2-bit indexed image to RGB24
 *
 * MSB first (i.e. byte 0, bits 7,6 = first pixel)
 *
 * `(width * height) % 4 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 4 24-bit RGB color values
 */
void upsample2bppIndexed(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[4][3]);


/**
 * Covert packed 3-bit indexed image to RGB24
 *
 * MSB first (i.e. byte 0, bits 7,6,5 = first pixel)
 *
 * `(width * height) % 3 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 8 24-bit RGB color values
 */
void upsample3bppIndexed(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[8][3]);


/**
 * Covert packed 4-bit indexed image to RGB24
 *
 * MSB first (i.e. byte 0, bits 7,6,5,4 = first pixel)
 *
 * `(width * height) % 2 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 16 24-bit RGB color values
 */
void upsample4bppIndexed(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[16][3]);

/**
 * Covert packed 6-bit indexed image to RGB24
 *
 * MSB first (i.e. byte 0, bits 7,6,5,4,3,2 = first pixel)
 *
 * `(width * height) % 4 == 0`
 *
 * @param in
 * @param out
 * @param width
 * @param height
 * @param colors array of 64 24-bit RGB color values
 */
void upsample6bppIndexed(const uint8_t* in, uint8_t* out, uint16_t width, uint16_t height, uint8_t (&colors)[64][3]);

} // namespace libp

#endif /* BITMAP_UPSAMPLE_H_ */
