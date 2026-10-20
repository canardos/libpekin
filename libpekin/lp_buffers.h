#ifndef LIB_LIBPEKIN_LP_BUFFERS_H_
#define LIB_LIBPEKIN_LP_BUFFERS_H_

#include <cstdint>

namespace libp {

/**
 * Reads a 16-bit unsigned integer in big-endian format.
 *
 * @param buf must be at least 2-bytes long
 */
[[gnu::always_inline]] inline
constexpr uint16_t readBEndU16(const uint8_t* buf)
{
    return (static_cast<uint16_t>(buf[0]) << 8) |
            static_cast<uint16_t>(buf[1]);
}

/**
 * Reads a 32-bit unsigned integer in big-endian format.
 *
 * @param buf must be at least 4-bytes long
 */
[[gnu::always_inline]] inline
constexpr uint32_t readBEndU32(const uint8_t* buf)
{
    return (static_cast<uint32_t>(buf[0]) << 24) |
           (static_cast<uint32_t>(buf[1]) << 16) |
           (static_cast<uint32_t>(buf[2]) <<  8) |
            static_cast<uint32_t>(buf[3]);
}

/**
 * Reads a 16-bit unsigned integer in little-endian format.
 *
 * @param buf must be at least 2-bytes long
 */
[[gnu::always_inline]] inline
constexpr uint16_t readLEndU16(const uint8_t* buf)
{
    return (static_cast<uint16_t>(buf[1]) << 8) |
            static_cast<uint16_t>(buf[0]);
}

/**
 * Reads a 32-bit unsigned integer in little-endian format.
 *
 * @param buf must be at least 4-bytes long
 */
[[gnu::always_inline]] inline
constexpr uint32_t readLEndU32(const uint8_t* buf)
{
    return (static_cast<uint32_t>(buf[3]) << 24) |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[1]) <<  8) |
            static_cast<uint32_t>(buf[0]);
}

/**
 * Writes a 16-bit unsigned integer in big-endian format.
 *
 * @param buf Output buffer (must be at least 2-bytes long)
 */
[[gnu::always_inline]] inline
constexpr void writeBEndU16(uint8_t* buf, uint16_t val)
{
    buf[0] = val >>  8 & 0xff;
    buf[1] = val >>  0 & 0xff;
}

/**
 * Writes a 32-bit unsigned integer in big-endian format.
 *
 * @param buf Output buffer (must be at least 4-bytes long)
 */
[[gnu::always_inline]] inline
constexpr void writeBEndU32(uint8_t* buf, uint32_t val)
{
    buf[0] = val >> 24 & 0xff;
    buf[1] = val >> 16 & 0xff;
    buf[2] = val >>  8 & 0xff;
    buf[3] = val >>  0 & 0xff;
}

/**
 * Writes a 16-bit unsigned integer in little-endian format.
 *
 * @param buf Output buffer (must be at least 2-bytes long)
 */
[[gnu::always_inline]] inline
constexpr void writeLEndU16(uint8_t* buf, uint16_t val)
{
    buf[0] = val >>  0 & 0xff;
    buf[1] = val >>  8 & 0xff;
}

/**
 * Writes a 32-bit unsigned integer in little-endian format.
 *
 * @param buf Output buffer (must be at least 4-bytes long)
 */
[[gnu::always_inline]] inline
constexpr void writeLEndU32(uint8_t* buf, uint32_t val)
{
    buf[0] = val >>  0 & 0xff;
    buf[1] = val >>  8 & 0xff;
    buf[2] = val >> 16 & 0xff;
    buf[3] = val >> 24 & 0xff;
}


} // namespace libp

#endif /* LIB_LIBPEKIN_LP_BUFFERS_H_ */
