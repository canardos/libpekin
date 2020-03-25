#ifndef LIB_LIBPEKIN_BYTES_H_
#define LIB_LIBPEKIN_BYTES_H_

#include <cstdint>

namespace Libp {

/**
 * Return the most significant 8-bits of a 16-bit value.
 *
 * @param val
 * @return the most significant 8-bits
 */
inline __attribute__((always_inline))
constexpr uint8_t highByte(uint16_t val) { return static_cast<uint8_t>(val >> 8); }

/**
 * Return the most significant 8-bits of a 32-bit value.
 *
 * @param val
 * @return the most significant 8-bits
 */
inline __attribute__((always_inline))
constexpr uint8_t highByte(uint32_t val) { return static_cast<uint8_t>(val >> 24); }

/**
 * Return the least significant 8-bits of a 16-bit value.
 *
 * @param val
 * @return the most significant 8-bits
 */
inline __attribute__((always_inline))
constexpr uint8_t lowByte(uint16_t val) { return static_cast<uint8_t>(val & 0xff); }

/**
 * Return the least significant 8-bits of a 32-bit value.
 *
 * @param val
 * @return the most significant 8-bits
 */
inline __attribute__((always_inline))
constexpr uint8_t lowByte(uint32_t val) { return static_cast<uint8_t>(val & 0xff); }

} // namespace Libp

#endif /* LIB_LIBPEKIN_BYTES_H_ */
