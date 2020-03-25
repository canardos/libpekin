#ifndef LIB_LIBPEKIN_BITS_H_
#define LIB_LIBPEKIN_BITS_H_

#include <cstdint>

namespace Libp::Bits {

// 8-bit volatile

inline __attribute__((always_inline))
void setBit(volatile uint8_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
inline __attribute__((always_inline))
void clearBit(volatile uint8_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
inline __attribute__((always_inline))
void updBit(volatile uint8_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
inline __attribute__((always_inline))
void setBits(volatile uint8_t& reg, uint8_t mask, uint8_t value)
{
    reg = (reg & ~mask) | value;
}
inline __attribute__((always_inline))
void setMask(volatile uint8_t& reg, uint8_t mask)
{
    reg |= mask;
}
inline __attribute__((always_inline))
void clearMask(volatile uint8_t& reg, uint8_t mask)
{
    reg &= ~mask;
}

// 8-bit

constexpr
inline  __attribute__((always_inline))
void setBit(uint8_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
constexpr
inline  __attribute__((always_inline))
void clearBit(uint8_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
constexpr
inline __attribute__((always_inline))
void updBit(uint8_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
constexpr
inline  __attribute__((always_inline))
void setBits(uint8_t& reg, uint8_t mask, uint8_t value)
{
    reg = (reg & ~mask) | value;
}
constexpr
inline __attribute__((always_inline))
void setMask(uint8_t& reg, uint8_t mask)
{
    reg |= mask;
}
constexpr
inline  __attribute__((always_inline))
void clearMask(uint8_t& reg, uint8_t mask)
{
    reg &= ~mask;
}

// 16-bit volatile

inline __attribute__((always_inline))
void setBit(volatile uint16_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
inline __attribute__((always_inline))
void clearBit(volatile uint16_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
inline __attribute__((always_inline))
void updBit(volatile uint16_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
inline __attribute__((always_inline))
void setBits(volatile uint16_t& reg, uint16_t mask, uint16_t value)
{
    reg = (reg & ~mask) | value;
}
inline __attribute__((always_inline))
void setMask(volatile uint16_t& reg, uint16_t mask)
{
    reg |= mask;
}
inline __attribute__((always_inline))
void clearMask(volatile uint16_t& reg, uint16_t mask)
{
    reg &= ~mask;
}

// 16-bit volatile

constexpr
inline __attribute__((always_inline))
void setBit(uint16_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
constexpr
inline __attribute__((always_inline))
void clearBit(uint16_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
constexpr
inline __attribute__((always_inline))
void updBit(uint16_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
constexpr
inline __attribute__((always_inline))
void setBits(uint16_t& reg, uint16_t mask, uint16_t value)
{
    reg = (reg & ~mask) | value;
}
constexpr
inline __attribute__((always_inline))
void setMask(uint16_t& reg, uint16_t mask)
{
    reg |= mask;
}
constexpr
inline __attribute__((always_inline))
void clearMask(uint16_t& reg, uint16_t mask)
{
    reg &= ~mask;
}

// 32-bit volatile

inline __attribute__((always_inline))
void setBit(volatile uint32_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
inline __attribute__((always_inline))
void clearBit(volatile uint32_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
inline __attribute__((always_inline))
void updBit(volatile uint32_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
inline __attribute__((always_inline))
void setBits(volatile uint32_t& reg, uint32_t mask, uint32_t value)
{
    reg = (reg & ~mask) | value;
}
inline __attribute__((always_inline))
void setMask(volatile uint32_t& reg, uint32_t mask)
{
    reg |= mask;
}
inline __attribute__((always_inline))
void clearMask(volatile uint32_t& reg, uint32_t mask)
{
    reg &= ~mask;
}

// 32-bit volatile

constexpr
inline __attribute__((always_inline))
void setBit(uint32_t& reg, uint8_t bit_pos)
{
    reg |= 1 << bit_pos;
}
constexpr
inline __attribute__((always_inline))
void clearBit(uint32_t& reg, uint8_t bit_pos)
{
    reg &= ~(1 << bit_pos);
}
constexpr
inline __attribute__((always_inline))
void updBit(uint32_t& reg, uint8_t bit_pos, bool set)
{
    reg = (reg & ~(1 << bit_pos)) | (set << bit_pos);
}
constexpr
inline __attribute__((always_inline))
void setBits(uint32_t& reg, uint32_t mask, uint32_t value)
{
    reg = (reg & ~mask) | value;
}
constexpr
inline __attribute__((always_inline))
void setMask(uint32_t& reg, uint32_t mask)
{
    reg |= mask;
}
constexpr
inline __attribute__((always_inline))
void clearMask(uint32_t& reg, uint32_t mask)
{
    reg &= ~mask;
}

} // namespace Libp::Bits

#endif /* LIB_LIBPEKIN_BITS_H_ */
