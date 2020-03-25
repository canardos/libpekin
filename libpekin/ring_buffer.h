#ifndef LIB_LIBPEKIN_RING_BUFFER_H_
#define LIB_LIBPEKIN_RING_BUFFER_H_

#include "misc_math.h"

namespace Libp {

/**
 * Basic single producer/consumer byte ring buffer.
 *
 * @tparam elements must be a power of 2 and >=2
 */
template <uint16_t elements>
struct ByteRingBuffer {
    static_assert(isPowerOf2(elements), "Elements template param must be a power of 2");
    static_assert(elements >= 2, "Elements template param must be >= 2");
    volatile uint16_t write_idx = 0;
    volatile uint16_t read_idx = 0;
    uint8_t data[elements];
    inline void write(uint8_t ch)
    {
        data[write_idx] = ch;
        write_idx = mask(write_idx + 1);
    }
    inline uint8_t read()
    {
        uint8_t ch = data[read_idx];
        read_idx = mask(read_idx + 1);
        return ch;
    }
    bool isFull()
    {
        return mask(write_idx + 1) == read_idx;
    }
    bool isEmpty()
    {
        return write_idx == read_idx;
    }
    /// Don't read/write until return
    /*void clear()
    {
        write_idx = 0;
        read_idx = 0;
    }*/
    uint16_t size()
    {
        return mask(write_idx - read_idx);
    }
private:
    constexpr uint16_t mask(uint16_t val)
    {
        return val & (elements - 1);
    }
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_RING_BUFFER_H_ */
