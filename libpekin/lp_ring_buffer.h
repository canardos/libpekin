#ifndef LIBPEKIN_RING_BUFFER_H_
#define LIBPEKIN_RING_BUFFER_H_

#include "lp_misc_math.h"
#include <cstdint>
#include <utility>

namespace libp {

/**
 * Basic single producer/consumer ring buffer.
 *
 * read/write/clear are not reentrant safe.
 *
 * Intended only for simple POD objects. Array of objects is fully allocated/
 * constructed on construction.
 *
 * REMINDER: Try to keep num_elements the same across buffers of the same T to
 * reduce template instantiation bloat.
 *
 * @tparam T type of elements to store
 * @tparam elements must be a power of 2 and >=2
 */
template <typename T, uint16_t num_elements>
struct RingBuffer {
    static_assert(isPowerOf2(num_elements), "num_elements must be a power of 2");
    static_assert(num_elements >= 2, "num_elements must be >= 2");

    /**
     * Oldest item dropped if the buffer is full.
     */
    void write(const T& element)
    {
        if (isFull()) {
            read_idx = read_idx + 1;
        }
        data[mask(write_idx)] = element;
        write_idx = write_idx + 1;
    }

    /**
     * Oldest item dropped if the buffer is full.
     */
    void write(T&& element)
    {
        if (isFull()) {
            read_idx = read_idx + 1;
        }
        data[mask(write_idx)] = std::move(element);
        write_idx = write_idx + 1;
    }

    T read()
    {
        T element = std::move(data[mask(read_idx)]);
        read_idx = read_idx + 1;
        return element;
    }

    /**
     * Return element at position write head - offset.
     *
     * Any value of offset is "safe", but data returned will be invalid if
     * offset >= size()
     *
     * @param offset negative offset from the write head.
     *
     * @return const element reference
     */
    const T& peek(uint16_t offset) const
    {
        return data[mask(write_idx - 1 - offset)];
    }

    [[nodiscard]] bool isFull() const
    {
        return size() == num_elements;
    }

    [[nodiscard]] bool isEmpty() const
    {
        return write_idx == read_idx;
    }

    void clear()
    {
        write_idx = 0;
        read_idx = 0;
    }

    [[nodiscard]] uint16_t size() const
    {
        return static_cast<uint16_t>(write_idx - read_idx);
    }

private:
    volatile uint16_t write_idx = 0;
    volatile uint16_t read_idx = 0;
    T data[num_elements];

    constexpr uint16_t mask(uint16_t val) const
    {
        return val & (num_elements - 1);
    }
};

} // namespace libp

#endif /* LIBPEKIN_RING_BUFFER_H_ */
