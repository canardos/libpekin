#ifndef UTIL_LP_SIMPLE_QUEUE_H_
#define UTIL_LP_SIMPLE_QUEUE_H_

#include <lp_ring_buffer.h>
#include <cstdint>

namespace libp {

inline int do_nothing_enter() { return 0; }
inline void do_nothing_exit([[maybe_unused]] int data) {}


/**
 * Simple send/receive fixed length queue.
 *
 * send/receive are not reentrant safe unless critical section guard functions
 * are provided.
 *
 * The function signatures are as follows:
 * - U critical_enter()
 * - dont_care_ret critical_exit(U state)
 *
 * Intended only for simple POD objects. Array of objects is fully allocated/
 * constructed on construction.
 *
 * REMINDER: Try to keep length the same across queues of the same type to
 * reduce template instantiation bloat.
 *
 * @tparam T type of queue element
 * @tparam length queue length (available number of elements is length - 1)
 * @tparam critical_enter critical section entry function returning type U
 * @tparam critical_exit  critical section exit function accepting type U param
 */
template <
    typename T,
    uint16_t length,
    auto critical_enter = do_nothing_enter,
    auto critical_exit  = do_nothing_exit>
        requires requires { critical_exit(critical_enter()); }
class SimpleQueue {
    template <auto F, auto G>
    static constexpr bool same_function()
    {
        if constexpr (std::is_same_v<decltype(F), decltype(G)>) {
            return F == G;
        } else {
            return false;
        }
    }
public:
    static_assert( ( same_function<critical_enter, do_nothing_enter>() && same_function<critical_exit, do_nothing_exit>()) ||
                   (!same_function<critical_enter, do_nothing_enter>() && !same_function<critical_exit, do_nothing_exit>()),
                   "Must provide both critical_enter and critical_exit or neither");

    bool send(const T& element)
    {
        auto state = critical_enter();
        if (buffer.isFull()) {
            critical_exit(state);
            return false;
        }
        buffer.write(element);
        critical_exit(state);
        return true;
    }

    bool send(T&& element)
    {
        auto state = critical_enter();
        if (buffer.isFull()) {
            critical_exit(state);
            return false;
        }
        buffer.write(std::move(element));
        critical_exit(state);
        return true;
    }

    bool receive(T *element)
    {
        auto state = critical_enter();
        if (buffer.isEmpty()) {
            critical_exit(state);
            return false;
        }
        *element = buffer.read();
        critical_exit(state);
        return true;
    }

    [[nodiscard]] bool isEmpty() const
    {
      return buffer.isEmpty();
    }

    [[nodiscard]] uint16_t size() const
    {
        return buffer.size();
    }

    void flush()
    {
        auto state = critical_enter();
        buffer.clear();
        critical_exit(state);
    }

    /*T& front()
    {
        return buffer.peek();
    }
    const T& front() const
    {
      return buffer.peek();
    }
    void pop()
    {
        buffer.read();
    }*/

private:
    RingBuffer<T, length> buffer;
};

} // namespace libp

#endif /* UTIL_LP_SIMPLE_QUEUE_H_ */
