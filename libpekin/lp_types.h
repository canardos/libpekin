#ifndef LIBPEKIN_LP_TYPES_H_
#define LIBPEKIN_LP_TYPES_H_

#include <type_traits>

namespace libp {

/**
 * Statically casts a C++11 strongly typed enum to its underlying type.
 */
template<typename T>
constexpr std::underlying_type_t<T> enumVal(T e) noexcept
{
    return static_cast<std::underlying_type_t<T>>(e);
}

/**
 * Concept ensuring a type conversion is safe from narrowing.
 *
 * true only if an instance of the source type can be converted to the
 * destination type without triggering narrowing conversions.
 *
 * @tparam To   The target type of the conversion.
 * @tparam From The source type of the conversion.
 */
template <typename To, typename From>
concept non_narrowing = requires(From f) { To{f}; };

/**
 * Proxy utility to prevent type narrowing in concepts.
 *
 * @tparam T The type of the value that shouldn't be narrowed.
 *
 * For example:
 * @code
 * template <auto start_timer>
 * requires
 * std::invocable<decltype(start_timer),
 *     uint8_t,                // timer_idx
 *     no_narrowing<uint32_t>> // delay_ms
 * void run_timer() {
 *     // ...
 * }
 * Candidates:
 * void start_timer(uint8_t, uint32_t);  // safe
 * void start_timer(uint8_t, uint64_t);  // safe
 * void start_timer(uint8_t, uint8_t);   // fails
 * @endcode
 */
template <typename T>
struct no_narrowing {
    T value;
    template <non_narrowing<T> U>
    constexpr operator U() const {
        return value;
    }
};


} // namespace libp


#endif /* LIBPEKIN_LP_TYPES_H_ */
