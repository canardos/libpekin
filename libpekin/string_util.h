#ifndef LIB_LIBPEKIN_STRING_UTIL_H_
#define LIB_LIBPEKIN_STRING_UTIL_H_

#include <cstring>
#include <type_traits>

namespace Libp {

/**
 * Return the string length required to hold the min/maximum value of the
 * provided integral type including trailing null.
 *
 * @param type_sample integer type
 * @return number of characters required to represent the min/max value (incl.
 *         null terminator). Assumes 1-byte per character.
 */
template <typename T>
constexpr uint8_t maxStrLen(T& type_sample) {
    static_assert(std::is_integral<T>::value, "Function does not support non-integral types");
    return std::is_signed<T>::value
            ? sizeof(type_sample) * 8 * 0.302 + 3
            : sizeof(type_sample) * 8 * 0.302 + 2;
}
// Check that this evaluated at compile time
inline constexpr uint8_t test_constexpr = 0;
static_assert(maxStrLen(test_constexpr) == 4, "maxStrLen error");

/**
 * Return the string length required to hold the min/maximum value of the
 * provided integral type including trailing null.
 *
 * @return number of characters required to represent the min/max value (incl.
 *         null terminator). Assumes 1-byte per character.
 */
template <typename T>
constexpr uint8_t maxStrLen() {
    static_assert(std::is_integral<T>::value, "Function does not support non-integral types");
    return std::is_signed<T>::value
            ? sizeof(T) * 8 * 0.302 + 3
            : sizeof(T) * 8 * 0.302 + 2;
}
// Confirm that this evaluated at compile time and test
static_assert(maxStrLen<int8_t>() == 5, "maxStrLen error");
static_assert(maxStrLen<uint8_t>() == 4, "maxStrLen error");

/**
 * Copies the first @p len chars of C string @p src to @p dest and adds a null
 * terminator at dest[len - 1].
 *
 * i.e. calls strncpy and ensures @p dest is null terminated.
 *
 * @param dest
 * @param src
 * @param len
 */
inline
void strcpy_safe(char* dest, const char* src, size_t len)
{
    if (len) {
        strncpy(dest, src, len);
        dest[len - 1] = '\0';
    }
}

} // namespace Libp

#endif /* LIB_LIBPEKIN_STRING_UTIL_H_ */
