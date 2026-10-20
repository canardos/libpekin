#ifndef LIBPEKIN_STRING_H_
#define LIBPEKIN_STRING_H_

#include <cstring>
#include <type_traits>
#include <limits>
#include <concepts>
#include <cstdint>
#include <cstddef>

namespace libp {

/**
 * Convert a single decimal or hexadecimal ASCII character digit to its
 * corresponding integer value.
 *
 * No validation of `c` performed.
 *
 * @param c 0-9 or a-f or A-F
 */
constexpr uint8_t atoh (uint8_t c)
{
    if (c > '9') {
        c += 9;
    }
    return c & 0x0F;
}
static_assert(atoh('0') == 0);
static_assert(atoh('1') == 1);
static_assert(atoh('9') == 9);
static_assert(atoh('a') == 10);
static_assert(atoh('A') == 10);
static_assert(atoh('f') == 15);
static_assert(atoh('F') == 15);

/**
 * Test if a character is a hexadecimal character (i.e. 0-9, a-f, A-F)
 *
 * @param c
 *
 * @return true if `c` is a hexadecimal character
 */
constexpr bool isHex(uint8_t c) {
    return static_cast<uint8_t>(c - '0') <= 9 || static_cast<uint8_t>((c | 0x20) - 'a') <= 5;
}

static_assert(isHex('0'));
static_assert(isHex('9'));
static_assert(!isHex('~'));
static_assert(!isHex('-'));
static_assert(isHex('a'));
static_assert(isHex('b'));
static_assert(isHex('c'));
static_assert(isHex('d'));
static_assert(isHex('e'));
static_assert(isHex('f'));
static_assert(!isHex('g'));
static_assert(isHex('A'));
static_assert(isHex('B'));
static_assert(isHex('C'));
static_assert(isHex('D'));
static_assert(isHex('E'));
static_assert(isHex('F'));
static_assert(!isHex('G'));
static_assert(!isHex('.'));
static_assert(!isHex(' '));
static_assert(!isHex(0));
static_assert(!isHex(255));


/**
 * Return the string length required to hold the min/maximum decimal value of
 * the provided integral type including trailing null, but not including any
 * digit separators.
 *
 * e.g.
 * maxStrLen<int8_t>() == 5 // '-255\0'
 *
 * @return number of characters required to represent the min/max value (incl.
 *         null terminator). Assumes 1-byte per character.
 */
template <std::integral T>
constexpr uint8_t maxStrLen() {
    return std::numeric_limits<T>::digits10 + (std::is_signed_v<T> ? 3 : 2);
}
// Confirm that this evaluated at compile time and test
static_assert(maxStrLen<uint8_t>()  == strlen(        "255") + 1, "maxStrLen error");
static_assert(maxStrLen<int8_t>()   == strlen(       "-255") + 1, "maxStrLen error");
static_assert(maxStrLen<uint32_t>() == strlen( "4294967296") + 1, "maxStrLen error");
static_assert(maxStrLen<int32_t>()  == strlen("-4294967296") + 1, "maxStrLen error");


/**
 * Return the string length required to hold the min/maximum decimal value of
 * the provided integral type including trailing null, but not including any
 * digit separators.
 *
 * e.g.
 * int8_t int8 = 0;
 * maxStrLen(int8) == 5; // '-255\0'
 *
 * @param type_sample integer type
 * @return number of characters required to represent the min/max value (incl.
 *         null terminator). Assumes 1-byte per character.
 */
template <std::integral T>
constexpr uint8_t maxStrLen([[maybe_unused]] const T& type_sample) {
	return maxStrLen<T>();
}
// Check that this evaluated at compile time and test
inline constexpr uint8_t uint8 = 0;
inline constexpr int8_t int8 = 0;
inline constexpr uint32_t uint32 = 0;
inline constexpr int32_t int32 = 0;
static_assert(maxStrLen( uint8) == strlen(        "255") + 1, "maxStrLen error");
static_assert(maxStrLen(  int8) == strlen(       "-255") + 1, "maxStrLen error");
static_assert(maxStrLen(uint32) == strlen( "4294967296") + 1, "maxStrLen error");
static_assert(maxStrLen( int32) == strlen("-4294967296") + 1, "maxStrLen error");


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


/**
 * Minimal fixed_string implementation
 */
template <size_t N>
struct fixed_string {
    char buf[N]{};

    constexpr fixed_string() = default;

    constexpr fixed_string(const char (&str)[N]) {
        for (size_t i = 0; i < N-1; ++i) {
            buf[i] = str[i];
        }
        buf[N-1] = '\0';
    }

    template <size_t M>
    constexpr fixed_string(const char (&str)[M]) {
        static_assert(M <= N, "Exceeds capacity");
        for (size_t i = 0; i < M - 1; ++i) {
            buf[i] = str[i];
        }
        buf[M-1] = '\0';
    }

    constexpr const char* c_str() const { return buf; }
    constexpr const char* data() const { return buf; }
    //skip imlicit conversion for now
    //constexpr operator const char*() const { return buf; }
    //constexpr operator std::string_view() const { return {buf, N - 1}; }

    constexpr size_t size() const { return N - 1; }
};



} // namespace libp

#endif /* LIBPEKIN_STRING_H_ */
