#ifndef SRC_USER_LIB_LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_STRING_H_
#define SRC_USER_LIB_LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_STRING_H_

#include "lp_string.h"
#include <cstddef>
#include <cstdint>

namespace libp::zcl {

/**
 * Storage for a ZCL string.
 *
 * Stored with leading size byte and trailing null terminator.
 *
 * Backing buffer is N+1.
 *
 * @tparam N character content + trailing null terminator
 */
template <size_t N>
struct zcl_string {
    static_assert(N >= 1 && N <= 256, "Valid ZCL string length is 0-255 characters");

    char buf[N+1]{};

    constexpr zcl_string() {}

    template <size_t M>
    constexpr zcl_string(const char (&str)[M]) {
        validateSize<M>();
        copyAndSetLen(str, M - 1);
    }
    template <size_t M>
    constexpr zcl_string(const fixed_string<M>& fs) {
        validateSize<M>();
        copyAndSetLen(fs.c_str(), fs.size());
    }


    template <size_t M>
    constexpr zcl_string& operator=(const char (&str)[M]) {
        validateSize<M>();
        copyAndSetLen(str, M - 1);
        return *this;
    }
    constexpr zcl_string& operator=(std::string_view sv) {
        const size_t len = (sv.size() <= N - 1) ? sv.size() : (N - 1);
        copyAndSetLen(sv.data(), len);
        return *this;
    }
    /** Truncates if needed */
    template <size_t OtherN>
    constexpr zcl_string& operator=(const zcl_string<OtherN>& other) {
        const size_t copy_len = (other.size() < N - 1) ? other.size() : (N - 1);
        copyAndSetLen(other.c_str(), copy_len);
        return *this;
    }

    template <size_t M>
    static constexpr void validateSize() {
        static_assert(M <= N, "Exceeds capacity");
    }

    /** ZCL payload pointer (i.e. to initial len byte) */
    constexpr const char* z_str() const { return buf; }

    /** Pointer to start of character data */
    constexpr const char* c_str() const { return buf + 1; }
    /** Pointer to start of character data */
    constexpr const char* data() const { return buf + 1; }
    //skip imlicit conversion for now
    //constexpr operator const char*() const { return buf + 1; }
    //constexpr operator std::string_view() const { return {buf + 1, size()}; }

    constexpr uint8_t length_byte() const { return static_cast<uint8_t>(buf[0]); }
    /** Length of content in characters, excluding null terminator */
    constexpr size_t size() const { return static_cast<size_t>(length_byte()); }
    /** Maximum number of characters that may be stored */
    constexpr size_t capacity() const { return N - 1; }

private:
    /**
     * @param len length of char data excl any null term
     */
    constexpr void copyAndSetLen(const char* src, size_t len) {
        buf[0] = static_cast<char>(len);
        for (size_t i = 0; i < len; i++) {
            buf[i + 1] = src[i];
        }
        buf[len + 1] = '\0';
    }
};

template <size_t M>
zcl_string(const char (&str)[M]) -> zcl_string<M>;
template <size_t M>
zcl_string(const fixed_string<M>&) -> zcl_string<M>;

} // namespace libp::zcl

#endif /* SRC_USER_LIB_LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_STRING_H_ */
