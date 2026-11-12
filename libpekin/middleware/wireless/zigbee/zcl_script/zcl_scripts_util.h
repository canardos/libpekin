#ifndef LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_SCRIPTS_UTIL_H_
#define LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_SCRIPTS_UTIL_H_

#include <cstdint>

namespace libp::zcl_script::internal {

/**
 * Convert two character array to a 16-bit uint.
 *
 * e.g. ['1','2'] -> '1' << 8 | '2'
 *
 * @param cmd 2-char cmd
 */
inline uint16_t constexpr dblCharToU16(const char* cmd)
{
    return static_cast<uint16_t>(
        static_cast<uint8_t>(cmd[0]) << 8 | static_cast<uint8_t>(cmd[1])
    );
}


inline constexpr uint64_t nstrToU64_invalid = 0xffff'ffff'ffff'ffff;

/**
 * Fixed length positive decimal or hex integer string to uint64_t.
 *
 * @param p_str String to parse
 * @param len Length of the string
 * @return 0xffff'ffff'ffff'ffff on error, overflow, or empty string
 */
inline constexpr uint64_t nstrToU64(const char* p_str, uint8_t len)
{
	if (p_str == nullptr || len == 0) {
		return nstrToU64_invalid;
	}

	uint8_t i = 0;
	uint64_t val = 0;
	uint8_t base = 10;

	if (len > 1 && p_str[0] == '0' && (p_str[1] == 'x' || p_str[1] == 'X')) {
		base = 16;
		i = 2;
	}

	bool hex = (base == 16);
	uint8_t max_chars = hex ? 18 : 20;
	if (len > max_chars) {
	    return nstrToU64_invalid;
	}

	uint64_t limit = nstrToU64_invalid / base;
	uint8_t rem = nstrToU64_invalid % base;

	while (i < len) {
		char c = p_str[i++];
		uint8_t digit = 0;

		if (c >= '0' && c <= '9') {
			digit = c - '0';
		}
		else if (hex && c >= 'A' && c <= 'F') {
			digit = c - 'A' + 10;
		}
		else if (hex && c >= 'a' && c <= 'f') {
			digit = c - 'a' + 10;
		}
		else {
			return nstrToU64_invalid;
		}

		// Overflow check
		if (val > limit || (val == limit && digit > rem)) {
			return nstrToU64_invalid;
		}

		val = val * base + digit;
	}

	return val;
}

static_assert(nstrToU64("0",1) == 0);
static_assert(nstrToU64("123",3) == 123);
static_assert(nstrToU64("020",3) == 20);
static_assert(nstrToU64("0d3",3) == nstrToU64_invalid);

static_assert(nstrToU64("0xa", 3) == 0xa);
static_assert(nstrToU64("0Xa", 3) == 0xa);
static_assert(nstrToU64("0xA", 3) == 0xa);
static_assert(nstrToU64("0x1234", 6) == 0x1234);
static_assert(nstrToU64("0x112233", 8) == 0x112233);

static_assert(nstrToU64("0x1122334455667788", 18) == 0x1122334455667788ull);

static_assert(nstrToU64("gx12", 4) == nstrToU64_invalid);
static_assert(nstrToU64("0x12g", 5) == nstrToU64_invalid);
static_assert(nstrToU64("0x12345678901234567", 19) == nstrToU64_invalid);


} // namespace libp::zcl_script::internal

#endif // LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_SCRIPTS_UTIL_H_
