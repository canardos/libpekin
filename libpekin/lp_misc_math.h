/**
 * Miscellaneous math utility functions
 */
#ifndef LIB_LIBPEKIN_MISC_MATH_H_
#define LIB_LIBPEKIN_MISC_MATH_H_

#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <bit>
#include <utility>

namespace libp {

/**
 * Celsius -> Fahrenheit conversion.
 *
 * @param Celsius tenths of a degree
 * @return Fahrenheit in tenths of a degree
 */
constexpr int16_t celsius_to_fahrenheit(int16_t celsius) {
    return static_cast<int16_t>(static_cast<int32_t>(celsius) * 9 / 5 + 320);
}

/**
 * Fahrenheit -> Celsius conversion.
 *
 * @param fahrenheit tenths of a degree
 * @return Celsius in tenths of a degree
 */
constexpr int16_t fahrenheit_to_celsius(int16_t fahrenheit) {
    return static_cast<int16_t>((static_cast<int32_t>(fahrenheit) - 320) * 5 / 9);
}

static_assert(celsius_to_fahrenheit(0) == 320, "0.0 C should equal 32.0 F");
static_assert(fahrenheit_to_celsius(320) == 0, "32.0 F should equal 0.0 C");

static_assert(celsius_to_fahrenheit(1000) == 2120, "100.0 C should equal 212.0 F");
static_assert(fahrenheit_to_celsius(2120) == 1000, "212.0 F should equal 100.0 C");

static_assert(celsius_to_fahrenheit(-400) == -400, "-40.0 C should equal -40.0 F");
static_assert(fahrenheit_to_celsius(-400) == -400, "-40.0 F should equal -40.0 C");

static_assert(celsius_to_fahrenheit(370) == 986, "37.0 C should equal 98.6 F");
static_assert(fahrenheit_to_celsius(986) == 370, "98.6 F should equal 37.0 C");

static_assert(celsius_to_fahrenheit(-100) == 140, "-10.0 C should equal 14.0 F");
static_assert(fahrenheit_to_celsius(140) == -100, "14.0 F should equal -10.0 C");


/// 8-bit integer log2
static constexpr uint8_t log2_8(uint8_t n)
{
    uint8_t result = 0;
    while (n >>= 1)
        ++result;
    return result;
}

// powers of 2
static_assert(log2_8(1u) == 0);
static_assert(log2_8(2u) == 1);
static_assert(log2_8(4u) == 2);
static_assert(log2_8(8u) == 3);
static_assert(log2_8(16u) == 4);
static_assert(log2_8(32u) == 5);
static_assert(log2_8(64u) == 6);
static_assert(log2_8(128u) == 7);
// non-powers of 2
static_assert(log2_8(3u) == 1);
static_assert(log2_8(7u) == 2);
static_assert(log2_8(15u) == 3);
static_assert(log2_8(100u) == 6);
static_assert(log2_8(255u) == 7);
// zero
static_assert(log2_8(0u) == 0);


/**
 * Straight line interpolation/extrapolation function.
 *
 * Returns the y value for the given x value \p xi on the
 * straight line specified by the points \p x0, \p y0 and \p x1 \p y1.
 */
constexpr double linearInterp(double x0, double y0, double x1, double y1, double xi) {
    if (x1 == x0) {
        return y0;
    }
    const double t = (xi - x0) / (x1 - x0);
    return (1.0 - t) * y0 + t * y1;
}

static_assert(linearInterp(1,1, 2,2, 1.5) == 1.5);
static_assert(linearInterp(1,1, 2,2, 0.5) == 0.5);
static_assert(linearInterp(1,1, 2,2, 2.5) == 2.5);
static_assert(linearInterp(1,1, 1,1, 1) == 1);
static_assert(linearInterp(-1,-1, -2,-2, -1.5) == -1.5);
static_assert(linearInterp(-1,-1, -2,-2, -0.5) == -0.5);
static_assert(linearInterp(-1,-1, -2,-2, -2.5) == -2.5);
static_assert(linearInterp(-1,-1, 1,1, 0) == 0);
static_assert(linearInterp(-1,-1, 1,1, 3) == 3);
static_assert(linearInterp(-1,-1, 1,1, -3) == -3);
static_assert(linearInterp(-1,-2, 1,2, 0) == 0);
static_assert(linearInterp(-1,-2, 1,2, 1) == 2);
static_assert(linearInterp(-1,-2, 1,2, 4) == 8);
static_assert(linearInterp(-1,-2, 1,2, -4) == -8);


// std::cmp_greater/cmp_less requires a real integer type and disallows these
template <typename T>
concept CmpSafeInt = std::is_integral_v<T> &&
                      !std::is_same_v<T, bool> &&
                      !std::is_same_v<T, char> &&
                      !std::is_same_v<T, wchar_t> &&
                      !std::is_same_v<T, char8_t> &&
                      !std::is_same_v<T, char16_t> &&
                      !std::is_same_v<T, char32_t>;

template <typename Out, typename In>
[[nodiscard]] constexpr Out constrain(const In& value, const Out& min, const Out& max)
{
    if constexpr (CmpSafeInt<In> && CmpSafeInt<Out>) {
        if (std::cmp_greater(value, max))
            return max;
        else if (std::cmp_less(value, min))
            return min;
        // else value is def representable by Out
    }
    else {
        if (value > max)
            return max;
        if (value < min)
            return min;
    }
    // Could avoid cast if In==Out and qualify for move...?
    return static_cast<Out>(value);
}

// Within bounds
static_assert(constrain(5, 0, 10) == 5);
static_assert(constrain(0, 0, 10) == 0);
static_assert(constrain(10, 0, 10) == 10);

// Out of bounds (signed)
static_assert(constrain(-5, 0, 10) == 0);
static_assert(constrain(15, 0, 10) == 10);

// Mixed signed / unsigned types (CmpSafeInt path)
static_assert(constrain(-10, 0u, 100u) == 0u);
static_assert(constrain(200, 0u, 100u) == 100u);
static_assert(constrain(50u, 0, 100) == 50);
static_assert(constrain(3'000'000'000u, 0, 1000) == 1000);

// Floating point types (non-CmpSafeInt path)
static_assert(constrain(-1.5, 0.0, 10.0) == 0.0);
static_assert(constrain(12.5, 0.0, 10.0) == 10.0);
static_assert(constrain(5.5, 0.0, 10.0) == 5.5);


constexpr bool deg_test_norm(uint16_t deg, uint16_t start, uint16_t end) {
    return deg >= start && deg <= end;
}

constexpr bool deg_test_inv(uint16_t deg, uint16_t start, uint16_t end) {
    return deg >= start || deg <= end;
}

/**
 * Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
 *
 * Converts any XY values including 0 to a degree value that should be
 * within +/- 1 degree of the accurate value without needing
 * large slow trig functions like ArcTan() or ArcCos().
 *
 * NOTE! at least one of the X or Y values must be non-zero!
 *
 * This is the full version, for all 4 quadrants and will generate
 * the angle in integer degrees from 0-360.
 *
 * Any values of X and Y are usable including negative values provided
 * they are between -1456 and 1456 so the 16bit multiply does not overflow.
 *
 * Source: http://www.romanblack.com/integer_degree.htm
 *
 * @param x must be between -1456 and 1456
 * @param y must be between -1456 and 1456
 *
 * @return integer degrees
 */
constexpr uint16_t integer_atan(int32_t x, int32_t y)
{
    uint8_t negflag;
    uint8_t tempdegree;
    uint8_t comp;
    uint32_t degree; // this will hold the result
    uint32_t ux;
    uint32_t uy;

    // Save the sign flags then remove signs and get XY as unsigned ints
    negflag = 0;
    if(x < 0) {
        negflag += 0x01;    // x flag bit
        x = (0 - x);        // is now +
    }
    ux = x;                // copy to unsigned var before multiply
    if(y < 0) {
        negflag += 0x02;    // y flag bit
        y = (0 - y);        // is now +
    }
    uy = y;                // copy to unsigned var before multiply

    // 1. Calc the scaled "degrees"
    if(ux > uy) {
        degree = (uy * 45) / ux;   // degree result will be 0-45 range
        negflag += 0x10;    // octant flag bit
    }
    else {
        degree = (ux * 45) / uy;   // degree result will be 0-45 range
    }

    // 2. Compensate for the 4 degree error curve
    comp = 0;
    tempdegree = degree;    // use an unsigned char for speed!
    if(tempdegree > 22) {   // if top half of range
        if(tempdegree <= 44) comp++;
        if(tempdegree <= 41) comp++;
        if(tempdegree <= 37) comp++;
        if(tempdegree <= 32) comp++;  // max is 4 degrees compensated
    }
    else {    // else is lower half of range
        if(tempdegree >= 2) comp++;
        if(tempdegree >= 6) comp++;
        if(tempdegree >= 10) comp++;
        if(tempdegree >= 15) comp++;  // max is 4 degrees compensated
    }
    degree += comp;   // degree is now accurate to +/- 1 degree!

    // Invert degree if it was X>Y octant, makes 0-45 into 90-45
    if(negflag & 0x10)
        degree = (90 - degree);

    // 3. Degree is now 0-90 range for this quadrant,
    // need to invert it for whichever quadrant it was in
    if(negflag & 0x02)   // if -Y
    {
        if(negflag & 0x01)   // if -Y -X
                degree = (180 + degree);
        else        // else is -Y +X
                degree = (180 - degree);
    }
    else {    // else is +Y
        if(negflag & 0x01)   // if +Y -X
                degree = (360 - degree);
    }
    return degree;
}


/// Round "up" (-1.8 -> -1, 3.2 -> 4 etc.)
constexpr int32_t ceiling(float value) {
    int32_t cast = static_cast<int32_t>(value);
    return (value <= 0.0f)
        ? cast
        : static_cast<float>(cast) == value ? cast : cast + 1;
}

static_assert(ceiling(1.2) == 2, "ceiling error");
static_assert(ceiling(1.8) == 2, "ceiling error");
static_assert(ceiling(1  ) == 1, "ceiling error");
static_assert(ceiling(-1.8) == -1, "ceiling error");


/// Round up to the next whole multiple of X
constexpr uint32_t ceiling(uint32_t value, uint32_t multiple) {
    if (multiple == 0 || value == 0)
        return value;
    return ((value - 1) / multiple + 1) * multiple;
}

static_assert(ceiling(6,3) == 6, "ceiling error");
static_assert(ceiling(5,3) == 6, "ceiling error");
static_assert(ceiling(2,3) == 3, "ceiling error");
static_assert(ceiling(2,0) == 2, "ceiling error");


/**
 * Returns a pseudo-random integer in [min, max]
 *
 * @note Check RAND_MAX on the platform for max range.
 * @note Biased distribution due to modulo operation.
 */
inline
int32_t randInRng(int32_t min, int32_t max)
{
    if (min >= max) {
        return min;
    }
    const uint64_t range = static_cast<uint64_t>(max) - min + 1;
    return static_cast<int32_t>(
            static_cast<int64_t>(min) + ( static_cast<uint32_t>(rand()) % range )
    );
}

template <std::unsigned_integral T>
constexpr bool isPowerOf2(T n) {
    return std::has_single_bit(n);
}

static_assert(isPowerOf2(1u), "isPowerOf2 error");
static_assert(isPowerOf2(2u), "isPowerOf2 error");
static_assert(isPowerOf2(256u), "isPowerOf2 error");

static_assert(!isPowerOf2(0u), "isPowerOf2 error");
static_assert(!isPowerOf2(3u), "isPowerOf2 error");
static_assert(!isPowerOf2(8388606u), "isPowerOf2 error");
static_assert(isPowerOf2(8388608u), "isPowerOf2 error");


/**
 * Convert a value to a 2-digit packed (4-bits per digit) binary-coded decimal
 * representation.
 *
 * @param value must be <=99
 */
constexpr uint8_t bin2bcd(uint8_t value)
{
    return ((((value) / 10) << 4) + (value) % 10);
}

/**
 * Convert a packed (4-bits per digit) 2-digit binary-coded decimal value to
 * normal binary representation.
 */
constexpr uint8_t bcd2bin(uint8_t value)
{
    return (((value) & 0x0f) + ((value) >> 4) * 10);
}

// Tests
static_assert(bin2bcd( 5) ==  0b0000'0101);
static_assert(bin2bcd(10) ==  0b0001'0000);
static_assert(bin2bcd(22) ==  0b0010'0010);
static_assert(bcd2bin(0b0000'0101) ==  5);
static_assert(bcd2bin(0b0001'0000) == 10);
static_assert(bcd2bin(0b0010'0010) == 22);

} // namespace libp

#endif /* LIB_LIBPEKIN_MISC_MATH_H_ */
