/**
 * Miscellaneous math functions
 */
#ifndef LIB_LIBPEKIN_MISC_MATH_H_
#define LIB_LIBPEKIN_MISC_MATH_H_

#include <cstdint>
#include <cstdlib>

namespace Libp {

/**
 * Celcius -> Farenheit conversion.
 *
 * @param celcius
 * @return Fahrenheit
 */
/*constexpr float celcius_to_fahrenheit(float celcius) {
    return celcius * 9 / 5.0f + 32;
}*/

/**
 * Celcius -> Farenheit conversion.
 *
 * @param celcius in tenths of a degree
 * @return Fahrenheit in tenths of a degree
 */
/*constexpr float fahrenheit_to_celcius(float fahrenheit) {
    return (fahrenheit - 32) * 5 / 9.0f;
}*/


/**
 * Celcius -> Fahrenheit conversion.
 *
 * @param celcius tenths of a degree
 * @return Fahrenheit in tenths of a degree
 */
constexpr int16_t celcius_to_fahrenheit(int16_t celcius) {
    return celcius * 9 / 5 + 320;
}

/**
 * Fahrenheit -> Celcius conversion.
 *
 * @param fahrenheit tenths of a degree
 * @return Celcius in tenths of a degree
 */
constexpr int16_t fahrenheit_to_celcius(int16_t fahrenheit) {
    return (fahrenheit - 320) * 5 / 9;
}

/// 8-bit integer log2
static constexpr uint8_t log2_8(uint8_t n)
{
    uint8_t result = 0;
    while (n >>= 1)
        ++result;
    return result;
}


/**
 * Straight line interpolation/extrapolation function.
 *
 * Returns the y value for the given x value \p xi on the
 * straight line specified by the points \p x0, \p y0 and \p x1 \p y1.
 */
constexpr double linearInterp(double x0, double y0, double x1, double y1, double xi) {
    return y0 + (y1 - y0) * ( (xi - x0) / (x1 - x0) );
}


// TODO: reference and test
template <class T>
const T& constrain(const T& value, const T& min, const T& max)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}

template <class T, class U>
constexpr T constrain(const T value, const U min, const U max)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}

inline
static bool deg_test_norm(uint16_t deg, uint16_t start, uint16_t end) {
    return deg >= start && deg <= end;
}

inline
static bool deg_test_inv(uint16_t deg, uint16_t start, uint16_t end) {
    return deg >= start || deg <= end;
}

/**
 * Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
 *
 * Converts any XY values including 0 to a degree value that should be
 * within +/- 1 degree of the accurate value without needing
 * large slow trig functions like ArcTan() or ArcCos().
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
inline
static uint16_t integer_atan(int32_t x, int32_t y)
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
constexpr uint32_t ceiling(float value) {
    uint32_t cast = static_cast<uint32_t>(value);
    bool is_integer = static_cast<float>(cast) == value;
    bool is_positive = value > 0;
    return is_integer
            ? cast
            : cast + (is_positive ? 1 : 0);
}

/// Round up to the next whole multiple of X
constexpr uint32_t ceiling(uint32_t value, uint32_t multiple) {
    if (multiple == 0)
        return value;
    return ((value - 1) / multiple + 1) * multiple;
}

// Tests
static_assert(ceiling(6,3) == 6, "ceiling error");
static_assert(ceiling(5,3) == 6, "ceiling error");
static_assert(ceiling(2,3) == 3, "ceiling error");
static_assert(ceiling(2,0) == 2, "ceiling error");


/// biased. don't use for small ranges
inline
int32_t randInRng(int32_t min, int32_t max)
{
    return min + (rand() % static_cast<int32_t>(max - min + 1));
    //return min + (rand() * static_cast<int16_t>((max - min) / RAND_MAX));
}


constexpr bool isPowerOf2(int n) {
    return (n && ((n & (n - 1)) == 0)) || (n == 0);
}

static_assert(isPowerOf2(0), "isPowerOf2 error");
static_assert(isPowerOf2(1), "isPowerOf2 error");
static_assert(isPowerOf2(2), "isPowerOf2 error");
static_assert(!isPowerOf2(3), "isPowerOf2 error");
static_assert(isPowerOf2(256), "isPowerOf2 error");
static_assert(!isPowerOf2(8388606), "isPowerOf2 error");
static_assert(isPowerOf2(8388608), "isPowerOf2 error");

constexpr uint8_t bin2bcd(uint8_t value)
{
    return ((((value) / 10) << 4) + (value) % 10);
}

constexpr uint8_t bcd2bin(uint8_t value)
{
    return (((value) & 0x0f) + ((value) >> 4) * 10);
}

// Tests
static_assert(bin2bcd( 5) ==  0b0000'0101, "bcd2bin error");
static_assert(bin2bcd(10) ==  0b0001'0000, "bcd2bin error");
static_assert(bin2bcd(22) ==  0b0010'0010, "bcd2bin error");
static_assert(bcd2bin(0b0000'0101) ==  5, "bcd2bin error");
static_assert(bcd2bin(0b0001'0000) == 10, "bcd2bin error");
static_assert(bcd2bin(0b0010'0010) == 22, "bcd2bin error");

} // namespace Libp

#endif /* LIB_LIBPEKIN_MISC_MATH_H_ */
