#ifndef LIBPEKIN_DATE_H_
#define LIBPEKIN_DATE_H_

#include <cstdint>

namespace libp::time {

/**
 * Calculate the day of week from a given Gregorian date using Tomohiko
 * Sakamoto's method.
 *
 * Accurate for any Gregorian date.
 *
 * source: https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week#Sakamoto's_methods
 *
 * @param y 4-digit year
 * @param m [1..12]
 * @param d [1..31]
 *
 * @return [0..6] where 0 == Sunday
 */
constexpr uint8_t dayOfWeek(uint16_t y, uint8_t m, uint8_t d)
{
    constexpr uint8_t t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static_assert(dayOfWeek(1840,01,05) == 0);
static_assert(dayOfWeek(2026,10,20) == 2);
static_assert(dayOfWeek(2134,11,27) == 6);

} // namespace libp::time

#endif /* LIBPEKIN_DATE_H_ */
