#ifndef LIBPEKIN_H_
#define LIBPEKIN_H_

#include <type_traits>

// Use when the getMillis implementation supports 64-bit
//#define USE_64_BIT_TIMERS
#include "libpekin_hal.h"

namespace Libp {

/**
 * Statically casts a C++11 strongly typed enum to its underlying type.
 */
template<typename T>
constexpr auto enumBaseT(T enum_val)
// TODO: change to consteval
{
    //static_assert(std::is_enum<T>::value, "This function is intended for enums");
    return static_cast<typename std::underlying_type<T>::type>(enum_val);
}

} // namespace Libp

#endif /* LIBPEKIN_H_ */
