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

} // namespace libp


#endif /* LIBPEKIN_LP_TYPES_H_ */
