#ifndef RASTERFONT_H_
#define RASTERFONT_H_

#include <stdint.h>
#include "text.h"

namespace Libp {

/**
 * TODO
 *
 * @tparam first_char_
 * @tparam last_char_
 */
template <uint8_t first_char_, uint8_t last_char_>
class RasterFont {
public:
	/// references to image & char_meta will be retained
	/// valid bpp = 1/2/4/8
	constexpr RasterFont(const Image2d& image, const CharMeta (&char_meta)[last_char_ - first_char_ + 1])
			: image_(image), meta_(char_meta)
	{
	    static_assert(last_char_ >= first_char_);
        // Confirm that interface is correctly implemented
        static_assert(RasterFontConcept<RasterFont<first_char_, last_char_>>);
	}

    /**
     * Check if the font contains the specified character
     *
     * @param character ASCII character code
     *
     * @return true of the font contains the specified character, false
     *         otherwise.
     */
	constexpr bool validChar(uint8_t character) const
	{
		return character >= first_char_ && character <= last_char_;
	}

    /**
     * Return the meta data for the specified character.
     *
     * @param character ASCII character code
     *
     * @return the requested character meta data or meta data for the first
     *         character if the requested character is invalid.
     */
	constexpr const CharMeta& charMeta(uint8_t character) const
	{
		if (!validChar(character))
			return meta_[0];
		return meta_[character - first_char_];
	}

    /**
     * Return the full image data for the font.
     *
     * @return
     */
	constexpr const Image2d& image() const
	{
		return image_;
	}


/*	static constexpr uint16_t maxHeight(CharMeta meta[])
	{
	    constexpr CharMeta max = *std::max_element(
	            std::begin(meta),
	            std::end(meta),
	            [](const CharMeta& lhs, const CharMeta& rhs) constexpr { return (lhs.size.y + lhs.offs.y) < (rhs.size.y + rhs.offs.y); }
	            );
	    return max.size.y + max.offs.y;
	}
#include <algorithm>
#include <cstdint>
#include <array>

struct CharMeta {
    /// Position in the font bitmap image.
    const uint16_t pos;
    const uint16_t offs;
};


// Type your code here, or load an example.

template<size_t n>
constexpr int square(const CharMeta (&MyMeta)[n]) {

    constexpr CharMeta max = *std::max_element(
            std::begin(MyMeta),
            std::end(MyMeta),
            [](const CharMeta& lhs, const CharMeta& rhs) constexpr { return (lhs.pos + lhs.offs) < (rhs.pos + rhs.offs); }
            );
    return max.pos + max.offs;
}

template<size_t n>
constexpr int square2(const int (&MyInt)[n]) {

    //return MyInt[2];
    constexpr int max = *std::max_element(
            std::begin(MyInt),
            std::end(MyInt));
    return max;
}


constexpr CharMeta a = {1, 5};
constexpr CharMeta b = {10, 12};
constexpr CharMeta c = {10, 17};
constexpr CharMeta meta[] = {a, b, c};
constexpr int myints[] = {1, 2, 3, 4};

int main()
{
    return square2(myints);
    //return square(meta);
}


*/
private:
	const Image2d& image_;
	const CharMeta (&meta_)[last_char_ - first_char_ + 1];
};

} // namespace Libp

#endif /* RASTERFONT_H_ */
