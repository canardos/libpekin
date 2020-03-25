#ifndef LIB_LIBPEKIN_AUDIO_DAC_AUDIO_H_
#define LIB_LIBPEKIN_AUDIO_DAC_AUDIO_H_

#include <cstdint>

namespace Libp {

enum class AudioResolution : uint8_t {
    bits8,
    bits12_right,
    bits12_left,
    bits16
};

struct AudioSample {
    /// bit resolution
    AudioResolution res;
    /// rate in Hertz
    uint16_t rate;
    const uint8_t* data;
    /// length in bytes, not samples
    uint16_t len;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_AUDIO_DAC_AUDIO_H_ */
