//
// Log build configuration to the console.
//
#include "libpekin.h"
#include "lp_logging.h"


#ifdef LP_ASSERT_ENABLE
    #ifdef LP_USING_DEFAULT_CASSERT
        #pragma message("Assertions are enabled, but a custom LP_ASSERT definition has not been provided." \
        " You probably want to define LP_ASSERT in your 'libpekin_config.h' to defer to your platform-specific assert macro/function." \
        " Using assert from 'cassert' header.")
    #else
        #pragma message("Assertions are enabled")
    #endif
#else
    #pragma message("Assertions are disabled")
#endif

#ifdef LP_LOG_ENABLE
    #pragma message("Logging is enabled")
#else
    #pragma message("Logging is disabled")
#endif
