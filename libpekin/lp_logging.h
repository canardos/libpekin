/*
 * lp_logging.h
 *
 * Simple logging facade class and macros to abstract SDK specific library
 * being used.
 *
 * 1. Implement Log::doOutput(Level level, bool raw, const char* format, va_list args)
 * 2. Enable logging by defining symbol LP_LOG_ENABLE
 * 3. Enable feature logging by defining zero or more of:
 *    - EN_LOG_BSM
 *    - EN_LOG_ONOFFLED
 *    - EN_LOG_ZCL_SCRIPTS
 * 4. Call log macros/functions as needed.
 */

#ifndef LIBPEKIN_LP_LOGGING_H_
#define LIBPEKIN_LP_LOGGING_H_

#include <cstdarg>
#include <cstdint>

//
// Global logging enable/disable flag
// Add LP_LOG_ENABLE to the build symbols to enable logging.
//
#ifdef LP_LOG_ENABLE
#undef LP_LOG_ENABLE  // silence compiler warning
#define LP_LOG_ENABLE true
#else
#define LP_LOG_ENABLE false
#endif

//
// libp components
// Add EN_LOG_[component] to the build symbols to enable logging for the
// component.
//
#ifdef EN_LOG_BSM  // Button state machine
#undef EN_LOG_BSM  // silence compiler warning
#define EN_LOG_BSM true
#else
#define EN_LOG_BSM false
#endif

#ifdef EN_LOG_ONOFFLED // on/off led
#undef EN_LOG_ONOFFLED // silence compiler warning
#define EN_LOG_ONOFFLED true
#else
#define EN_LOG_ONOFFLED false
#endif

#ifdef EN_LOG_ZCL_SCRIPTS // ZCL scripts
#undef EN_LOG_ZCL_SCRIPTS // silence compiler warning
#define EN_LOG_ZCL_SCRIPTS true
#else
#define EN_LOG_ZCL_SCRIPTS false
#endif


#define LP_PRINTF_FMT_CHECK_ATTR(fmt_idx, params_idx) [[gnu::format(printf, fmt_idx, params_idx)]]

namespace libp {

class Log {
public:
    enum class Level : uint8_t { INFO, DEBUG, WARN, ERROR };

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Adds trailing newline.
     *
     * @tparam enable
     * @param format printf style null-terminated format string
     */
    template <bool enable = true>
    LP_PRINTF_FMT_CHECK_ATTR(1, 2)
    static void debug(const char* format, ...)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            va_list args;
            va_start(args, format);
            doOutput(Level::DEBUG, false, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Adds trailing newline.
     *
     * @tparam enable
     * @param format printf style null-terminated format string
     */
    template <bool enable = true>
    LP_PRINTF_FMT_CHECK_ATTR(1, 2)
    static void info(const char* format, ...)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            va_list args;
            va_start(args, format);
            doOutput(Level::INFO, false, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Adds trailing newline.
     *
     * @tparam enable
     * @param format printf style null-terminated format string
     */
    template <bool enable = true>
    LP_PRINTF_FMT_CHECK_ATTR(1, 2)
    static void warn(const char* format, ...)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            va_list args;
            va_start(args, format);
            doOutput(Level::WARN, false, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Adds trailing newline.
     *
     * @tparam enable
     * @param format printf style null-terminated format string
     */
    template <bool enable = true>
    LP_PRINTF_FMT_CHECK_ATTR(1, 2)
    static void error(const char* format, ...)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            va_list args;
            va_start(args, format);
            doOutput(Level::ERROR, false, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Output `format` only. No prefix or newline suffix added.
     *
     * @tparam enable
     * @param format printf style null-terminated format string
     */
    template <bool enable = true>
    LP_PRINTF_FMT_CHECK_ATTR(2, 3)
    static void raw(Level level, const char* format, ...)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            va_list args;
            va_start(args, format);
            doOutput(level, true, format, args);
            va_end(args);
        }
    }


private:
    /**
     * Must be implemented by application specific to SDK/platform in use.
     *
     * Format can be platform dependent, but a trailing new line must be added
     * if `raw == false`.
     *
     * @param level
     * @param raw log provided text/args only. No leading/trailing level/newline
     * etc.
     * @param format printf style null-terminated format string
     * @param args
     */
    static void doOutput(Level level, bool raw, const char* format, va_list args);
};

#define LP_LOG_DEBUG_IF(enable, format, ...) \
    do { \
        if constexpr (enable && LP_LOG_ENABLE) { \
            ::libp::Log::debug<true>(format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_DEBUG(format, ...) \
    LP_LOG_DEBUG_IF(true, format, ##__VA_ARGS__)


#define LP_LOG_INFO_IF(enable, format, ...) \
    do { \
        if constexpr (enable && LP_LOG_ENABLE) { \
            ::libp::Log::info<true>(format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_INFO(format, ...) \
    LP_LOG_INFO_IF(true, format, ##__VA_ARGS__)


#define LP_LOG_WARN_IF(enable, format, ...) \
    do { \
        if constexpr (enable && LP_LOG_ENABLE) { \
            ::libp::Log::warn<true>(format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_WARN(format, ...) \
    LP_LOG_WARN_IF(true, format, ##__VA_ARGS__)


#define LP_LOG_ERROR_IF(enable, format, ...) \
    do { \
        if constexpr (enable && LP_LOG_ENABLE) { \
            ::libp::Log::error<true>(format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_ERROR(format, ...) \
    LP_LOG_ERROR_IF(true, format, ##__VA_ARGS__)


#define LP_LOG_RAW_IF(enable, level, format, ...) \
    do { \
        if constexpr (enable && LP_LOG_ENABLE) { \
            ::libp::Log::raw<true>(level, format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_RAW(level, format, ...) \
    LP_LOG_RAW_IF(true, level, format, ##__VA_ARGS__)

} // namespace libp

#undef LP_PRINTF_FMT_CHECK_ATTR

#endif /* LIBPEKIN_LP_LOGGING_H_ */
