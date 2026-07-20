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
#include <cinttypes>

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
            doOutput(Level::DEBUG, false, true, format, args);
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
            doOutput(Level::INFO, false, true, format, args);
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
            doOutput(Level::WARN, false, true, format, args);
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
            doOutput(Level::ERROR, false, true, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Output `format` only. No prefix or trailing newline.
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
            doOutput(level, true, true, format, args);
            va_end(args);
        }
    }

    /**
     * Note: Use provided LP_LOG_xxxx wrapper macro rather than calling this
     * function directly to ensure params are not evaluated when disabled.
     *
     * Log float value. For use where platform printf doesn't support float.
     *
     * @tparam enable
     * @param level
     * @param value
     */
    template <bool enable = true>
    static void logFloat(Level level, float value)
    {
        if constexpr (enable && LP_LOG_ENABLE) {
            //int32_t whole = value;
            //uint32_t fraction = (value - whole)* 100.0f + 0.5f;
            //raw(level, "%" PRId32 ".%02" PRIu32, whole, fraction);
            int32_t whole = static_cast<int32_t>(value);
            float diff = value - static_cast<float>(whole);
            if (diff < 0.0f) { diff = -diff; }
            uint32_t fraction = static_cast<uint32_t>(diff * 100.0f + 0.5f);
            if (fraction >= 100) {
                fraction = 0;
                if (whole >= 0) { whole += 1; } else { whole -= 1; }
            }
            raw(level, "%" PRId32 ".%02" PRIu32, whole, fraction);
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
     * @param newline append a platform dependent newline
     * @param format printf style null-terminated format string
     * @param args
     */
    static void doOutput(Level level, bool raw, bool newline, const char* format, va_list args);
    friend void doOutputWrapper(Log::Level level, bool raw, bool newline, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Log::doOutput(level, raw, newline, format, args);
        va_end(args);
    }
};

} // namespace libp


#define LP_IMPL_LOG_CALL(level_enum, raw_bool, nl_bool, enable_expr, format, ...) \
    do { \
        if constexpr ((enable_expr) && LP_LOG_ENABLE) { \
            doOutputWrapper(level_enum, raw_bool, nl_bool, format, ##__VA_ARGS__); \
        } \
    } while (0)

#define LP_LOG_LEVEL_IF(LEVEL, enable, format, ...) \
    LP_IMPL_LOG_CALL(::libp::Log::Level::LEVEL, false, true, enable, format, ##__VA_ARGS__)

#define LP_LOG_LEVEL_NONL_IF(LEVEL, enable, format, ...) \
    LP_IMPL_LOG_CALL(::libp::Log::Level::LEVEL, false, false, enable, format, ##__VA_ARGS__)

#define LP_LOG_LEVEL_RAW_IF(LEVEL, enable, format, ...) \
    LP_IMPL_LOG_CALL(::libp::Log::Level::LEVEL, true, false, enable, format, ##__VA_ARGS__)

#define LP_LOG_FLOAT_IF(LEVEL, enable, value) \
    do { \
        if constexpr ((enable) && LP_LOG_ENABLE) {  \
            ::libp::Log::logFloat<true>(::libp::Log::Level::LEVEL, value); \
        } \
    } while (0)

#define LP_LOG_DEBUG(format, ...)             LP_LOG_LEVEL_IF(DEBUG, true, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_IF(en, format, ...)      LP_LOG_LEVEL_IF(DEBUG, en, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_NONL(format, ...)        LP_LOG_LEVEL_NONL_IF(DEBUG, true, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_NONL_IF(en, format, ...) LP_LOG_LEVEL_NONL_IF(DEBUG, en, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_RAW(format, ...)         LP_LOG_LEVEL_RAW_IF(DEBUG, true, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_RAW_IF(en, format, ...)  LP_LOG_LEVEL_RAW_IF(DEBUG, en, format, ##__VA_ARGS__)
#define LP_LOG_DEBUG_FLOAT(value)             LP_LOG_FLOAT_IF(DEBUG, true, value)
#define LP_LOG_DEBUG_FLOAT_IF(en, value)      LP_LOG_FLOAT_IF(DEBUG, en, value)

#define LP_LOG_INFO(format, ...)              LP_LOG_LEVEL_IF(INFO, true, format, ##__VA_ARGS__)
#define LP_LOG_INFO_IF(en, format, ...)       LP_LOG_LEVEL_IF(INFO, en, format, ##__VA_ARGS__)
#define LP_LOG_INFO_NONL(format, ...)         LP_LOG_LEVEL_NONL_IF(INFO, true, format, ##__VA_ARGS__)
#define LP_LOG_INFO_NONL_IF(en, format, ...)  LP_LOG_LEVEL_NONL_IF(INFO, en, format, ##__VA_ARGS__)
#define LP_LOG_INFO_RAW(format, ...)          LP_LOG_LEVEL_RAW_IF(INFO, true, format, ##__VA_ARGS__)
#define LP_LOG_INFO_RAW_IF(en, format, ...)   LP_LOG_LEVEL_RAW_IF(INFO, en, format, ##__VA_ARGS__)
#define LP_LOG_INFO_FLOAT(value)              LP_LOG_FLOAT_IF(INFO, true, value)
#define LP_LOG_INFO_FLOAT_IF(en, value)       LP_LOG_FLOAT_IF(INFO, en, value)

#define LP_LOG_WARN(format, ...)              LP_LOG_LEVEL_IF(WARN, true, format, ##__VA_ARGS__)
#define LP_LOG_WARN_IF(en, format, ...)       LP_LOG_LEVEL_IF(WARN, en, format, ##__VA_ARGS__)
#define LP_LOG_WARN_NONL(format, ...)         LP_LOG_LEVEL_NONL_IF(WARN, true, format, ##__VA_ARGS__)
#define LP_LOG_WARN_NONL_IF(en, format, ...)  LP_LOG_LEVEL_NONL_IF(WARN, en, format, ##__VA_ARGS__)
#define LP_LOG_WARN_RAW(format, ...)          LP_LOG_LEVEL_RAW_IF(WARN, true, format, ##__VA_ARGS__)
#define LP_LOG_WARN_RAW_IF(en, format, ...)   LP_LOG_LEVEL_RAW_IF(WARN, en, format, ##__VA_ARGS__)
#define LP_LOG_WARN_FLOAT(value)              LP_LOG_FLOAT_IF(WARN, true, value)
#define LP_LOG_WARN_FLOAT_IF(en, value)       LP_LOG_FLOAT_IF(WARN, en, value)

#define LP_LOG_ERROR(format, ...)             LP_LOG_LEVEL_IF(ERROR, true, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_IF(en, format, ...)      LP_LOG_LEVEL_IF(ERROR, en, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_NONL(format, ...)        LP_LOG_LEVEL_NONL_IF(ERROR, true, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_NONL_IF(en, format, ...) LP_LOG_LEVEL_NONL_IF(ERROR, en, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_RAW(format, ...)         LP_LOG_LEVEL_RAW_IF(ERROR, true, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_RAW_IF(en, format, ...)  LP_LOG_LEVEL_RAW_IF(ERROR, en, format, ##__VA_ARGS__)
#define LP_LOG_ERROR_FLOAT(value)             LP_LOG_FLOAT_IF(ERROR, true, value)
#define LP_LOG_ERROR_FLOAT_IF(en, value)      LP_LOG_FLOAT_IF(ERROR, en, value)

#undef LP_PRINTF_FMT_CHECK_ATTR

#endif /* LIBPEKIN_LP_LOGGING_H_ */
