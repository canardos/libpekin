#ifndef LIB_LIBPEKIN_SERIAL_WRITER_H_
#define LIB_LIBPEKIN_SERIAL_WRITER_H_

#include <cstdint>
#include <stdarg.h>
#include <algorithm>
#include "i_serial_io.h"

namespace Libp {

/**
 * Functions to write formatted output to an @p ISerialIo device.
 *
 * @tparam max_output_len_ space (in bytes) to allocate for string output in
 *                         `printf` calls. Additional output will be truncated.
 */
template <uint16_t max_output_len_ = 128>
class SerialWriter {

public:
    /**
     * @param output  device to write to
     */
    SerialWriter(ISerialIo& output_dev) : output_dev_(output_dev) { };

    /**
     * Write formatted output. Format specifiers and parameters are C @p printf
     * standard
     *
     * Output will be truncated if it exceeds `max_output_len`
     */
    void printf(const char* format, ...) const
    {
        va_list ap;
        va_start(ap, format);
        printf(format, ap);
        va_end(ap);
    }

    /**
     * Write formatted output. Format specifiers and parameters are C @p printf
     * standard.
     *
     * Output will be truncated if it exceeds `max_output_len`
     */
    void printf(const char* format, va_list ap) const
    {
        char buf[max_output_len_];
        vsnprintf(buf, max_output_len_, format, ap);
        output_dev_.write(buf);
    }

    /**
     * Write a string of characters.
     *
     * @param msg null terminated string
     */
    void print(const char* msg) const
    {
        output_dev_.write(msg);
    }

    /**
     * Write a string of characters followed by a CR and NL character.
     *
     * @param msg null terminated string
     */
    void println(const char* msg) const
    {
        output_dev_.write(msg);
        output_dev_.write("\r\n");
    }

private:
    ISerialIo& output_dev_;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_SERIAL_WRITER_H_ */
