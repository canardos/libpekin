#ifndef LIB_LIBPEKIN_I_SERIAL_IO_H_
#define LIB_LIBPEKIN_I_SERIAL_IO_H_

#include <cstdint>

namespace Libp {

/**
 * "Interface" for a basic serial comms device.
 *
 * Intended for ASCII I/O.
 */
class ISerialIo {

public:
    /// End of line type for use with `readln` function
    enum class Eol : uint8_t {
        cr,
        lf,
        crlf
    };

    virtual ~ISerialIo() = default;
    /**
     * Read multiple characters from the serial device.
     *
     * @param buf buffer to receive the data. Must be >= @p len bytes. No null
     *            terminator will be added.
     * @param len number of bytes to read.
     * @param timeout_ms maximum time to wait for a single character. 0 for no
     *                   limit.
     * @return the number of characters read.
     */
    virtual uint16_t read(char* buf, uint16_t len, uint32_t timeout_ms) = 0;

    /**
     * Read characters from the serial device until a terminator character is
     * found, @p max_len is reached, or a timeout occurs.
     *
     * The buffer will contain the terminator character, but no additional
     * termination.
     *
     * @param buf buffer to receive the data.
     * @param terminator
     * @param max_len maximum number of bytes to read including the terminating
     *                character (i.e. @p buf size).
     * @param timeout_ms maximum time to wait for a single character. 0 for no
     *                   limit.
     *
     * @return the number of characters read (including the terminator if read)
     */
    virtual uint16_t read(char* buf, char terminator, uint16_t max_len, uint32_t timeout_ms) = 0;

    /**
     * Read characters from the serial device until an end of line or null is
     * found, @p max_len is reached, or a timeout occurs.
     *
     * The buffer will not contain the CR/LF terminator(s), but a null
     * terminator will be added in all cases where `max_len > 0`.
     * Note that if the CRLF terminator is used and the function times out
     * after receiving a CR, but not a LF, the buffer will contain the CR.
     *
     * @param buf buffer to receive the data.
     * @param eol_type type of end-of-line-terminator.
     * @param max_len maximum number of bytes to read including the terminating
     *                null (i.e. @p buf size).
     * @param timeout_ms maximum time to wait for a single character. 0 for no
     *                   limit.
     *
     * @return the number of characters read, excluding the terminators and
     *         added null terminator (i.e. @p strlen(buf)).
     */
    virtual uint16_t readln(char* buf, Eol eol_type, uint16_t max_len, uint32_t timeout_ms) = 0;

    /**
     * Write a single character.
     *
     * @param ch
     */
    virtual void write(char ch) const = 0;

    /**
     * Write a null terminated C-style string.
     *
     * @param msg null terminated character string.
     */
    virtual void write(const char* msg) const = 0;

    /**
     * Write a fixed length C-style string.
     *
     * @param msg character array.
     * @param len number of characters to write.
     */
    virtual void write(const char* msg, uint16_t len) const = 0;
};

} // namespace Libp

#endif /* LIB_LIBPEKIN_I_SERIAL_IO_H_ */
