#ifndef SRC_RN_52_H_
#define SRC_RN_52_H_

#include <cstdint>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>

#include <strlcpy.h>
//#include <error_handler.h>
#include "libpekin.h"
#include "serial/i_serial_io.h"

namespace Libp {

/**
 * Limited driver for the Microchip RN52 Bluetooth audio module.
 *
 * Includes basic functionality required for a Bluetooth speaker application
 * with a display for metadata.
 *
 * The driver is stateless and assumes the device is always in command mode
 * (i.e. GPIO9 low).
 *
 * Developed for and tested on firmware version 1.16.
 *
 * Note
 * ----
 * The RN52 firmware 1.16 exhibits undocumented behavior in regard to audio
 * metadata. When track change notification is enabled, the device transmits
 * one or more metadata lines over the UART at connection, when a track
 * changes, and at other times. This issue only occurs when connected to
 * certain devices.
 *
 * This driver attempts to mitigate the problem by ignoring lines received that
 * appear to be metadata, unless we have specifically requested it via the AD
 * command.
 */
class Rn52 {

public:
    /// Commands for use with `basicActionCmd` function.
    struct Cmd {
        static constexpr char dicoverable_on[] = "@,1\r";
        static constexpr char dicoverable_off[] = "@,0\r";
        static constexpr char avrcp_vol_up[] = "AV+\r";
        static constexpr char avrcp_vol_down[] = "AV-\r";
        static constexpr char avrcp_play_pause[] = "AP\r";
        static constexpr char avrcp_next[] = "AT+\r";
        static constexpr char avrcp_prev[] = "AT-\r";
        static constexpr char clear_saved_pairings[] = "U\r";
        static constexpr char accept_pairing[] = "#,1\r";  ///< Accept pairing in Keyboard I/O auth. mode.
        static constexpr char reject_pairing[] = "#,0\r";  ///< Reject pairing in Keyboard I/O auth. mode.
        static constexpr char recon_most_recent[] = "B\r"; /**  The module attempts to reconnect the Bluetooth
                                                                profiles specified in the connection mask to
                                                                the most recently paired device. */
    };

    /// Configuration options for use with `setConfig` function.
    enum class Feature : uint16_t {
        en_avrcp_btns = 1<<0,                   ///< Enable AVRCP buttons
        en_pwr_on_reconnect = 1<<1,             ///< Enable reconnect on power on
        en_discover_on_start = 1<<2,            ///< Discoverable on power on
        en_codec_indic = 1<<3,                  ///< Enable codec indicators on PIO6/7
        en_reboot_on_disconnect = 1<<4,         ///< Reboot after disconnect
        mute_vol_chg_tones = 1<<5,              ///<
        en_voice_cmd_button = 1<<6,             ///<
        disable_sys_tones = 1<<7,               ///<
        en_pwr_off_on_pair_timeout = 1<<8,      ///<
        en_reset_after_pwr_off = 1<<9,          ///<
        en_list_reconnect_after_panic = 1<<10,  ///<
        en_latch_event_indic = 1<<11,           ///< Enable latched event signal on PIO2
        en_track_change_event = 1<<12,          ///<
        en_fixed_vol_tones = 1<<13,             ///<
        en_auto_keyboard_passcode = 1<<14       ///< Enable auto-accept passkey when in Keyboard I/O Auth mode
    };

    /// Connection state (bits 0-3 of status received on 'Q' cmd)
    enum class StatusState : uint8_t {
        // DO NOT REORDER
        state_limbo = 0,
        state_connectable,
        state_discoverable_connectable,
        state_connected,
        state_out_call_established,
        state_in_call_established,
        state_active_call_headset,
        state_test_mode,
        state_three_way_waiting,
        state_three_way_hold,
        state_three_way_multi_call,
        state_in_call_hold,
        state_active_call_handset,
        state_audio_streaming,
        state_low_battery,
    };
    friend constexpr uint8_t operator&(uint8_t& status1, StatusState status2) {
        return status1 & enumBaseT(status2);
    }

    /// Status register values from the 'Q' command
    enum class StatusFlags : uint16_t {
        /// HFP audio volume change from the audio gateway (phone)
        audio_vol_change_event = 1<< 4,
        /// HFP audio microphone change from audio gateway (phone)
        microphone_vol_change_event = 1<< 5,
        // Bits 6-7 reserved

        // Connected profiles (bits 8-13)
        active_con_iap = 1<<8,
        active_con_spp = 1<<9,
        active_con_a2dp = 1<<10,
        active_con_hfphsp = 1<<11,

        caller_id_event = 1<<12,
        track_change_event = 1<<13,
    };  // Bits 14-15 reserved

    friend constexpr uint16_t operator&(StatusFlags& status1, StatusFlags status2) {
        return enumBaseT(status1) & enumBaseT(status2);
    }
    friend constexpr uint16_t operator&(uint16_t& status1, StatusFlags status2) {
        return status1 & enumBaseT(status2);
    }

    /// BT profiles
    enum class Profile : uint8_t {
        iap = 1,  ///< Wireless iAP (MFi)
        spp = 2,  ///< Virtual serial port profile
        a2dp = 4, ///< Advanced audio distribution profile
        hfp = 8   ///< Hands-free profile
    };

    /// See `setAudioOut` function.
    enum class AudioOutput : char {
        analog = '0',      ///< Analog output (default)
        i2s = '1',         /**< I2S (Master. Variable sample rate. 24-bit. Data
                                is right channel with word select (WS) high.
                                The SD data MSB occurs in the second SCLK
                                period. Left justified) **/
        spdif = '2',       ///< S/PDIF
        intercom_dac = '3' ///< Intercom DAC mode
    };

    /// See `setAudioOut` function.
    enum class BitsPerSample : char {
        bits_24 = '0',
        bits_32 = '1'
    };

    /// See `setAudioOut` function.
    enum class SampleRate : char {
        rate_8k = '0',
        rate_32k = '1',
        rate_44k = '2',
        rate_48k = '3'
    };

    /// See `setAuth` function.
    enum class AuthMethod : char {
        open = '0',         ///< Authentication is not required. PIN code mode accepted. (legacy)
        ssp_keyboard = '1', ///< SSP keyboard I/O mode. (Default)
        ssp = '2',          ///< SSP "just works" mode.
        pin_code = '4'      ///< Forces PIN code mode, requiring host to enter PIN. (legacy)
    };

    /// Now playing metadata for `avrcpGetMetaData` function.
    struct MetaData {
        /// Excluding null terminator
        static constexpr uint8_t attribute_maxlen = 64;
        char title[attribute_maxlen + 1];
    	char artist[attribute_maxlen + 1];
    	char album[attribute_maxlen + 1];
    	uint8_t track;
    	uint8_t tracks;
    	char genre[attribute_maxlen + 1];
    	uint32_t duration;
    };

    /*
     * All set commands respond with:
     * AOK,[xxx]\r\n
     * ERR,[xxx]\r\n
     */

    /**
     * No initialization is performed on construction.
     *
     * If `serial` is buffered, be sure to clear buffer prior using this
     * driver. "CMD\r\n" for example will typically be output by the device
     * on startup (or when GPIO9 is pulled low).
     *
     * @param serial serial connection to the RN52 device. Default connection
     *               parameters are 8-bits, 1 stop, no parity, 115200 baud.
     */
    Rn52(ISerialIo& serial) : serial_(serial) {  }

    /**
     * Set the A2DP audio output routing.
     *
     * Reboot required to take effect.
     *
     * @param output output channel
     * @param bits bits per sample
     * @param rate sample rate
     *
     * @return `true` on success, `false` on error.
     */
    bool setAudioOut(AudioOutput output, BitsPerSample bits, SampleRate rate)
    {
        char cmd[] = "S|,0XXX\r";
        cmd[4] = static_cast<char>(output);
        cmd[5] = static_cast<char>(bits);
        cmd[6] = static_cast<char>(rate);
        serial_.write(cmd);
        return receivedAok();
    }

    // TODO: we should probably have a runtime version of this to allow for
    //       modifying features via user settings.
    //       Ditto for set discovery/connection mask.
    /**
     * Set the the extended features.
     *
     * Reboot required to take effect.
     *
     * @tparam feature
     *
     * @return `true` on success, `false` on error.
     */
    template <Feature...feature>
    bool setFeatures()
    {
        constexpr uint16_t cfg = (enumBaseT(feature) | ... | 0);
        constexpr uint8_t max_len = sizeof("S%,ABCD\r");
        char cmd[max_len];
        snprintf(cmd, max_len, "S%%,%04x\n", cfg);
        serial_.write(cmd);
        return receivedAok();
    }

    enum class Option : uint16_t {
        option_a = 1,
        option_b = 2,
        option_c = 8,
        option_d = 64
    };

    /***
     * Set the authentication to use when a remote device tries to connect. See
     * the RN52 reference manual for full details.
     *
     * Reboot required to take effect??
     *
     * @param auth
     */
    bool setAuth(AuthMethod auth)
    {
        char cmd[] = "SA,X\r";
        cmd[3] = static_cast<char>(auth);
        serial_.write(cmd);
        return receivedAok();
    }

    /**
     * Set the Class Of Device (COD).
     *
     * @param cod 6-character numeric COD code.
     *
     * @return `true` on success, `false` on error.
     */
    bool setCod(const char (&cod)[6])
    {
    	serial_.write("SC,");
    	serial_.write(cod, 6);
    	serial_.write("\r");
        return receivedAok();
    }

    template <Profile... profiles>
    bool setDiscoveryMask()
    {
        constexpr uint8_t mask = (enumBaseT(profiles) | ... | 0);
        constexpr uint8_t max_len = sizeof("SD,12\r");
        char cmd[max_len];
        // TODO: this should be doable at compile time.
        //       C++20 container ops with std::string?
        snprintf(cmd, max_len, "SD,%02x\n", mask);
        serial_.write(cmd);
        return receivedAok();
    }

    template <Profile... profiles>
    bool setConnectionMask()
    {
        constexpr uint8_t mask = (enumBaseT(profiles) | ... | 0);
        constexpr uint8_t max_len = sizeof("SK,12\r");
        char cmd[max_len];
        // TODO: this should be doable at compile time.
        //       C++20 container ops with std::string?
        snprintf(cmd, max_len, "SK,%02x\n", mask);
        serial_.write(cmd);
        return receivedAok();
    }

    /**
     * Set the device name.
     *
     * @param name Max 20 characters
     *
     * @return `true` on success, `false` on error.
     */
    bool setDevName(const char* name)
    {
        serial_.write("SN,");
        serial_.write(name);
        serial_.write("\r");
        return receivedAok();
    }

    /**
     * Set the speaker gain level (analog route only).
     *
     * @param gain 0x00->0x0f
     *
     * @return `true` on success, `false` on error.
     */
    bool setSpkrGain(uint8_t gain)
    {
        constexpr uint8_t max_len = sizeof("SS,12\r");
        char cmd[max_len];
        snprintf(cmd, max_len, "SS,%02x\n", gain);
        serial_.write(cmd);
        return receivedAok();
    }

    /**
     * Set the tone gain level.
     *
     * @param gain 0x00->0x1f
     *
     * @return `true` on success, `false` on error.
     */
    bool setToneGain(uint8_t gain)
    {
        constexpr uint8_t max_len = sizeof("ST,12\r");
        char cmd[max_len];
        snprintf(cmd, max_len, "ST,%02x\n", gain);
        serial_.write(cmd);
        return receivedAok();
    }

    /**
     * Return the battery status of the audio gateway (phone)
     *
     * @return Battery level in percent or -1 on error
     */
    int8_t getBatteryStatus()
    {
        serial_.write("GB\r");
        char buf[max_line_len];
        uint8_t len = serialReadLineIgnoreMetaData(buf, max_line_len, cmd_response_timeout_ms);
        // TODO: is it \r\r\n or \r\n?
        len -= 2;
        if (len == 18) {
            return 100;
        }
        else if (len == 17) {
            char ones = buf[--len] - '0';
            char tens = buf[--len] - '0';
            return tens * 10 + ones;
        }
        else if (len == 16) {
            return buf[--len] - '0';
        }
        else {
            return -1;
        }
    }

    /// Maximum single line length returned over the UART.
    static constexpr uint16_t max_line_len = 255;

    /// actQueryStatus function will return this value on parse error
    static constexpr uint16_t query_status_error = 999;

    /**
     * Return the device status (result of "Q" command).
     *
     * A zero return value likely represents an error.
     *
     * @return 16-bit status value. Refer to the RN52 docs for details.
     */
    uint16_t actQueryStatus()
    {
        serial_.write("Q\r");
        char buf[max_line_len]; // "ffff\r\n"
        serialReadLineIgnoreMetaData(buf, max_line_len, cmd_response_timeout_ms);
        errno = 0;
        uint16_t status = strtoul(buf, nullptr, 16);
        return (errno == 0)
                ? status
                : query_status_error;
    }

    /**
     * Send a basic action command.
     *
     * See `Rn52::Cmd` for commands.
     *
     * @param cmd
     *
     * @return `true` on success, `false` otherwise.
     */
    bool basicActionCmd(const char* cmd)
    {
        serial_.write(cmd);
        return receivedAok();
    }

    /**
     * Request audio metadata via AVRCP.
     *
     * On success, zero of more of the meta data fields will be filled.
     *
     * To minimize stack usage, the function parses the returned data line by
     * line. Metadata is copied one attribute at a time into the `MetaData`
     * structure.
     *
     * It is recommended to use interrupt buffering on the UART to avoid
     * missing data. Behavior is undefined if data is missed, although likely
     * only attributes will be missed.
     *
     * If interrupt buffering can't be used, a reduced baud rate might be
     * required on slow MCUs.
     *
     * @param meta_data [out] structure to receive the meta data. Fields that
     *                  receive data will be null terminated, but fields are
     *                  not guaranteed to receive data so the struct should be
     *                  zero initialized before passing.
     *
     * @return `true` on success, `false` on error.
     */
	bool avrcpGetMetaData(MetaData* meta_data)
	{
        /*
        Example device response:

        "AOK\r\r\n
        Title=abc\r\r\n
        Artist=abc\r\r\n
        Album=abc\r\r\n
        TrackNumber=0\r\r\n
        TrackCount=1\r\r\n
        Genre=abc\r\r\n
        Time(ms)=358000\r\r\n"

        If nothing is playing, returns: "AOK\r\n" only
        */
        serial_.write("AD\r");
        if (!receivedAok())
            return false;

        constexpr uint8_t max_attr_line_len = MetaData::attribute_maxlen + strlen("Artist=\r\r\n");
	    char meta[max_attr_line_len];

        constexpr uint16_t meta_data_line_timeout_ms = 200; // TODO: tune
	    uint16_t n = serial_.read(meta, '\n', max_attr_line_len, meta_data_line_timeout_ms);

	    while (n >= 3) {
	        // trim \r\r\n
	        meta[n - 3] = '\0'; // If we didn't read full line, we lose 1-2 chars here

	        // TODO; we could shorten the strings here to save flash
	        if (strncmp(meta, "Title=", 6) == 0) {
	            strlcpy(meta_data->title, meta + 6, MetaData::attribute_maxlen + 1);
	        }
            else if (strncmp(meta, "Artist=", 7) == 0) {
                strlcpy(meta_data->artist, meta + 7, MetaData::attribute_maxlen + 1);
            }
            else if (strncmp(meta, "Album=", 6) == 0) {
                strlcpy(meta_data->album, meta + 6, MetaData::attribute_maxlen + 1);
            }
            else if (strncmp(meta, "Genre=", 6) == 0) {
                strlcpy(meta_data->genre, meta + 6, MetaData::attribute_maxlen + 1);
            }
            else if (strncmp(meta, "TrackNumber=", 12) == 0) {
                meta_data->track = strtoul(&meta[12], nullptr, 10);
            }
            else if (strncmp(meta, "TrackCount=", 11) == 0) {
                meta_data->track = strtoul(&meta[11], nullptr, 10);
            }
            else if (strncmp(meta, "Time(ms)=", 9) == 0) {
                meta_data->duration = strtoul(&meta[9], nullptr, 10);
            }
	        // If we didn't get the full line, need to read and discard the rest
	        while (meta[n - 1] != '\n') {
	            n = serial_.read(meta, '\n', max_attr_line_len, meta_data_line_timeout_ms);
	            if (n == 0)
	                // shouldn't be possible if all lines are '\n' terminated
	                return false;
	        }
	        n = serial_.read(meta, '\n', max_attr_line_len, meta_data_line_timeout_ms);
	    }
        return true;
	}

    /**
	 * Send the reboot command and wait for reboot to complete successfully.
	 *
	 * Assumes device remains in CMD mode.
	 *
	 * The device may also be reset by driving GPIO3 high for 100ms during
	 * runtime.
	 *
     * @return `true` on success, `false` on error.
	 */
	bool actReboot()
	{
	    serial_.write("R,1\r");
        return receivedResponseLine("Reboot!\r\n", 500) && receivedResponseLine("CMD\r\n", 4000);
        // On reboot we receive "Reboot!\r\n" immediately, and then "CMD\r\n"
        // about 2-3s later. At one point we received two nulls "\0\0" between
        // these..
	}

	/**
	 * Await pairing of device when in pairing mode with auth mode
	 * `ssp_keyboard` and return passkey.
	 *
	 * When in pairing mode using the `ssp_keyboard` auth method, once a device
	 * attempts to pair, we receive a 6-digit passkey (which is also displayed
	 * on the device) and may then accept or reject the pairing using
	 * the `basicActionCmd` function.
	 *
     * @param passkey [out] 6-digit passkey. No termination
     * @param timeout_ms timeout in milliseconds.
     *
     * @return false on timeout or error.
	 */
	bool awaitPairingPasskey(char (&passkey)[6], uint32_t timeout_ms)
	{
	    /*
	     * On connection attempt, RN52 outputs the following:
	     * PassKey=123456[\r?]\r\n
	     * Enter accept/reject pairing command![\r?]\r\n
	     */
        char buf[max_line_len];
	    serialReadLineIgnoreMetaData(buf, max_line_len, timeout_ms);
	    if (strncmp(buf, "PassKey=", 8) != 0) {
	        return false;
	    }
        for (uint8_t i = 0; i < 6; i++) {
            passkey[i] = buf[i + 8];
        }
        /*uint8_t i = 0, j = 8;
        while (i < 6) {
            passkey[i++] = buf[j++];
        }*/
	    // read past remaining output
	    serial_.read(buf, '\n', max_line_len, 200); // TODO: timeout
	    return true;
	}

private:
	// TODO: reduce if possible
    static constexpr uint32_t cmd_response_timeout_ms = 2000;
    ISerialIo& serial_;


    /**
     * Read a '\n' terminated line from the UART, ignoring metadata lines.
     *
     * Zero termination is applied in all cases.
     *
     * @param buf
     * @param buf_len maximum number of bytes to read including the '\n'
     *                and zero terminator (i.e. buf size). Must be >= 2.
     * @param timeout_ms
     *
     * @return number of characters read
     */
    uint16_t serialReadLineIgnoreMetaData(char* buf, uint16_t buf_len, uint32_t timeout_ms)
    {
        uint32_t start_time = getMillis();
        bool timed_out = false;
        uint16_t n;
        while (!timed_out) {
            // TODO: a partial metadata response then timeout is not handled
            n = serial_.read(buf, '\n', buf_len - 1, timeout_ms);
            if (n >= 2 && !isMetaData(buf)) {
                buf[n] = '\0';
                return n;
            }
            // Overflow safe
            timed_out = (getMillis() - start_time) > timeout_ms;
        }
        buf[0] = '\0';
        return 0;
    }

    /**
     * Determines whether a given line of text is a metadata line.
     *
     * Only the first two characters are examined.
     *
     * Used to filter unsolicited metadata that is received on track change with
     * most connected devices.
     *
     * @param line MUST be >= 2 chars long.
     *
     * @return true if the line is a metadata item
     */
    static bool isMetaData(char* line)
    {
        /*
         * Metadata fields:
         * ----------------
         * Title=abc\r\r\n
         * Album=abc\r\r\n
         * Artist=abc\r\r\n
         * TrackNumber=0\r\r\n
         * TrackCount=1\r\r\n
         * Genre=abc\r\r\n
         * Time(ms)=358000\r\r\n"
         *
         * Possible non-metadata responses:
         * --------------------------------
         * AOK
         * ERR
         * CMD
         * Reboot!
         * PassKey=123456[\r?]\r\n
         * Enter accept/reject pairing command![\r?]\r\n
         */
        return (line[0] == 'T')                   // Title, TrackNumber, TrackCount, Time
            || (line[0] == 'A' && line[1] != 'O') // Album, Artist, not AOK
            || (line[0] == 'G');                  // Genre
    }

    /**
     *
     * @param expected last two chars must be '\n\0'
     *
     * @return true if `expected` is received over the UART before timeout.
     */
    bool receivedResponseLine(const char* expected)
    {
        return receivedResponseLine(expected, cmd_response_timeout_ms);
    }

    /**
     * Read a line from the UART and determine if it matches an expected
     * string.
     *
     * @param expected last two chars must be '\n\0'
     * @param timeout_ms
     *
     * @return true if `expected` is received over the UART before timeout.
     */
    bool receivedResponseLine(const char* expected, uint32_t timeout_ms)
    {
        char buf[max_line_len];
        serialReadLineIgnoreMetaData(buf, max_line_len, timeout_ms);
        bool success = strcmp(buf, expected) == 0;
        /*if (!success) {
            getErrHndlr().report("ERROR: Expected '%s', got '%s'\r\n", expected, buf);
        }*/
        return success;
    }

    /**
     * @return true if "AOK\r\n" is received over the UART before timeout.
     */
    bool receivedAok()
    {
        return receivedResponseLine("AOK\r\n");
    }
};

} // namespace Libp

#endif /* SRC_RN_52_H_ */
