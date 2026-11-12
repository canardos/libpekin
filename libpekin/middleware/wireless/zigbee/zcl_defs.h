#ifndef LIBPEKIN_NETWORK_ZIGBEE_ZCL_DEFS_H_
#define LIBPEKIN_NETWORK_ZIGBEE_ZCL_DEFS_H_

#include <cstdint>

namespace libp::zcl {

constexpr uint8_t max_ep = 240;
constexpr uint8_t max_scene_id = 0xff;
constexpr uint16_t max_group_id = 0xfff7;
constexpr uint8_t max_lvl_ctrl_lvl = 254;

namespace mfr {
    constexpr uint16_t test_1 = 0xfff1;
    constexpr uint16_t test_2 = 0xfff2;
    constexpr uint16_t test_3 = 0xfff3;
    constexpr uint16_t test_4 = 0xfff4;
    constexpr uint16_t dk = test_4;
} // namespace mfr_id


/**
 * Non-exhaustive list of ZCL cluster IDs, including custom.
 */
namespace cluster {
    constexpr uint16_t device_temperature_config = 0x0002;
    constexpr uint16_t scenes                    = 0x0005;
    constexpr uint16_t onoff                     = 0x0006;
    constexpr uint16_t oosc                      = 0x0007;
    constexpr uint16_t level                     = 0x0008;
    constexpr uint16_t level_ctrl_for_lighting   = 0x0008;
    constexpr uint16_t multistate_input          = 0x0012;
    constexpr uint16_t window_covering           = 0x0102;
    constexpr uint16_t diagnostics               = 0x0b05;

    // CUSTOM
    constexpr uint16_t dk_dev_config = 0xfc10;
    constexpr uint16_t dk_dev_status = 0xfc11;
} // namespace cluster


namespace attr {
    constexpr uint16_t multistate_input_pv = 0x0055;

    // OnOff cluster custom attributes
    constexpr uint16_t onoff_dk_auto_off_delay_secs = 0xf000;

    // OOSC cluster custom attributes
    constexpr uint16_t oosc_dk_max_taps           = 0xf000;
    constexpr uint16_t oosc_dk_tap1_mode          = 0xf010;
    constexpr uint16_t oosc_dk_tap2_mode          = 0xf011;
    constexpr uint16_t oosc_dk_tap3_mode          = 0xf012;
    constexpr uint16_t oosc_dk_script_tap1        = 0xf020;
    constexpr uint16_t oosc_dk_script_tap2        = 0xf021;
    constexpr uint16_t oosc_dk_script_tap3        = 0xf022;
    constexpr uint16_t oosc_dk_script_hold1       = 0xf028;
    constexpr uint16_t oosc_dk_script_hold2       = 0xf029;
    constexpr uint16_t oosc_dk_script_hold3       = 0xf02a;
    constexpr uint16_t oosc_dk_script_en_tap1     = 0xf030;
    constexpr uint16_t oosc_dk_script_en_tap2     = 0xf031;
    constexpr uint16_t oosc_dk_script_en_tap3     = 0xf032;
    constexpr uint16_t oosc_dk_script_en_hold1    = 0xf038;
    constexpr uint16_t oosc_dk_script_en_hold2    = 0xf039;
    constexpr uint16_t oosc_dk_script_en_hold3    = 0xf03a;
    constexpr uint16_t oosc_dk_switches_load_tap1 = 0xf040;
    constexpr uint16_t oosc_dk_switches_load_tap2 = 0xf041;
    constexpr uint16_t oosc_dk_switches_load_tap3 = 0xf042;

    // Custom Device Config cluster attributes
    constexpr uint16_t dk_dev_config_max_time_btn_taps_ms = 0xf000;
    constexpr uint16_t dk_dev_config_min_time_for_hold_ms = 0xf001;
    constexpr uint16_t dk_dev_config_on_off_led_type      = 0xf010;
    constexpr uint16_t dk_dev_config_on_off_led_color     = 0xf011;
    constexpr uint16_t dk_dev_config_led_on_brightness    = 0xf012;
    constexpr uint16_t dk_dev_config_led_off_brightness   = 0xf013;
    constexpr uint16_t dk_dev_config_min_delay_btn_cmds   = 0xf020;
    constexpr uint16_t dk_dev_config_ota_block_delay      = 0xf021;
    constexpr uint16_t dk_dev_config_tx_power_level       = 0xf030;

    // Custom Device Status cluster attributes
    constexpr uint16_t dk_dev_status_uptime          = 0xf000;
    constexpr uint16_t dk_dev_status_status          = 0xf001;
    constexpr uint16_t dk_dev_status_status_ext      = 0xf002;
    constexpr uint16_t dk_dev_status_rx_crc_pct_fail = 0xf100;
}


namespace cmd {
    constexpr uint8_t onoff_off    = 0x00;
    constexpr uint8_t onoff_on     = 0x01;
    constexpr uint8_t onoff_toggle = 0x02;

    constexpr uint8_t level_move_to_lvl       = 0x00;
    constexpr uint8_t level_move_to_lvl_onoff = 0x04;

    constexpr uint8_t windowc_up_open    = 0x00;
    constexpr uint8_t windowc_down_close = 0x01;
    constexpr uint8_t windowc_stop       = 0x02;

    constexpr uint8_t scenes_recall_scene = 0x05;
} // namespace cmd


enum class DataType: uint8_t {
    no_data = 0x00,
    data8 = 0x08,
    data16 = 0x09,
    data24 = 0x0a,
    data32 = 0x0b,
    data40 = 0x0c,
    data48 = 0x0d,
    data56 = 0x0e,
    data64 = 0x0f,
    boolean = 0x10,
    bitmap8 = 0x18,
    bitmap16 = 0x19,
    bitmap24 = 0x1a,
    bitmap32 = 0x1b,
    bitmap40 = 0x1c,
    bitmap48 = 0x1d,
    bitmap56 = 0x1e,
    bitmap64 = 0x1f,
    uint8 = 0x20,
    uint16 = 0x21,
    uint24 = 0x22,
    uint32 = 0x23,
    uint40 = 0x24,
    uint48 = 0x25,
    uint56 = 0x26,
    uint64 = 0x27,
    int8 = 0x28,
    int16 = 0x29,
    int24 = 0x2a,
    int32 = 0x2b,
    int40 = 0x2c,
    int48 = 0x2d,
    int56 = 0x2e,
    int64 = 0x2f,
    // TODO: underscore is to avoid conflict with #define
    enum_8 = 0x30,
    enum_16 = 0x31,
    semi_prec = 0x38,
    single_prec = 0x39,
    double_prec = 0x3a,
    octet_str = 0x41,
    char_str = 0x42,
    long_octet_str = 0x43,
    long_char_str = 0x44,
    array = 0x48,
    a_struct = 0x4c,
    set = 0x50,
    bag = 0x51,
    tod = 0xe0,
    date = 0xe1,
    utc = 0xe2,
    cluster_id = 0xe8,
    attr_id = 0xe9,
    bac_oid = 0xea,
    // eui64
    ieee_addr = 0xf0,
    ec_key_128_bit = 0xf1,
    unknown = 0xff,
};

/**
 * ZCL rev. 8 status enumeration
 */
enum class Status : uint8_t  {
    success = 0x00,               /**< success */
    failure = 0x01,               /**< failure */
    not_authorized = 0x7E,        /**< not_authorized */
    malformed_command = 0x80,     /**< malformed command: wrong/missing fields etc. */
    unsup_command = 0x81,         /**< unsup_command */
    // Deprecated
    //unsup_general_command = 0x82,
    //unsup_manuf_cluster_command = 0x83,
    //unsup_manuf_general_command = 0x84,
    invalid_field = 0x85,         /**< invalid_field */
    unsupported_attribute = 0x86, /**< unsupported_attribute */
    invalid_value = 0x87,         /**< invalid_value */
    read_only = 0x88,             /**< attempted write to read only attribute */
    insufficient_space = 0x89,    /**< insufficient_space */
    // Deprecated
    //duplicate_exists = 0x8A,
    not_found = 0x8B,             /**< not_found */
    unreportable_attribute = 0x8C,/**< unreportable_attribute */
    invalid_data_type = 0x8D,     /**< incorrect attribute data type */
    invalid_selector = 0x8E,      /**< incorrect attribute selected */
    // Deprecated
    //write_only = 0x8F,
    //inconsistent_startup_state = 0x90,
    //defined_out_of_band = 0x91,
    //action_denied = 0x93,
    timeout = 0x94,               /**< timeout */
    abort = 0x95,                 /**< OTA abort */
    invalid_image = 0x96,         /**< invalid OTA image */
    wait_for_data = 0x97,         /**< server doesn't have block available yet */
    no_image_available = 0x98,    /**< no OTA image available for client */
    require_more_image = 0x99,    /**< more OTA images files needed */
    notification_pending = 0x9A,  /**< command received and still processing */
    // Deprecated
    //hardware_failure = 0xC0,
    //software_failure = 0xC1,
    unsupported_cluster = 0xC3,   /**< unsupported_cluster */
    // Deprecated
    //limit_reached = 0xC4,
};

} // namespace libp::zcl

#endif // LIBPEKIN_NETWORK_ZIGBEE_ZCL_DEFS_H_
