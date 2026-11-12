#ifndef SRC_USER_LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_UTILS_H_
#define SRC_USER_LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_UTILS_H_

#include "middleware//wireless/zigbee/zcl_defs.h"
#include <cstdint>

namespace libp::zcl {



    /**
     * Copy a ZCL string between two buffers.
     *
     * Read : local  -> remote
     * WRite: remote -> local
     *
     * @param op
     * @param local
     * @param local_len including leading size byte(s)
     * @param remote
     * @param remote_len including leading size byte(s)
     *
     * @return
     */
    //sl_zigbee_af_status_t readWriteString(ZclOp op, uint8_t* local, uint16_t local_len, uint8_t* remote, uint16_t remote_len);



/*sl_zigbee_af_status_t zcl::readWriteString(ZclOp op, uint8_t* local, uint16_t local_len, uint8_t* remote, uint16_t remote_len)
{
    uint8_t* src;
    uint8_t* dst;
    uint16_t dst_len;

    if (op == ZclOp::read) {
        src = local;
        dst = remote;
        dst_len = remote_len;
    }
    else {
        src = remote;
        dst = local;
        dst_len = local_len;
    }
    uint16_t zcl_str_len = sl_zigbee_af_string_length(src) + 1;
    if (zcl_str_len > dst_len) {
        return op == ZclOp::read ? SL_ZIGBEE_ZCL_STATUS_INSUFFICIENT_SPACE : SL_ZIGBEE_ZCL_STATUS_INVALID_VALUE;
    }
    sl_zigbee_af_copy_string(dst, src, dst_len - 1);
    return SL_ZIGBEE_ZCL_STATUS_SUCCESS;
}*/


/**
 * Return the length of a ZCL character string.
 *
 * @param p_data ZCL character string
 *
 * @return
 */
inline constexpr uint8_t charStringLen(uint8_t* p_data)
{
    // First byte is string length
    uint8_t len = p_data[0];
    // 0x00 AND 0xff leading byte both specify an empty string
    return len == 0xff
            ? 0x00
            : len;
}

inline constexpr uint8_t dataTypeLen(DataType type)
{
    switch(type) {
    case DataType::boolean:
    case DataType::data8:
    case DataType::bitmap8:
    case DataType::int8:
    case DataType::uint8:
    case DataType::enum_8:
        return 1;
    case DataType::enum_16:
    case DataType::data16:
    case DataType::bitmap16:
    case DataType::uint16:
    case DataType::int16:
        return 2;
    case DataType::data32:
    case DataType::bitmap32:
    case DataType::uint32:
    case DataType::int32:
        return 4;
    case DataType::data64:
    case DataType::bitmap64:
    case DataType::uint64:
    case DataType::int64:
    case DataType::ieee_addr:
        return 8;
    case DataType::ec_key_128_bit:
        return 16;

    //TODO:
    case DataType::semi_prec:
    case DataType::single_prec:
    case DataType::double_prec:

    case DataType::octet_str:
    case DataType::char_str:
    case DataType::long_octet_str:
    case DataType::long_char_str:

    case DataType::array:
    case DataType::a_struct:
    case DataType::set:
    case DataType::bag:
    case DataType::tod:
    case DataType::date:
    case DataType::utc:
    case DataType::cluster_id:
    case DataType::attr_id:
    case DataType::bac_oid:
    case DataType::data40:
    case DataType::data48:
    case DataType::data56:
    case DataType::data24:
    case DataType::bitmap24:
    case DataType::bitmap40:
    case DataType::bitmap48:
    case DataType::bitmap56:
    case DataType::uint24:
    case DataType::uint40:
    case DataType::uint48:
    case DataType::uint56:
    case DataType::int24:
    case DataType::int40:
    case DataType::int48:
    case DataType::int56:
    case DataType::unknown:
    case DataType::no_data:
        return 0;
    default:
        return 0;
    }
}

} // namespace libp::zcl


#endif /* SRC_USER_LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_UTILS_H_ */
