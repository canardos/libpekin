#ifndef LIBPEKIN_NETWORK_ZIGBEE_ZCL_ATTRIBUTE_STORE_H_
#define LIBPEKIN_NETWORK_ZIGBEE_ZCL_ATTRIBUTE_STORE_H_

#include "middleware//wireless/zigbee/zcl_defs.h"
#include "middleware//wireless/zigbee/zcl_utils.h"
#include "lp_buffers.h"
#include "lp_types.h"
#include "lp_logging.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <bit>

#define DBG_PREFIX "attr_store: "

namespace libp::zcl {

struct ZclAttribute {
    enum class Op {
        read, write
    };

    constexpr ZclAttribute() = default;
    /** read-only attribute constructor */
    constexpr ZclAttribute(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, DataType data_type, void* p_data)
            : ZclAttribute(ep, cluster_id, attr_id, data_type, p_data, false, 0, UINT32_MAX) {}
    template <typename T1, typename T2>
    constexpr ZclAttribute(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, DataType data_type, void* p_data, bool writable, T1 min_val, T2 max_val)
            : p_data(p_data),
              min(static_cast<uint32_t>(min_val)),
              max(static_cast<uint32_t>(max_val)),
              cluster_attr_id( makeClustAttrId(cluster_id, attr_id) ),
              ep(ep),
              data_type(data_type),
              writable(writable) {
        switch (data_type) {
        case DataType::int8:
        case DataType::enum_8:
        case DataType::uint8:
        case DataType::boolean:
        case DataType::uint16:
        case DataType::uint32:
        case DataType::char_str:
            break;
        default:
            LP_ASSERT(false);
        }
    }

    void* p_data;
    /** min value or min length of character type */
    uint32_t min;
    /** max value or max length of character type */
    uint32_t max;
    uint32_t cluster_attr_id;

    uint8_t ep;
    DataType data_type;
    bool writable;

    [[gnu::always_inline]] inline
    static constexpr uint32_t makeClustAttrId(uint16_t cluster_id, uint16_t attr_id)
    {
        return (uint32_t)cluster_id << 16 | attr_id;
    }

    /**
     * Copy attribute value into provided buffer.
     *
     * @param p_dest
     * @param len size of p_dest.
     *
     * @return false if len is too small.
     */
    Status read(uint8_t* p_dest, uint16_t len) const
    {
        uint16_t n_bytes_to_copy = data_type == DataType::char_str
                ? charStringLen(static_cast<uint8_t*>(p_data)) + 1
                : dataTypeLen(data_type);
        if (len < n_bytes_to_copy) {
            LP_LOG_ERROR_IF(EN_LOG_ZCL_GEN, DBG_PREFIX"Buffer too small for data (%u vs %u)", len, n_bytes_to_copy);
            return Status::failure;
        }
        static_assert(std::endian::native == std::endian::little, "Only LE platform currently supported");
        memcpy(p_dest, p_data, n_bytes_to_copy);
        return Status::success;
    }

    /**
     * Update attribute value.
     *
     * @param p_src
     * @param len
     *
     * @return
     */
    Status write(uint8_t* p_src, uint16_t len) const
    {
        if (!writable) {
            return Status::read_only;
        }
        if (!valueIsValid(p_src, len)) {
            return Status::invalid_value;
        }
        uint16_t n_bytes_to_copy = data_type == DataType::char_str
                ? charStringLen(p_src) + 1
                : dataTypeLen(data_type);
        memcpy(p_data, p_src, n_bytes_to_copy);
        return Status::success;
    }
private:
    bool valueIsValid(uint8_t* p_value, uint16_t len) const
    {
        if (data_type != DataType::char_str && len < dataTypeLen(data_type)) {
            return false;
        }
        uint32_t value;
        switch (data_type) {
        case DataType::int8: {
            int8_t signed_val = static_cast<int8_t>(p_value[0]);
            return signed_val >= static_cast<int32_t>(min) &&
                   signed_val <= static_cast<int32_t>(max);
        }
        case DataType::enum_8:
        case DataType::uint8:
        case DataType::boolean:
            value = p_value[0];
            break;
        case DataType::uint16:
            value = libp::readLEndU16(p_value);
            break;
        case DataType::uint32:
            value = libp::readLEndU32(p_value);
            break;
        case DataType::char_str:
            value = charStringLen(p_value);
            if (len < (value + 1)) {
                LP_LOG_ERROR_IF(EN_LOG_ZCL_GEN, DBG_PREFIX"Buffer too small for string");
                // buffer doesn't have enough space for claimed str length
                return false;
            }
            break;
        default:
            // unsupported
            LP_LOG_ERROR_IF(EN_LOG_ZCL_GEN, DBG_PREFIX"Unsupported attr type: %u", libp::enumVal(data_type));
            return false;
        }
        return value >= min && value <= max;
    }
};


template <auto& Attr>
requires std::is_same_v<typename std::remove_cvref_t<decltype(Attr)>::value_type, ZclAttribute>
class AttributeStore {

public:
    libp::zcl::Status readWrite(ZclAttribute::Op op, uint8_t ep, uint16_t cluster_id, uint16_t attr_id, uint8_t *buf, uint16_t buf_len)
    {
        if (buf == nullptr || buf_len == 0) {
            LP_LOG_ERROR_IF(EN_LOG_ZCL_GEN, DBG_PREFIX"Nullptr/0-len buffer");
            return op == ZclAttribute::Op::write
                    ? Status::invalid_value
                    : Status::failure;
        }
        const ZclAttribute* attr = findAttr(ep, cluster_id, attr_id);
        if (attr == nullptr) {
            return Status:: unsupported_attribute;
        }
        return op == ZclAttribute::Op::write
                ? attr->write(buf, buf_len)
                : attr->read(buf, buf_len);
    }

private:
    /**
     * @return nullptr if attribute is not found
     */
    static const ZclAttribute* findAttr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id)
    {
        const uint32_t cluster_attr_id = ZclAttribute::makeClustAttrId(cluster_id, attr_id);
        for (const auto& attr : Attr) {
            if (ep == attr.ep && cluster_attr_id == attr.cluster_attr_id) {
                return &attr;
            }
        }
        LP_LOG_ERROR_IF(EN_LOG_ZCL_GEN, DBG_PREFIX"Attr not found for ep/clust/attr = %u/0x%04x/0x%04x", ep, cluster_id, attr_id);
        return nullptr;
    }
};

} // namespace libp::zcl

#undef DBG_PREFIX

#endif /* LIBPEKIN_NETWORK_ZIGBEE_ZCL_ATTRIBUTE_STORE_H_ */
