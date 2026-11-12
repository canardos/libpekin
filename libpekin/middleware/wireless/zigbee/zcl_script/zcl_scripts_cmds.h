/*
 * zcl_scripts_cmds.h
 *
 *  Created on: 23 Oct 2026
 *      Author: bob
 */

#ifndef LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_CMDS_H_
#define LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_CMDS_H_


#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_defs.h"
#include "middleware//wireless/zigbee/zcl_script/tuya.h"

#include "middleware//wireless/zigbee/zcl_defs.h"

#include "lp_buffers.h"
#include "lp_types.h"

#include <cstdint>

namespace libp::zcl_script {

/**
 * Data required to send a generic ZCL command.
 *
 * May also represent a local device delay request. See \ref isLocalDelay()
 */
struct ZclCommand { // 40-bytes after padding
    static constexpr uint8_t max_payload_len = 16;

    /** IEEE or local 16-bit address */
    uint64_t dst_addr;

    uint16_t group = unset_group;

    AddrType addr_type = AddrType::not_set; // 1-byte

    uint8_t src_ep;
    uint8_t dst_ep;

    uint8_t cmd_id;
    uint16_t mfr_id;
    uint16_t cluster_id = unset_cluster_id;

    uint8_t payload[max_payload_len];
    uint8_t payload_len;

    /**
     * Default cluster_id value to check if it's been set, since 0 is a valid
     * id, but 0x8000 is not.
     */
    static constexpr uint16_t unset_cluster_id = 0x8000;
    /**
     * Default group value to check if it's been set, since 0 is valid in some
     * circumstances.
     */
    static constexpr uint16_t unset_group = 0xffff;
    /**
     * Signifies this is a local delay device command command and NOT a network
     * command. The delay (ms) is stored in the first two bytes of the payload.
     * Use \ref getLocalDelayMs() to get delay.
     */
    static constexpr uint16_t cluster_id_local_delay = 0x8001;

    constexpr bool isLocalDelay() const {
        return cluster_id == cluster_id_local_delay;
    }
    constexpr uint16_t getLocalDelayMs() const {
        return readLEndU16(payload);
    }
};


/**
 * This struct is used to store the initial parse result for a single command
 * in a script.
 *
 * It also contains a .zcl ZclCommand member, which will contain the final ZCL
 * command after initial parse + command conversion.
 *
 * The parse is a 2-step process. Example flow:
 * 1) "on,a1234,e1" is parsed into command (on), address, and endpoint.
 * 2) This is then converted to a general ZCL command, setting the
 *    cluster/payload etc.
 */
struct ScriptCmd {
    ZclCommand zcl;

    //
	// Below is non-ZCL command data.
    // It stores parsed script params temporarily before they are moved to
    // the payload during processing (see scriptCmdToZclCommand)
	//

	Cmd cmd;  // 2-bytes
	uint32_t value; // Tuya value/delay

	uint8_t scene;
	tuya::DataType tuya_type = tuya::DataType::invalid; // 1-byte
	tuya::DataPoint tuya_dp; // 1-byte

    enum class Error : uint8_t {
        valid = 0,
        missing_dst_ep,
        missing_group,
        missing_cluster_id,
        missing_cmd,
        missing_tuya_dp,
        missing_tuya_type,
        missing_payload,
        invalid_addr,
        invalid_addr_type,
        invalid_group,
        invalid_level,
        invalid_tuya_value,
        invalid_delay,
        unexpected_addr,
        unexpected_group,
        unexpected_dst_ep,
        unexpected_cluster_id,
        unexpected_cmd_id,
        unexpected_mfr_id,
        unexpected_payload,
        unexpected_value,
        unexpected_scene,
        unexpected_tuya_dp,
        unexpected_tuya_dt,
        payload_too_long,
        LAST
    };
    static constexpr const char* cmd_error_desc[] = {
        "", // valid
        "mis dstEP",
        "mis grp",
        "mis clustID",
        "mis cmd",
        "mis tuyaDp",
        "mis tuyaTyp",
        "mis pload",
        "inv addr",
        "inv addrTyp",
        "inv grp",
        "inv lvl",
        "inv tuyaVal",
        "inv delay",
        "unxpt addr",
        "unxpt grp",
        "unxpt dstEP",
        "unxpt clustID",
        "unxpt cmdID",
        "unxpt mfrID",
        "unxpt pload",
        "unxpt val",
        "unxpt scn",
        "unxpt tuyaDp",
        "unxpt tuyaTyp",
        "pload 2 long",
    };
    static_assert( (sizeof(cmd_error_desc) / sizeof(cmd_error_desc[0])) == libp::enumVal(Error::LAST) );
    static inline constexpr const char* errorToDesc(Error error) {
    	uint8_t idx = libp::enumVal(error);
        return idx < (sizeof(cmd_error_desc) / sizeof(cmd_error_desc[0]))
                ? cmd_error_desc[idx]
                : "unknown";
    }

    /**
     * Validate a ScriptCmd object.
     *
     * @return Error::valid if the command is had the required parameters and
     * their values are valid.
     */
    Error validate() const
    {
        //===============================================
        // Check for extra params
        //===============================================
        switch(cmd) {
            case Cmd::recall_scene:
            case Cmd::on:
            case Cmd::off:
            case Cmd::toggle:
            case Cmd::up:
            case Cmd::down:
            case Cmd::stop:
                if (value != 0) {
                    return Error::unexpected_value;
                }
                [[fallthrough]];
            case Cmd::mv_to_lvl:
            case Cmd::mv_to_lvl_onoff:
            case Cmd::delay:
                if (tuya_dp != tuya::DataPoint::invalid) {
                    return Error::unexpected_tuya_dp;
                }
                [[fallthrough]];
            case Cmd::tuya_set_dp_u32:
            case Cmd::tuya_set_dp_enum:
            case Cmd::tuya_set_dp_bool:
                if (tuya_type != tuya::DataType::invalid) { return Error::unexpected_tuya_dt; }
                if (zcl.payload_len != 0)                 { return Error::unexpected_payload; }
                [[fallthrough]];
            case Cmd::tuya_set_dp_raw:
                if (zcl.cluster_id != ZclCommand::unset_cluster_id) { return Error::unexpected_cluster_id; }
                if (scene != 0 && cmd != Cmd::recall_scene)         { return Error::unexpected_scene;      }
                if (zcl.cmd_id != 0)                                { return Error::unexpected_cmd_id;     }
                if (zcl.mfr_id != 0)                                { return Error::unexpected_mfr_id;     }
                break;
            case Cmd::zcl_general:
                if (value != 0)                             { return Error::unexpected_value;   }
                if (scene != 0 && cmd != Cmd::recall_scene) { return Error::unexpected_scene;   }
                if (tuya_type != tuya::DataType::invalid)   { return Error::unexpected_tuya_dt; }
                if (tuya_dp != tuya::DataPoint::invalid)    { return Error::unexpected_tuya_dp; }
                break;
            default:
                return Error::missing_cmd;
        }
        //
        // Delay only requires value and has no addressing
        //
        if (cmd == Cmd::delay) {
        	if (zcl.dst_addr != 0)                              { return Error::unexpected_addr;      }
        	if (zcl.group != ZclCommand::unset_group)           { return Error::unexpected_group;     }
        	if (zcl.addr_type != AddrType::not_set)             { return Error::unexpected_addr;      }
        	if (zcl.dst_ep != 0)                                { return Error::unexpected_dst_ep;    }
            if (zcl.cmd_id != 0)                                { return Error::unexpected_cmd_id;    }
            if (zcl.mfr_id != 0)                                { return Error::unexpected_mfr_id;    }
            if (zcl.cluster_id != ZclCommand::unset_cluster_id) { return Error::unexpected_cluster_id;}
            if (scene != 0 && cmd != Cmd::recall_scene)         { return Error::unexpected_scene;     }
            if (tuya_type != tuya::DataType::invalid)           { return Error::unexpected_tuya_dt;   }
            if (tuya_dp != tuya::DataPoint::invalid)            { return Error::unexpected_tuya_dp;   }
            if (zcl.payload_len != 0)                           { return Error::unexpected_payload;   }
            return (value == 0 || value > 0xffff)
                ? Error::invalid_delay
                : Error::valid;
        }
        //===============================================
        // Check for illegal values
        //===============================================
        if (zcl.payload_len > ZclCommand::max_payload_len) { return Error::payload_too_long; }
        //===============================================
        // Check Addressing
        //===============================================

        if (zcl.group != ZclCommand::unset_group &&
            zcl.addr_type != AddrType::group     &&
            cmd != Cmd::recall_scene) {
            return Error::unexpected_group;
        }
        switch (zcl.addr_type) {

            case AddrType::local:
                if (zcl.dst_ep == 0)       { return Error::missing_dst_ep;   }
                if (zcl.dst_addr > 0xfff7) { return Error::invalid_addr;     }
                // 0x0000 is a valid 16-bit address
                break;
            case AddrType::ieee64:
                if (zcl.dst_ep == 0)   { return Error::missing_dst_ep; }
                if (zcl.dst_addr == 0) { return Error::invalid_addr;   }
                break;
            case AddrType::group:
                if (zcl.group == ZclCommand::unset_group) {
                    return Error::missing_group;
                }
                // group 0 is valid for recall scene in unicast only
                if (zcl.group == 0 || zcl.group > libp::zcl::max_group_id) {
                    return Error::invalid_group;
                }
                [[fallthrough]];
            case AddrType::broadcast:
            case AddrType::bound:
                if (zcl.dst_addr != 0) { return Error::unexpected_addr;   }
                if (zcl.dst_ep != 0)   { return Error::unexpected_dst_ep; }
                break;
            case AddrType::not_set:
                return Error::invalid_addr;
                break;
            }
        //===============================================
        // Check for missing (non addr) params
        //===============================================
        switch(cmd) {
            // recall scene group is required for groupcast
            // and optional for unicast. If omitted in unicast it
            // is set to 0 on conversion to ZclCommand
            case Cmd::recall_scene:
                switch (zcl.addr_type) {
                    case AddrType::not_set:
                    case AddrType::broadcast:
                    case AddrType::bound:
                        return Error::invalid_addr_type;
                    case AddrType::group:
                    case AddrType::local:
                    case AddrType::ieee64:
                        break;
                }
                // scene ID 0->0xFF is valid so can't test for presence.
                [[fallthrough]];
            case Cmd::on:
            case Cmd::off:
            case Cmd::toggle:
            case Cmd::mv_to_lvl:
            case Cmd::mv_to_lvl_onoff:
            case Cmd::up:
            case Cmd::down:
            case Cmd::stop:
                break;

            case Cmd::tuya_set_dp_raw:
                if (tuya_type == tuya::DataType::invalid) {
                    return Error::missing_tuya_type;
                }
                [[fallthrough]];
            case Cmd::tuya_set_dp_u32:
            case Cmd::tuya_set_dp_enum:
            case Cmd::tuya_set_dp_bool:
            	// TODO: test tuya broadcast to see if it works
            	if (/*zcl.addr_type == AddrType::broadcast ||*/
					zcl.addr_type == AddrType::bound) {
                    return Error::invalid_addr_type;
            	}
                if (tuya_dp == tuya::DataPoint::invalid) {
                    return Error::missing_tuya_dp;
                }
                break;

            case Cmd::zcl_general:
                // cmd_id==0 and no payload are valid
                if (zcl.cluster_id == ZclCommand::unset_cluster_id) {
                    return Error::missing_cluster_id;
                }
                break;
            case Cmd::delay:
                break;
        }
        //===============================================
        // Check for invalid value field
        //===============================================
        switch (cmd) {
            case Cmd::mv_to_lvl:
            case Cmd::mv_to_lvl_onoff:
                if (value > zcl::max_lvl_ctrl_lvl) {
                    return Error::invalid_level;
                }
                break;
            case Cmd::tuya_set_dp_enum:
                if (value > 0xff) {
                    return Error::invalid_tuya_value;
                }
                break;
            case Cmd::tuya_set_dp_bool:
                if (value > 0x01) {
                    return Error::invalid_tuya_value;
                }
                break;
            case Cmd::tuya_set_dp_u32:
                // value is u32
                break;
            case Cmd::tuya_set_dp_raw:
                if (zcl.payload_len == 0) {
                    return Error::missing_payload;
                }
                if (value > 0) {
                    return Error::unexpected_value;
                }
                break;
            case Cmd::delay:
                // already checked in short circuit path above
            default:
                break;
        }

        return Error::valid;
    }
};

} // namespace libp::zcl_script

#endif /* LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_CMDS_H_ */
