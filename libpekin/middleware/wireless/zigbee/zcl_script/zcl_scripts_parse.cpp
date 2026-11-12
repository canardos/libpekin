
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_parse.h"

#include "libpekin.h"
#include "lp_string.h"
#include "lp_logging.h"
#include "lp_types.h"
#include "lp_buffers.h"
#include <cstring>
#include <cstdint>


using namespace libp;
using namespace libp::zcl_script;

/**
 * Return position of next ',' | ';' delimiter or len if not found.
 *
 * @return exclusive
 */
static constexpr uint16_t findEndOfToken(const char *str, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
		if (str[i] == delim_token || str[i] == delim_script) {
			return i;
		}
	}
	return len;
}

static_assert(findEndOfToken("1,", 2) == 1);
static_assert(findEndOfToken("1,33", 2) == 1);
static_assert(findEndOfToken("1,33", 4) == 1);
static_assert(findEndOfToken("12", 2) == 2);
static_assert(findEndOfToken("12;a", 4) == 2);
static_assert(findEndOfToken("s1234", 5) == 5);


internal::ParseResult libp::zcl_script::internal::parseScriptCmdFirstPass(const char *p_script, uint16_t len, uint16_t *p_idx, ScriptCmd* p_cmd)
{
    uint16_t i = *p_idx;
	if ((len - i) < 2 || p_script == nullptr) {
		return ParseResult::invalid_command;
	}
	*p_cmd = {};
	ZclCommand* p_zcl = &p_cmd->zcl;
	Cmd cmd = static_cast<Cmd>(dblCharToU16(&p_script[i]));

	switch(cmd) {
	case Cmd::recall_scene:
	case Cmd::on:
	case Cmd::off:
	case Cmd::toggle:
	case Cmd::mv_to_lvl:
	case Cmd::mv_to_lvl_onoff:
	case Cmd::up:
	case Cmd::down:
	case Cmd::stop:
	case Cmd::tuya_set_dp_u32:
	case Cmd::tuya_set_dp_enum:
	case Cmd::tuya_set_dp_bool:
	case Cmd::tuya_set_dp_raw:
	case Cmd::zcl_general:
    case Cmd::delay:
		p_cmd->cmd = cmd;
		break;
	default:
		return ParseResult::invalid_command;
	}
	i += sizeof(cmd);

	// -- Parse all params

	while (i < len && p_script[i] != delim_script) {

		if (p_script[i++] != delim_token) {
			*p_idx  = i - 1;
			return ParseResult::missing_delim;
		}
		uint16_t value_len = findEndOfToken(p_script + i, len - i) - 1;
		// if strings ends in ',' value_len = 0 - 1
		if (value_len == 0xffff) {
            return ParseResult::invalid_param;
		}
		// get param
		CmdParam param = static_cast<CmdParam>(p_script[i++]);

		// Handle broadcast/bound separately as nstrToU64 will fail
		if (param == CmdParam::dst_addr && value_len == 1) {
            switch(p_script[i]) {
                case addr_sym_broadcast:
                    p_zcl->addr_type = AddrType::broadcast;
                    i += value_len;
                    continue;
                case addr_sym_bound:
                    p_zcl->addr_type = AddrType::bound;
                    i += value_len;
                    continue;
            }
		}
		// Parse param values <= 64-bit
		uint64_t val;
		bool invalid_val;
		// Don't parse payload as it's > 64-bit - parse below
		if (param != CmdParam::payload) {
	        val = nstrToU64(&p_script[i], value_len);
	        invalid_val = val == nstrToU64_invalid;
	        i += value_len;
		}

		switch(param) {

		case CmdParam::dst_addr:
			if (invalid_val) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_addr;
			}
			p_zcl->dst_addr = val;
			p_zcl->addr_type = val > 0xffff
			        ? AddrType::ieee64
                    : AddrType::local;
			break;

		case CmdParam::group:
			if (invalid_val || val > zcl::max_group_id) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_group;
			}
			p_zcl->group = val;
			// recall scene etc may include group AND unicast address
            if (p_zcl->addr_type == AddrType::not_set) {
                p_zcl->addr_type = AddrType::group;
            }
			break;

		case CmdParam::scene:
			if (invalid_val || val > zcl::max_scene_id) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_scene;
			}
			p_cmd->scene = val;
			break;

		case CmdParam::dst_ep:
			if (invalid_val || val > zcl::max_ep) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_dst_ep;
			}
			p_zcl->dst_ep = val;
			break;

		case CmdParam::cluster_id:
			if (invalid_val || val > 0xffff) { // max valid cluster_id
				*p_idx  = i - value_len;
				return ParseResult::invalid_cluster_id;
			}
			p_zcl->cluster_id = val;
			break;

		case CmdParam::cmd_id:
			if (invalid_val || val > 0xff) { // max valid cmd
				*p_idx  = i - value_len;
				return ParseResult::invalid_cmd_id;
			}
			p_zcl->cmd_id = val;
			break;

		case CmdParam::mfr_id:
			if (invalid_val || val > 0xffff) { // max valid cmd
				*p_idx  = i - value_len;
				return ParseResult::invalid_mfr_id;
			}
			p_zcl->mfr_id = val;
			break;

		case CmdParam::tuya_dp:
			if (invalid_val || val > 0xff) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_tuya_dp;
			}
			p_cmd->tuya_dp = static_cast<tuya::DataPoint>(val);
			break;

		case CmdParam::tuya_dt:
			if (invalid_val || val > 0xff || val == libp::enumVal(tuya::DataType::invalid)) {
				*p_idx  = i - value_len;
				return ParseResult::invalid_tuya_type;
			}
			p_cmd->tuya_type = static_cast<tuya::DataType>(val);
			break;

		case CmdParam::value:
		    // max is u32 for tuya u32 or delay ms
            if (invalid_val || val > 0xffff'ffff) {
                *p_idx  = i - value_len;
                return ParseResult::invalid_value;
            }
            p_cmd->value = val;
		    break;

		case CmdParam::payload: {
			uint16_t payload_end = i + value_len;
			// Check payload len
            uint8_t allowed_payload_len = cmd == Cmd::tuya_set_dp_raw
                    ? ZclCommand::max_payload_len - tuya_payload_prefix_len
                    : ZclCommand::max_payload_len;
            if (value_len > allowed_payload_len*2) {
                *p_idx  = i;
                return ParseResult::payload_too_long;
            }
            // Check payload format
			if (value_len & 1) { // payload must be hex pairs
				*p_idx  = i;
				return ParseResult::invalid_payload;
			}
			// Parse the payload
			uint8_t payload_i = 0;
			while (i < payload_end) {
			    if (!libp::isHex(p_script[i]) || !libp::isHex(p_script[i+1])) {
					*p_idx  = i;
	                return ParseResult::invalid_payload;
			    }
			    p_zcl->payload[payload_i]    = libp::atoh(p_script[i++]) << 4;
			    p_zcl->payload[payload_i++] |= libp::atoh(p_script[i++]);
			}
			p_zcl->payload_len = payload_i;
			break;
		}
		default:
            *p_idx  = i - value_len - 1;
			return ParseResult::invalid_param;
		}
	} // while (i < len && p_script[i] != delim_script)
	*p_idx  = (i < len) ? i + 1 : i;

	return ParseResult::success;
}

static constexpr uint8_t scriptCmdToZclCmdId(Cmd cmd)
{
	switch (cmd) {
		case Cmd::recall_scene:     return libp::zcl::cmd::scenes_recall_scene;
		case Cmd::on:               return libp::zcl::cmd::onoff_on;
		case Cmd::off:              return libp::zcl::cmd::onoff_off;
		case Cmd::toggle:           return libp::zcl::cmd::onoff_toggle;
		case Cmd::mv_to_lvl:        return libp::zcl::cmd::level_move_to_lvl;
		case Cmd::mv_to_lvl_onoff:  return libp::zcl::cmd::level_move_to_lvl_onoff;
		case Cmd::up:               return libp::zcl::cmd::windowc_up_open;
		case Cmd::down:             return libp::zcl::cmd::windowc_down_close;
		case Cmd::stop:             return libp::zcl::cmd::windowc_stop;
		case Cmd::tuya_set_dp_u32:
		case Cmd::tuya_set_dp_enum:
		case Cmd::tuya_set_dp_bool:
		case Cmd::tuya_set_dp_raw:
		case Cmd::zcl_general:
		case Cmd::delay:
			break;
	}
	return 0;
}

/**
 * Insert 6-byte Tuya data point prefix into data array.
 *
 * Existing data is pushed 6-bytes forward.
 *
 * @return length including the prefix or 0 on error.
 */
template <uint16_t n>
static uint8_t addTuyaDataPointPrefix(tuya::DataPoint data_point, tuya::DataType data_type, uint8_t data[n], uint8_t data_len)
{
    /*
     * Tuya payload
     *
     * 0    : transaction seq # MSB
     * 1    : transaction seq # LSB
     * 2    : datapoint (dp)
     * 3    : datapoint type
     * 4    : datapoint length MSB
     * 5    : datapoint length LSB
     * 6..n : datapoint data
     */
    static_assert(n >= 6, "Buffer too small");
    if ( (data_len + 6) > n ) {
        return 0;
    }
	// Shift existing payload forward 6-bytes
	for (uint8_t i = data_len + 5; i > 5; i--) {
		data[i] = data[i-6];
	}
	static uint16_t trans_seq = 1;
	libp::writeBEndU16(&data[0], trans_seq++); // overflow is fine, 0 is valid.
	data[2] = libp::enumVal(data_point);
	data[3] = libp::enumVal(data_type);
	// Tuya is big-endian
    data[4] = (data_len >> 8) & 0xff;
    data[5] = data_len & 0xff;

    return data_len + 6;
}


ZclCommand& libp::zcl_script::internal::scriptCmdToZclCommand(ScriptCmd& cmd)
{
    ZclCommand& zcl = cmd.zcl;

	switch(cmd.cmd) {
	case Cmd::recall_scene:
		// Unicast recall scene without group should default to global scene (0)
		if (  zcl.group == ZclCommand::unset_group &&
			 (zcl.addr_type == AddrType::ieee64 || zcl.addr_type == AddrType::local) ) {
			zcl.group = 0;
		}
	    zcl.cluster_id = zcl::cluster::scenes;
	    zcl.cmd_id = zcl::cmd::scenes_recall_scene;
		// u16 group id | u8 scene id | u16 trans time
		// Zigbee is little-endian
        libp::writeLEndU16(zcl.payload, zcl.group);
		zcl.payload[2] = cmd.scene;
		zcl.payload[3] = 0;
		zcl.payload[4] = 0;
		zcl.payload_len = 5;
		cmd.cmd = Cmd::zcl_general;
		break;

	// Convert no payload ZCL commands to generic ZCL command
	case Cmd::on:
	case Cmd::off:
	case Cmd::toggle:
	    zcl.cluster_id = zcl::cluster::onoff;
	    zcl.cmd_id = scriptCmdToZclCmdId(cmd.cmd);
		cmd.cmd = Cmd::zcl_general;
		break;

    case Cmd::mv_to_lvl:
    case Cmd::mv_to_lvl_onoff:
	    zcl.cluster_id = zcl::cluster::level;
	    zcl.cmd_id = scriptCmdToZclCmdId(cmd.cmd);
	    cmd.cmd = Cmd::zcl_general;
    	zcl.payload[0] = cmd.value & 0xff;// u8 level
    	zcl.payload[1] = 0xff;            // u16 transition time (1/10s of s)
    	zcl.payload[2] = 0xff;            // - 0xffff = use OnOffTransitionTime
    	zcl.payload[3] = 0;               // map8 opt mask
    	zcl.payload[4] = 0;               // map8 opt override
		zcl.payload_len = 5;
        break;

	case Cmd::up:
	case Cmd::down:
	case Cmd::stop:
	    zcl.cluster_id = zcl::cluster::window_covering;
	    zcl.cmd_id = scriptCmdToZclCmdId(cmd.cmd);
		cmd.cmd = Cmd::zcl_general;
		break;

    case Cmd::tuya_set_dp_enum:
        cmd.tuya_type = tuya::DataType::enum_t;
        cmd.cmd = Cmd::tuya_set_dp_raw;
        zcl.payload[0] = cmd.value & 0xff;
        zcl.payload_len = 1;
        break;

    case Cmd::tuya_set_dp_bool:
        cmd.tuya_type = tuya::DataType::bool_t;
        cmd.cmd = Cmd::tuya_set_dp_raw;
        zcl.payload[0] = cmd.value & 0xff;
        zcl.payload_len = 1;
        break;

    case Cmd::tuya_set_dp_u32:
		cmd.tuya_type = tuya::DataType::value_t;
		cmd.cmd = Cmd::tuya_set_dp_raw;
        // Tuya payloads are big-endian
        libp::writeBEndU32(zcl.payload, cmd.value);
        zcl.payload_len = 4;
		break;

	case Cmd::tuya_set_dp_raw: {
	    zcl.cluster_id = tuya::cluster_id;
		zcl.cmd_id = libp::enumVal(tuya::ZclCmd::data_request);
		zcl.payload_len = addTuyaDataPointPrefix<ZclCommand::max_payload_len>(
		        cmd.tuya_dp,
		        cmd.tuya_type,
		        zcl.payload,
		        zcl.payload_len);
		LP_ASSERT(zcl.payload_len != 0); // 0 is error return for addTuyaDataPointPrefix
		cmd.cmd = Cmd::zcl_general;
		break;
	}

	case Cmd::zcl_general:
		// zcl data set by user
        break;

    case Cmd::delay:
        libp::writeLEndU16(zcl.payload, cmd.value);
        // Don't need payload len as this a non-nwk command with known payload
        // format
        cmd.zcl.cluster_id = ZclCommand::cluster_id_local_delay;
        break;
	}
	// single recursive step to setup Tuya cmd/payload
	if (cmd.cmd == Cmd::tuya_set_dp_raw) {
		scriptCmdToZclCommand(cmd);
	}
	return zcl;
}



// Not static as we need it for parseScriptQueueAllCmds, which is a template fn
// and thus in a header.
bool internal::parseScriptSingleCmd(uint8_t src_ep, const char *p_script, uint16_t len, uint16_t *p_idx, ScriptCmd* p_cmd)
{
    ParseResult result = parseScriptCmdFirstPass(p_script, len, p_idx, p_cmd);
    if (result != ParseResult::success) {
    	LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "script parse err@idx %u: %s",
                     	*p_idx, parseResultToDesc(result));
        return false;
    }
    ScriptCmd::Error cmd_result = p_cmd->validate();
    if (cmd_result != ScriptCmd::Error::valid) {
    	LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "script cmd err@idx %u: %s",
                        *p_idx, ScriptCmd::errorToDesc(cmd_result));
        return false;
    }
    p_cmd->zcl.src_ep = src_ep;
    scriptCmdToZclCommand(*p_cmd);
    return true;
}


bool libp::zcl_script::parseScriptSingleCmd(uint8_t src_ep, const char *p_script, uint16_t len, uint16_t *p_idx, ZclCommand* p_cmd)
{
    ScriptCmd script_cmd{};
    bool success = internal::parseScriptSingleCmd(src_ep, p_script, len, p_idx, &script_cmd);
    if (!success) {
        return false;
    }
    // Copy to receiver
    *p_cmd = script_cmd.zcl;
    return true;
}
