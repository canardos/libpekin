#ifndef LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_QUEUE_H_
#define LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_QUEUE_H_

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_cmds.h"
#include "lp_logging.h"

#include <concepts>
#include <type_traits>
#include <cstdint>

namespace libp::zcl_script {

namespace internal {

	/**
	 *
	 */
	enum class ParseResult : uint8_t {
		success = 0,
		invalid_command,    // 1
		missing_delim,      // 2
		invalid_param,      // 3
		invalid_addr,       // 4
		invalid_scene,      // 5
		invalid_dst_ep,     // 6
		invalid_group,      // 7
		invalid_cluster_id, // 8
		invalid_cmd_id,     // 9
		invalid_mfr_id,     // 10
		invalid_tuya_dp,    // 11
		invalid_tuya_type,  // 12
		invalid_value,      // 13
		invalid_payload,    // 14
		payload_too_long,   // 15
		LAST
	};
	inline constexpr const char* parse_result_desc[] = {
		"",
		"Inv cmd",       // 1
		"Mis ','",       // 2
		"Inv param",     // 3
		"Inv addr",      // 4
		"Inv scn",       // 5
		"Inv EP",        // 6
		"Inv grp",       // 7
		"Inv clustID",   // 8
		"Inv cmdID",     // 9
		"Inv mfrID",     // 10
		"Inv TuyaDP",    // 11
		"Inv TuyaTyp",   // 12
		"Inv val",       // 13
		"Inv pload",     // 14
		"Pload len",     // 15
	};
	static_assert( (sizeof(parse_result_desc) / sizeof(parse_result_desc[0])) == libp::enumVal(ParseResult::LAST) );
	static inline constexpr const char* parseResultToDesc(ParseResult result) {
		uint8_t idx = libp::enumVal(result);
		return idx < (sizeof(parse_result_desc) / sizeof(parse_result_desc[0]))
				? parse_result_desc[idx]
				: "unknown";
	}

	/**
	 *
	 */
	bool parseScriptSingleCmd(uint8_t src_ep, const char *p_script, uint16_t len, uint16_t *p_idx, ScriptCmd* p_cmd);


	/**
	 * Parse a single text command into a ScriptCmd object. Validates parameter
	 * values, but does not check command/param combinations.
	 *
	 * Use ScriptCmd.validate on returned p_cmd for that.
	 *
	 * The returned ScriptCmd does not necessarily contain a valid ZclCommand (.zcl)
	 * and must first be processed with \ref scriptCmdToZclCommand.
	 *
	 * Don't forget to set p_cmd->src_ep.
	 *
	 * @note This function is not intended for direct application usage. Use
	 *       parseScriptSingleCmd / parseScriptQueueAllCmds.
	 *
	 * @param p_script  [in] script to parse
	 * @param len       [in] length of p_script
	 * @param p_idx [in/out] where to begin parsing - will be updated to end of
	 *                       parsed command + 1 (e.g. for "on,g1" or "on,g1;tg,g2"
	 *                       p_idx==6 after parse)
	 * @param p_cmd    [out] parsed command
	 *
	 * @return ParseResult::success or other value on error
	 */
	ParseResult parseScriptCmdFirstPass(const char *p_script, uint16_t len, uint16_t *p_idx, ScriptCmd* p_cmd);


	/**
	 * Convert a parsed script cmd into a standard zcl_general command with payload
	 * populated.
	 *
	 * After conversion, the ZCL command is in cmd.zcl.
	 *
	 * Exception for delay command TODO XXX
	 *
	 * @note This function is not intended for direct application usage. Use
	 *       \ref parseScriptSingleCmd / \ref parseScriptQueueAllCmds.
	 *
	 * @param ScriptCmd [in,out]
	 *
	 * @return ScriptCmd.zcl ref
	 */
	ZclCommand& scriptCmdToZclCommand(ScriptCmd& cmd);

} // namespace internal


/**
 * Parse a single text command into a ZclCommand object.
 *
 * The returned ScriptCmd does not necessarily contain a valid ZclCommand (.zcl)
 * and must first be processed with scriptCmdToZclCommand.
 *
 * @note Use \ref validateScript prior to calling this function to ensure the
 *       script is valid and get detailed error information if it isn't.
 *
 * @param src_ep    [in] source endpoint
 * @param p_script  [in] script to parse
 * @param len       [in] length of p_script
 * @param p_idx [in/out] where to begin parsing - will be updated to end of
 *                       parsed command (after delimiter - if any)
 * @param p_cmd    [out] parsed command
 *
 * @return true on success
 */
bool parseScriptSingleCmd(uint8_t src_ep, const char *p_script, uint16_t len, uint16_t *p_idx, ZclCommand* p_cmd);


/**
 * Concept for parseScriptQueueAllCmds
 */
template <typename T>
concept ScriptCmdQueueWithPush = requires(T t, const ZclCommand& cmd) {
    /// returns false if queue is full
    { t.push(cmd) } -> std::same_as<bool>;
};


/**
 * Parse a script into commands and place them in the specified queue.
 *
 * @param src_ep source endpoint
 * @param p_script script text
 * @param len length of script
 * @param queue
 *
 * @return false on parse/cmd error or if queue.push returns false.
 */
inline bool parseScriptQueueAllCmds(
        uint8_t src_ep, const char* p_script, uint16_t len,
        ScriptCmdQueueWithPush auto& queue)
{
    uint16_t n_cmd = 0;
    uint16_t idx = 0;

    while (idx < len) {
        ScriptCmd script_cmd{};
        bool success = internal::parseScriptSingleCmd(src_ep, p_script, len, &idx, &script_cmd);
        if (!success) {
            return false;
        }
        if (!queue.push(script_cmd.zcl)) {
            LP_LOG_ERROR("cmd q full");
            return false;
        }
        n_cmd++;
    }
    LP_LOG_DEBUG_IF(EN_LOG_ZCL_SCRIPTS, "Queued %u script cmds from ep %u", n_cmd, src_ep);
    return true;
}


} // namespace libp::zcl_script

#endif // LIB_LIBPEKIN_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_QUEUE_H_
