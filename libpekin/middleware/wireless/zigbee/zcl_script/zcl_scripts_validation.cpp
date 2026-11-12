
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_validation.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_cmds.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_parse.h"
#include "lp_logging.h"

#include <cstdint>

bool libp::zcl_script::validateScript(const char *p_script, uint16_t len, err_report_callback_t err_printf)
{
	uint16_t idx = 0;
	uint8_t n_cmd = 0;
	while (idx < len) {
		ScriptCmd cmd{};
		internal::ParseResult result = internal::parseScriptCmdFirstPass(p_script, len, &idx, &cmd);
		if (result != internal::ParseResult::success) {
            if(err_printf != nullptr) {
                err_printf("Script parse err @ idx %u: %s\r\n",
                           idx,
						   internal::parseResultToDesc(result));
            }
            LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "Script parse err @ idx %u: %s",
                                         idx,
										 internal::parseResultToDesc(result));
			return false;
		}
		ScriptCmd::Error cmd_status = cmd.validate();
		if (cmd_status != ScriptCmd::Error::valid) {
            if(err_printf != nullptr) {
                err_printf("Invalid cmd %u (%c%c) @ idx %u: %s\r\n",
                           n_cmd,
                           static_cast<uint16_t>(cmd.cmd) >> 8,
                           static_cast<uint16_t>(cmd.cmd) & 0xff,
                           idx,
                           ScriptCmd::errorToDesc(cmd_status) );
            }
            LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "Invalid cmd %u (%c%c) @ idx %u: %s",
                                         n_cmd,
                                         static_cast<uint16_t>(cmd.cmd) >> 8,
                                         static_cast<uint16_t>(cmd.cmd) & 0xff,
                                         idx,
                                         ScriptCmd::errorToDesc(cmd_status) );
			return false;
		}
		n_cmd++;
	}
	return true;
}
