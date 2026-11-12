/*
 * zcl_scripts_execution_context.h
 *
 *  Created on: Oct 25, 2026
 *      Author: bob
 */

#ifndef LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_EXEC_CTX_H_
#define LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_EXEC_CTX_H_

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_defs.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_parse.h"
#include "lp_logging.h"
#include <cstdint>

namespace libp::zcl_script {

namespace internal {
 /**
  * Script execution context.
  *
  * Encapsulates context for an executing command script.
 */
struct ScriptExecCtx {

	uint8_t src_ep;             ///<
	const char *script;         ///< script
	uint16_t len;               ///< script total length
	uint16_t offset;            ///< current
	uint32_t last_exec_time_ms; ///<
	uint32_t current_delay_ms;  ///< ms after last_exec_time_ms to start executing
	bool is_active;             ///< currently executing

	void init(uint8_t new_src_ep, const char *new_script, uint16_t new_len) {
		src_ep = new_src_ep;
		script = new_script;
		len = new_len;
		reset();
	}
	/**
	 * Reset script execution and set inactive.
	 */
	void reset()
	{
		offset = 0;
		last_exec_time_ms = 0;
		current_delay_ms = 0;
		is_active = false;
	}
	bool readyToSend(uint32_t now_ms) const
	{
		return is_active && (now_ms - last_exec_time_ms >= current_delay_ms) && !isAtEnd();
	}
	bool isAtEnd() const
	{
		return offset >= len;
	}
};
} // namespace internal

static constexpr uint16_t default_delay_btn_cmd_ms = 30;

using send_zcl_cmd_callback_t = uint32_t (*)(const ZclCommand&);

/**
 *
 */
template <
	uint8_t NUM_SCRIPTS,
	err_report_callback_t report_err,
	send_zcl_cmd_callback_t send_zcl_cmd,
	uint32_t STATUS_SUCCESS=0
>
class ScriptExecMgr {
public:

	/**
	 * Start execution of a script.
	 *
	 * After this call, \ref tick() must be called regularly to ensure all
	 * commands are processed.
	 *
	 * @note A reference to \ref script is retained and used until execution is
	 * complete. Do not free or modify the script without waiting for script
	 * completion or calling \ref stopExec first.
	 *
	 * @param script_idx
	 * @param src_ep
	 * @param script
	 * @param lin
	 */
	void startScript(uint8_t script_idx, uint8_t src_ep, const char* script, uint16_t len)
	{
		if (script_idx >= NUM_SCRIPTS) {
			return;
		}
		auto& ctx = exec_ctx_[script_idx];
	    ctx.init(src_ep, script, len);
		ctx.is_active = true;
        LP_LOG_DEBUG_IF(EN_LOG_ZCL_SCRIPTS, "script[%u] start, len=%u", script_idx, len);
	}

	/**
	 * Stop script execution.
	 */
	void stopScript(uint8_t script_idx)
	{
		if (script_idx >= NUM_SCRIPTS) {
			return;
		}
        LP_LOG_DEBUG_IF(EN_LOG_ZCL_SCRIPTS, "script[%u] stop", script_idx);
		auto& ctx = exec_ctx_[script_idx];
		ctx.reset();
	}

	/**
	 * A maximum of one command per executing script will be sent.
	 *
	 * @return true if a command was sent
	 */
	bool tick()
	{
		ZclCommand cmd{};
		bool cmd_sent = false;
	    uint32_t now_ms = libp::getMillis();

		for (uint8_t i = 0; i < NUM_SCRIPTS; i++) {
			auto& ctx = exec_ctx_[i];
			if (ctx.readyToSend(now_ms)) {
				bool success = parseScriptSingleCmd(ctx.src_ep, ctx.script, ctx.len, &ctx.offset, &cmd);
				if (success) {
					ctx.last_exec_time_ms = now_ms;
					if (cmd.isLocalDelay()) {
						ctx.current_delay_ms = cmd.getLocalDelayMs();
					}
					else {
						uint32_t status = send_zcl_cmd(cmd);
						if (status != STATUS_SUCCESS) {
                            // Report send fail, but proceed to next command.
							report_err("script[%u]: zcl cmd (cl/cmd = 0x%04x/0x%02x) failed: 0x%02x",
									i, cmd.cluster_id, cmd.cmd_id, status);
	                        LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "script[%u]: zcl cmd (cl/cmd = 0x%04x/0x%02x) failed: 0x%02x",
	                                i, cmd.cluster_id, cmd.cmd_id, status);
						}
						else {
							cmd_sent = true;
						}
						ctx.current_delay_ms = delay_btn_cmd_ms_;
					}
	                if (ctx.isAtEnd()) {
				        LP_LOG_DEBUG_IF(EN_LOG_ZCL_SCRIPTS, "script[%u]: finished", i);
	                    ctx.reset();
	                }
				}
				else {
                    // Stop on parse fail
					LP_LOG_ERROR_IF(EN_LOG_ZCL_SCRIPTS, "script[%u]: parseScriptSingleCmd fail for ep %u", i, ctx.src_ep);
					ctx.reset();
				}
			}
		}
		return cmd_sent;	}

	void setDelayBtnCmd(uint16_t delay_btn_cmd_ms)
	{
	    delay_btn_cmd_ms_ = delay_btn_cmd_ms;
	}

	bool isActive(uint8_t script_idx)
	{
        if (script_idx >= NUM_SCRIPTS) {
            return false;
        }
        auto& ctx = exec_ctx_[script_idx];
        return ctx.is_active;
	}

private:
	internal::ScriptExecCtx exec_ctx_[NUM_SCRIPTS]{};
	uint16_t delay_btn_cmd_ms_ = default_delay_btn_cmd_ms;
};

} // namespace libp::zcl_script

#endif /* LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_EXEC_CTX_H_ */
