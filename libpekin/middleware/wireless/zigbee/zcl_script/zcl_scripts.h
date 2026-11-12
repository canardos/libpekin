/**
 * @file
 * @brief ZCL command scripting functions
 *
 * These functions allow for serial ZCL network commands (ZclCommand) to be
 * stored in a simple text string.
 *
 * The typical application is storing a sequence of user-supplied network
 * commands that are sent on an external event such as a button press.
 *
 * e.g: "on,g1;tg,a0xabcd,e2";
 * - Turn on group 1 (onOff cluster)
 * - Toggle endpoint 2 of device with local address 0xabcd (onOff cluster)
 *
 * The recommended usage is as follows:
 *
 * @code{.cpp}
 * uint32_t appSendZclCommand(ZclCommand& cmd);
 * void appPrintErr(const char* fmt, ...);
 *
 * // Holds execution context for running scripts
 * ScriptExecMgr<
 *     4,                // number of scripts (e.g. 2-buttons, 1/2-tap)
 *     appPrintErr,      // app fn to receive descriptions on error
 *     appSendZclCommand // app fn to send ZclCommand
 *     25,               // OPTIONAL: min delay between calls to appSendZclCommand
 *     0                 // OPTIONAL: return value of appSendZclCommand that
 * > exec_mgr;           //           indicates success
 *
 * // Start a script
 * uint8_t src_ep = 1;
 * const char* script = "on,g1;of,a1234,e1;";
 * exec_mgr.startScript(
 *     0,                // script idx
 *     src_ep,           // source EP
 *     script,           // script
 *     strlen(script));  // script len
 *
 * // And in the main app loop
 * {
 *     // ...
 *     exec_mgr.tick()
 *     // ...
 * }
 *
 * // The startScript call doesn't copy the script, and so if the script is to
 * // be modified, then call stopExec first to avoid errors.
 * exec_mgr.stopScript(0);
 *
 * @endcode
 *
 * The application should use the \ref validateScript function to validate the
 * script text prior to parsing/executing.
 *
 * A lower-level API is also available for direct parsing. Two functions
 * provided. Use depends on memory constraints:
 *
 * 1) Use \ref parseScriptQueueAllCmds to parse full script in one call and
 *    place all commands in a queue (if memory allows).
 * 2) Use \ref parseScriptSingleCmd to parse and process script command by
 *    command. This is used by the \ref ScriptExecMgr.
 *
 * See README.md in this folder for the script/command format.
 *
 * @par Example low-level usage: One-shot queue
 * @code{.cpp}
 *
 * // This is an example to highlight the usage of the API functions. A proper
 * // app can't busy spin and will typically use ScriptExecCtx objuects to
 * // manage individual script execution.
 *
 * using namespace libp::zcl_script;
 *
 * uint32_t appSendZclCommand(ZclCommand& cmd);
 * void printErr(const char* fmt, ...);
 *
 * libp::SimpleQueue<ZclCommand, 16> cmd_q{};
 * const char* script = "on,a1234,e1;tg,a4567,e2;dl,v500;up,a0x12ab,e1";
 * uint8_t len = strlen(script);
 * uint8_t src_ep = 1;
 *
 * if (validateScript(script, len, printErr)) {
 *     // Parse the script into individual ZCL commands
 *     ZclCommand cmd;
 *     parseScriptQueueAllCmds(src_ep, script, len, cmd_q);
 *     // Send the commands (typically done in the main app loop)
 *     while (cmd_q.pop(&cmd)) {
 *         // Commands are either a local delay request; OR
 *         if (cmd.isLocalDelay()) {
 *             delayMs(cmd.getLocalDelayMs());
 *         }
 *         // a ZCL command to be sent
 *         else {
 *             appSendZclCommand(cmd);
 *             delayMs(min_delay_btn_cmd_ms);
 *         }
 *     }
 * }
 * @endcode
 *
 * @par Example low-level usage: Online command by command processing
 * @code{.cpp}
 *
 * // This is an example to highlight the usage of the API functions. A proper
 * // app can't busy spin and will typically use ScriptExecCtx objuects to
 * // manage individual script execution.
 *
 * using namespace libp::zcl_script;
 *
 * void appSendZclCommand(ZclCommand& cmd);
 * void printErr(const char* fmt, ...);
 *
 * libp::SimpleQueue<ZclCommand, 16> cmd_q{};
 * const char* script = "on,a1234,e1;tg,a4567,e2;dl,v500;up,a0x12ab,e1";
 * uint8_t len = strlen(script);
 * uint8_t src_ep = 1;
 *
 * if (validateScript(script, len, printErr)) {
 *     uint16_t idx = 0;
 *     while (idx < len) {
 *         ZclCommand cmd{};
 *         if (!parseScriptSingleCmd(src_ep, script, len, &idx, &cmd)) {
 *             return false;
 *         }
 *         // Commands are either a local delay request; OR
 *         if (cmd.isLocalDelay()) {
 *             delayMs(cmd.getLocalDelayMs());
 *         }
 *         // a ZCL command to be sent
 *         else {
 *             appSendZclCommand(cmd);
 *             delayMs(min_delay_btn_cmd_ms);
 *         }
 *     }
 * }
 * @endcode
 */

#ifndef LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_H_
#define LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_H_

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_defs.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_util.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_cmds.h"

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_parse.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_validation.h"
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_exec_ctx.h"

#endif // LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_H_
