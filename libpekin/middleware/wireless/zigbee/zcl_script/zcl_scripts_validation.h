/*
 * zcl_scripts_validation.h
 *
 *  Created on: 23 Oct 2026
 *      Author: bob
 */

#ifndef LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_VALIDATION_H_
#define LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_VALIDATION_H_

#include <cstdint>

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_defs.h"

namespace libp::zcl_script {

/**
 * Validate a multi-command script (both parse and command validation).
 *
 * @param p_script
 * @param len
 * @param err_printf function to receive description on error. May be null
 *
 * @return false on error. Error will be output to err_printf if provided.
 */
bool validateScript(const char *p_script, uint16_t len, err_report_callback_t err_printf);

} // namespace libp::zcl_script

#endif /* LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_VALIDATION_H_ */
