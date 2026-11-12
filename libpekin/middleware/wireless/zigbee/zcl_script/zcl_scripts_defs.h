/*
 * zcl_scripts_defs.h
 *
 *  Created on: 23 Oct 2026
 *      Author: bob
 */

#ifndef LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_DEFS_H_
#define LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_DEFS_H_

#include "middleware//wireless/zigbee/zcl_script/zcl_scripts_util.h"


namespace libp::zcl_script {

/**
 * Function to report script validation/execution error descriptions.
 */
using err_report_callback_t  = void (*)(const char*, ...);


constexpr char delim_token = ',';
constexpr char delim_script = ';';
constexpr char addr_sym_broadcast = '*';
constexpr char addr_sym_bound = '#';

constexpr uint8_t tuya_payload_prefix_len = 6;

enum class Cmd : uint16_t {
	recall_scene     = internal::dblCharToU16("rs"),

	on               = internal::dblCharToU16("on"),
	off              = internal::dblCharToU16("of"),
	toggle           = internal::dblCharToU16("tg"),

    mv_to_lvl        = internal::dblCharToU16("ml"),
    mv_to_lvl_onoff  = internal::dblCharToU16("mo"),

	up               = internal::dblCharToU16("up"),
	down             = internal::dblCharToU16("dn"),
	stop             = internal::dblCharToU16("st"),

	tuya_set_dp_u32  = internal::dblCharToU16("dv"),
	tuya_set_dp_enum = internal::dblCharToU16("de"),
	tuya_set_dp_bool = internal::dblCharToU16("db"),

	tuya_set_dp_raw  = internal::dblCharToU16("dp"),

	zcl_general      = internal::dblCharToU16("zl"),

	delay = internal::dblCharToU16("dl"),
};

enum class CmdParam : uint8_t {
	dst_addr    = 'a',
	group       = 'g',
	scene       = 's',
	dst_ep      = 'e',
	cluster_id  = 'l',
	cmd_id      = 'c',
	mfr_id      = 'm',
	tuya_dp     = 'd',
	tuya_dt     = 't',
	value       = 'v',
	payload     = 'p',
};

enum class AddrType : uint8_t {
    not_set, local, ieee64, group, bound, broadcast
};



} // namespace libp::zcl_script


#endif /* LIBPEKIN_NEW_WIRELESS_ZIGBEE_ZCL_SCRIPT_ZCL_SCRIPTS_DEFS_H_ */
