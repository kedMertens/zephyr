/* gatt_cl.h - Bluetooth tester GATT Client service definitions */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TESTS_BLUETOOTH_TESTER_SRC_BTP_GATT_CL_
#define TESTS_BLUETOOTH_TESTER_SRC_BTP_GATT_CL_

#include <stdint.h>

#include <zephyr/bluetooth/addr.h>

/* GATT Client Service (0x06) */
#define BTP_GATT_CL_READ_SUPPORTED_COMMANDS	0x01
struct btp_gatt_cl_read_supported_commands_rp {
	uint8_t data[0];
} __packed;

#define BTP_GATT_CL_EXCHANGE_MTU		0x02
struct btp_gatt_cl_exchange_mtu_cmd {
	bt_addr_le_t address;
} __packed;

#define BTP_GATT_CL_DISC_ALL_PRIM		0x03
struct btp_gatt_cl_disc_all_prim_cmd {
	bt_addr_le_t address;
} __packed;

#define BTP_GATT_CL_DISC_PRIM_UUID		0x04
struct btp_gatt_cl_disc_prim_uuid_cmd {
	bt_addr_le_t address;
	uint8_t uuid_length;
	uint8_t uuid[];
} __packed;

#define BTP_GATT_CL_FIND_INCLUDED		0x05
struct btp_gatt_cl_find_included_cmd {
	bt_addr_le_t address;
	uint16_t start_handle;
	uint16_t end_handle;
} __packed;

#define BTP_GATT_CL_DISC_ALL_CHRC		0x06
struct btp_gatt_cl_disc_all_chrc_cmd {
	bt_addr_le_t address;
	uint16_t start_handle;
	uint16_t end_handle;
} __packed;

#define BTP_GATT_CL_DISC_CHRC_UUID		0x07
struct btp_gatt_cl_disc_chrc_uuid_cmd {
	bt_addr_le_t address;
	uint16_t start_handle;
	uint16_t end_handle;
	uint8_t uuid_length;
	uint8_t uuid[];
} __packed;

#define BTP_GATT_CL_DISC_ALL_DESC		0x08
struct btp_gatt_cl_disc_all_desc_cmd {
	bt_addr_le_t address;
	uint16_t start_handle;
	uint16_t end_handle;
} __packed;

#define BTP_GATT_CL_WRITE_WITHOUT_RSP		0x0d
struct btp_gatt_cl_write_without_rsp_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_SIGNED_WRITE_WITHOUT_RSP	0x0e
struct btp_gatt_cl_signed_write_without_rsp_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_WRITE			0x0f
struct btp_gatt_cl_write_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_WRITE_LONG			0x10
struct btp_gatt_cl_write_long_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t offset;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_WRITE_RELIABLE		0x11
struct btp_gatt_cl_write_reliable_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t offset;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_CFG_NOTIFY			0x12
#define BTP_GATT_CL_CFG_INDICATE		0x13
struct btp_gatt_cl_cfg_notify_cmd {
	bt_addr_le_t address;
	uint8_t enable;
	uint16_t ccc_handle;
} __packed;

#define BTP_GATT_CL_READ_MULTIPLE_VAR		0x14
struct btp_gatt_cl_read_multiple_var_cmd {
	bt_addr_le_t address;
	uint8_t handles_count;
	uint16_t handles[];
} __packed;

#define BTP_GATT_CL_EATT_CONNECT		0x1f
struct btp_gatt_cl_eatt_connect_cmd {
	bt_addr_le_t address;
	uint8_t num_channels;
} __packed;

#define BTP_GATT_CL_READ			0x09
struct btp_gatt_cl_read_cmd {
	bt_addr_le_t address;
	uint16_t handle;
} __packed;

#define BTP_GATT_CL_READ_UUID			0x0a
struct btp_gatt_cl_read_uuid_cmd {
	bt_addr_le_t address;
	uint16_t start_handle;
	uint16_t end_handle;
	uint8_t uuid_length;
	uint8_t uuid[];
} __packed;

#define BTP_GATT_CL_READ_MULTIPLE		0x0c
struct btp_gatt_cl_read_multiple_cmd {
	bt_addr_le_t address;
	uint8_t handles_count;
	uint16_t handles[];
} __packed;

#define BTP_GATT_CL_READ_LONG			0x0b
struct btp_gatt_cl_read_long_cmd {
	bt_addr_le_t address;
	uint16_t handle;
	uint16_t offset;
} __packed;

/* Events */
#define BTP_GATT_CL_EV_MTU_EXCHANGED		0x80
struct btp_gatt_cl_mtu_exchanged_ev {
	bt_addr_le_t address;
	uint8_t status;
} __packed;

#define BTP_GATT_CL_EV_DISC_ALL_PRIM_RP		0x81
struct btp_gatt_cl_disc_all_prim_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t services_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_DISC_PRIM_UUID_RP	0x82
struct btp_gatt_cl_disc_prim_uuid_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t services_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_FIND_INCLUDED_RP		0x83
struct btp_gatt_cl_find_included_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t services_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_DISC_ALL_CHRC_RP		0x84
struct btp_gatt_cl_disc_all_chrc_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t characteristics_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_DISC_CHRC_UUID_RP	0x85
struct btp_gatt_cl_disc_chrc_uuid_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t characteristics_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_DISC_ALL_DESC_RP		0x86
struct btp_gatt_cl_disc_all_desc_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint8_t descriptors_count;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_WRITE_RP			0x8b
struct btp_gatt_cl_write_ev {
	bt_addr_le_t address;
	uint8_t att_response;
} __packed;

#define BTP_GATT_CL_EV_WRITE_LONG_RP		0x8c
struct btp_gatt_cl_write_long_ev {
	bt_addr_le_t address;
	uint8_t att_response;
} __packed;

#define BTP_GATT_CL_EV_RELIABLE_WRITE_RP	0x8d
struct btp_gatt_cl_reliable_write_ev {
	bt_addr_le_t address;
	uint8_t att_response;
} __packed;

#define BTP_GATT_CL_EV_CFG_NOTIFY_RP		0x8e
struct btp_gatt_cl_cfg_notify_ev {
	bt_addr_le_t address;
	uint8_t att_response;
} __packed;

#define BTP_GATT_CL_EV_CFG_INDICATE_RP		0x8f
struct btp_gatt_cl_cfg_indicate_ev {
	bt_addr_le_t address;
	uint8_t att_response;
} __packed;

#define BTP_GATT_CL_EV_NOTIFICATION_RXED	0x90
struct btp_gatt_cl_notification_ev {
	bt_addr_le_t address;
	uint8_t type;
	uint16_t handle;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_READ_MULTIPLE_VAR	0x91
struct btp_gatt_cl_read_multiple_var_ev {
	bt_addr_le_t address;
	uint8_t status;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_READ_RP			0x87
struct btp_gatt_cl_read_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_READ_UUID_RP		0x88
struct btp_gatt_cl_read_uuid_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t value_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_READ_LONG_RP		0x89
struct btp_gatt_cl_read_long_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#define BTP_GATT_CL_EV_READ_MULTIPLE_RP		0x8a
struct btp_gatt_cl_read_multiple_ev {
	bt_addr_le_t address;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t data[];
} __packed;

#endif /* TESTS_BLUETOOTH_TESTER_SRC_BTP_GATT_CL_ */
