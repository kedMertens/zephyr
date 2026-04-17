/* gatt_cl.c - Bluetooth GATT Client Tester service */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "btp/btp.h"

#define LOG_MODULE_NAME bttester_gatt_cl
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_BTTESTER_LOG_LEVEL);

#define GATT_CL_MAX_OPS 4
#define GATT_CL_MAX_HANDLES 5
#define GATT_CL_MAX_VALUE_LEN 2048
#define GATT_CL_EATT_CONNECT_RETRIES 30
#define GATT_CL_EATT_CONNECT_RETRY_DELAY_MS 100
#define MAX_SUBSCRIPTIONS 2
#define UNUSED_SUBSCRIBE_CCC_HANDLE 0x0000
#define MAX_NOTIF_DATA (MIN(BT_L2CAP_RX_MTU, BT_L2CAP_TX_MTU) - 3)

union uuid {
	struct bt_uuid uuid;
	struct bt_uuid_16 u16;
	struct bt_uuid_128 u128;
};

struct gatt_cl_read_multiple_var_op {
	struct bt_gatt_read_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t handles_count;
	uint16_t handles[GATT_CL_MAX_HANDLES];
	uint8_t status;
	uint8_t att_response;
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

struct gatt_cl_exchange_mtu_op {
	struct bt_gatt_exchange_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	bool in_use;
};

struct gatt_cl_discovery_op {
	struct bt_gatt_discover_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	union uuid uuid;
	uint8_t status;
	uint8_t event_opcode;
	uint8_t count;
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

struct gatt_cl_write_op {
	struct bt_gatt_write_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t event_opcode;
	uint8_t status;
	bool in_use;
};

struct gatt_cl_cfg_subscribe_op {
	struct bt_gatt_discover_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t event_opcode;
	uint16_t ccc_handle;
	uint16_t value;
	bool in_use;
};

static struct gatt_cl_read_multiple_var_op ops[GATT_CL_MAX_OPS];
static struct gatt_cl_exchange_mtu_op exchange_mtu_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_discovery_op discovery_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_write_op write_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_cfg_subscribe_op cfg_ops[GATT_CL_MAX_OPS];
static struct bt_gatt_subscribe_params subscriptions[MAX_SUBSCRIPTIONS];
static uint8_t notif_ev_buf[sizeof(struct btp_gatt_cl_notification_ev) + MAX_NOTIF_DATA];
static K_MUTEX_DEFINE(ops_lock);

struct gatt_cl_read_long_op {
	struct bt_gatt_read_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t status;
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

struct gatt_cl_read_op {
	struct bt_gatt_read_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t status;
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

struct gatt_cl_read_multiple_op {
	struct bt_gatt_read_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	uint8_t status;
	uint8_t handles_count;
	uint16_t handles[GATT_CL_MAX_HANDLES];
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

struct gatt_cl_read_uuid_op {
	struct bt_gatt_read_params params;
	struct bt_conn *conn;
	bt_addr_le_t address;
	union uuid uuid;
	uint8_t status;
	uint8_t value_length;
	uint16_t data_length;
	uint8_t data[GATT_CL_MAX_VALUE_LEN];
	bool in_use;
};

static struct gatt_cl_read_long_op read_long_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_read_op read_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_read_multiple_op read_multiple_ops[GATT_CL_MAX_OPS];
static struct gatt_cl_read_uuid_op read_uuid_ops[GATT_CL_MAX_OPS];
#if defined(CONFIG_BT_EATT)
static atomic_t read_multiple_var_seq;
#endif

/* Convert UUID from BTP command to bt_uuid */
static uint8_t btp2bt_uuid(const uint8_t *uuid, uint8_t len, struct bt_uuid *bt_uuid)
{
	uint16_t le16;

	switch (len) {
	case 0x02: /* UUID 16 */
		bt_uuid->type = BT_UUID_TYPE_16;
		memcpy(&le16, uuid, sizeof(le16));
		BT_UUID_16(bt_uuid)->val = sys_le16_to_cpu(le16);
		break;
	case 0x10: /* UUID 128 */
		bt_uuid->type = BT_UUID_TYPE_128;
		memcpy(BT_UUID_128(bt_uuid)->val, uuid, 16);
		break;
	default:
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static struct gatt_cl_read_multiple_var_op *alloc_op(void)
{
	struct gatt_cl_read_multiple_var_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(ops); i++) {
		if (!ops[i].in_use) {
			op = &ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static void free_op(struct gatt_cl_read_multiple_var_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void send_read_multiple_var_ev(struct gatt_cl_read_multiple_var_op *op)
{
	struct btp_gatt_cl_read_multiple_var_ev *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC event payload");
		return;
	}

	ev = (struct btp_gatt_cl_read_multiple_var_ev *)payload;
	ev->address = op->address;
	ev->status = op->status;
	ev->att_response = op->att_response;
	ev->data_length = sys_cpu_to_le16(op->data_length);
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_READ_MULTIPLE_VAR,
		     payload, payload_size);

	k_free(payload);
}

static struct gatt_cl_read_long_op *alloc_read_long_op(void)
{
	struct gatt_cl_read_long_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(read_long_ops); i++) {
		if (!read_long_ops[i].in_use) {
			op = &read_long_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_read_op *alloc_read_op(void)
{
	struct gatt_cl_read_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(read_ops); i++) {
		if (!read_ops[i].in_use) {
			op = &read_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_read_multiple_op *alloc_read_multiple_op(void)
{
	struct gatt_cl_read_multiple_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(read_multiple_ops); i++) {
		if (!read_multiple_ops[i].in_use) {
			op = &read_multiple_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_read_uuid_op *alloc_read_uuid_op(void)
{
	struct gatt_cl_read_uuid_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(read_uuid_ops); i++) {
		if (!read_uuid_ops[i].in_use) {
			op = &read_uuid_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_exchange_mtu_op *alloc_exchange_mtu_op(void)
{
	struct gatt_cl_exchange_mtu_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(exchange_mtu_ops); i++) {
		if (!exchange_mtu_ops[i].in_use) {
			op = &exchange_mtu_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_discovery_op *alloc_discovery_op(void)
{
	struct gatt_cl_discovery_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(discovery_ops); i++) {
		if (!discovery_ops[i].in_use) {
			op = &discovery_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_write_op *alloc_write_op(void)
{
	struct gatt_cl_write_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(write_ops); i++) {
		if (!write_ops[i].in_use) {
			op = &write_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static struct gatt_cl_cfg_subscribe_op *alloc_cfg_op(void)
{
	struct gatt_cl_cfg_subscribe_op *op = NULL;

	k_mutex_lock(&ops_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(cfg_ops); i++) {
		if (!cfg_ops[i].in_use) {
			op = &cfg_ops[i];
			op->in_use = true;
			break;
		}
	}

	k_mutex_unlock(&ops_lock);

	return op;
}

static void free_read_long_op(struct gatt_cl_read_long_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_read_op(struct gatt_cl_read_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_read_multiple_op(struct gatt_cl_read_multiple_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_read_uuid_op(struct gatt_cl_read_uuid_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_exchange_mtu_op(struct gatt_cl_exchange_mtu_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_discovery_op(struct gatt_cl_discovery_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_write_op(struct gatt_cl_write_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void free_cfg_op(struct gatt_cl_cfg_subscribe_op *op)
{
	if (op->conn != NULL) {
		bt_conn_unref(op->conn);
	}

	k_mutex_lock(&ops_lock, K_FOREVER);
	(void)memset(op, 0, sizeof(*op));
	k_mutex_unlock(&ops_lock);
}

static void send_read_long_ev(struct gatt_cl_read_long_op *op)
{
	struct btp_gatt_cl_read_long_ev *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC read_long event payload");
		return;
	}

	ev = (struct btp_gatt_cl_read_long_ev *)payload;
	ev->address = op->address;
	ev->att_response = op->status;
	ev->data_length = sys_cpu_to_le16(op->data_length);
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_READ_LONG_RP,
		     payload, payload_size);

	k_free(payload);
}

static void send_read_ev(struct gatt_cl_read_op *op)
{
	struct btp_gatt_cl_read_ev *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC read event payload");
		return;
	}

	ev = (struct btp_gatt_cl_read_ev *)payload;
	ev->address = op->address;
	ev->att_response = op->status;
	ev->data_length = sys_cpu_to_le16(op->data_length);
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_READ_RP,
		     payload, payload_size);

	k_free(payload);
}

static void send_read_multiple_ev(struct gatt_cl_read_multiple_op *op)
{
	struct btp_gatt_cl_read_multiple_ev *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC read_multiple event payload");
		return;
	}

	ev = (struct btp_gatt_cl_read_multiple_ev *)payload;
	ev->address = op->address;
	ev->att_response = op->status;
	ev->data_length = sys_cpu_to_le16(op->data_length);
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_READ_MULTIPLE_RP,
		     payload, payload_size);

	k_free(payload);
}

static void send_read_uuid_ev(struct gatt_cl_read_uuid_op *op)
{
	struct btp_gatt_cl_read_uuid_ev *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC read_uuid event payload");
		return;
	}

	ev = (struct btp_gatt_cl_read_uuid_ev *)payload;
	ev->address = op->address;
	ev->att_response = op->status;
	ev->data_length = sys_cpu_to_le16(op->data_length);
	ev->value_length = op->value_length;
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_READ_UUID_RP,
		     payload, payload_size);

	k_free(payload);
}

static void send_mtu_exchanged_ev(const struct gatt_cl_exchange_mtu_op *op,
				  uint8_t status)
{
	struct btp_gatt_cl_mtu_exchanged_ev ev;

	ev.address = op->address;
	ev.status = status;

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_MTU_EXCHANGED,
		     &ev, sizeof(ev));
}

static void send_discovery_ev(const struct gatt_cl_discovery_op *op)
{
	struct {
		bt_addr_le_t address;
		uint8_t att_response;
		uint8_t count;
		uint8_t data[];
	} __packed *ev;
	size_t payload_size = sizeof(*ev) + op->data_length;
	uint8_t *payload = k_malloc(payload_size);

	if (payload == NULL) {
		LOG_ERR("Failed to allocate GATTC discovery event payload");
		return;
	}

	ev = (void *)payload;
	ev->address = op->address;
	ev->att_response = op->status;
	ev->count = op->count;
	memcpy(ev->data, op->data, op->data_length);

	tester_event(BTP_SERVICE_ID_GATTC, op->event_opcode, payload, payload_size);
	k_free(payload);
}

static bool append_discovery_data(struct gatt_cl_discovery_op *op,
				  const void *data, size_t length)
{
	if ((op->data_length + length) > sizeof(op->data)) {
		return false;
	}

	memcpy(&op->data[op->data_length], data, length);
	op->data_length += length;

	return true;
}

static bool append_discovery_uuid(struct gatt_cl_discovery_op *op,
				  const struct bt_uuid *uuid)
{
	uint8_t uuid_length;
	uint16_t le16;

	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		uuid_length = 2U;
		le16 = sys_cpu_to_le16(BT_UUID_16(uuid)->val);
		return append_discovery_data(op, &uuid_length, sizeof(uuid_length)) &&
		       append_discovery_data(op, &le16, sizeof(le16));
	case BT_UUID_TYPE_128:
		uuid_length = 16U;
		return append_discovery_data(op, &uuid_length, sizeof(uuid_length)) &&
		       append_discovery_data(op, BT_UUID_128(uuid)->val, uuid_length);
	default:
		return false;
	}
}

static void send_status_ev(uint8_t event_opcode, const bt_addr_le_t *address,
			   uint8_t status)
{
	struct {
		bt_addr_le_t address;
		uint8_t att_response;
	} __packed ev;

	ev.address = *address;
	ev.att_response = status;

	tester_event(BTP_SERVICE_ID_GATTC, event_opcode, &ev, sizeof(ev));
}

static struct bt_gatt_subscribe_params *find_subscription(uint16_t ccc_handle)
{
	for (size_t i = 0; i < ARRAY_SIZE(subscriptions); i++) {
		if (subscriptions[i].ccc_handle == ccc_handle) {
			return &subscriptions[i];
		}
	}

	return NULL;
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	struct btp_gatt_cl_notification_ev *ev = (void *)notif_ev_buf;

	if ((conn == NULL) || (data == NULL)) {
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	ev->type = (uint8_t)params->value;
	ev->handle = sys_cpu_to_le16(params->value_handle);
	length = MIN(length, MAX_NOTIF_DATA);
	ev->data_length = sys_cpu_to_le16(length);
	memcpy(ev->data, data, length);
	bt_addr_le_copy(&ev->address, bt_conn_get_dst(conn));

	tester_event(BTP_SERVICE_ID_GATTC, BTP_GATT_CL_EV_NOTIFICATION_RXED,
		     ev, sizeof(*ev) + length);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t read_multiple_var_cb(struct bt_conn *conn, uint8_t err,
				    struct bt_gatt_read_params *params,
				    const void *data, uint16_t length)
{
	struct gatt_cl_read_multiple_var_op *op =
		CONTAINER_OF(params, struct gatt_cl_read_multiple_var_op, params);

	ARG_UNUSED(conn);

	if (err != 0U) {
		/*
		 * Preserve legacy semantics:
		 * - status: BTP transport/procedure status (0 for completed request)
		 * - att_response: ATT error returned by peer
		 */
		op->status = BTP_STATUS_SUCCESS;
		op->att_response = err;
	}

	if (data != NULL) {
		if ((op->data_length + length) > sizeof(op->data)) {
			op->status = BTP_STATUS_FAILED;
			op->att_response = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
			send_read_multiple_var_ev(op);
			free_op(op);
			return BT_GATT_ITER_STOP;
		}

		memcpy(&op->data[op->data_length], data, length);
		op->data_length += length;
		return BT_GATT_ITER_CONTINUE;
	}

	send_read_multiple_var_ev(op);
	free_op(op);

	return BT_GATT_ITER_STOP;
}

static uint8_t read_long_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_read_params *params,
			    const void *data, uint16_t length)
{
	struct gatt_cl_read_long_op *op =
		CONTAINER_OF(params, struct gatt_cl_read_long_op, params);

	ARG_UNUSED(conn);

	if (err != 0U) {
		op->status = err;
	}

	if (data != NULL) {
		if ((op->data_length + length) > sizeof(op->data)) {
			op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
			send_read_long_ev(op);
			free_read_long_op(op);
			return BT_GATT_ITER_STOP;
		}

		memcpy(&op->data[op->data_length], data, length);
		op->data_length += length;
		return BT_GATT_ITER_CONTINUE;
	}

	send_read_long_ev(op);
	free_read_long_op(op);

	return BT_GATT_ITER_STOP;
}

static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
		       struct bt_gatt_read_params *params,
		       const void *data, uint16_t length)
{
	struct gatt_cl_read_op *op =
		CONTAINER_OF(params, struct gatt_cl_read_op, params);

	ARG_UNUSED(conn);

	if (err != 0U) {
		op->status = err;
	}

	if (data != NULL) {
		if ((op->data_length + length) > sizeof(op->data)) {
			op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
			send_read_ev(op);
			free_read_op(op);
			return BT_GATT_ITER_STOP;
		}

		memcpy(&op->data[op->data_length], data, length);
		op->data_length += length;
		return BT_GATT_ITER_CONTINUE;
	}

	send_read_ev(op);
	free_read_op(op);

	return BT_GATT_ITER_STOP;
}

static uint8_t read_multiple_cb(struct bt_conn *conn, uint8_t err,
				struct bt_gatt_read_params *params,
				const void *data, uint16_t length)
{
	struct gatt_cl_read_multiple_op *op =
		CONTAINER_OF(params, struct gatt_cl_read_multiple_op, params);

	ARG_UNUSED(conn);

	if (err != 0U) {
		op->status = err;
	}

	if (data != NULL) {
		if ((op->data_length + length) > sizeof(op->data)) {
			op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
			send_read_multiple_ev(op);
			free_read_multiple_op(op);
			return BT_GATT_ITER_STOP;
		}

		memcpy(&op->data[op->data_length], data, length);
		op->data_length += length;
		return BT_GATT_ITER_CONTINUE;
	}

	send_read_multiple_ev(op);
	free_read_multiple_op(op);

	return BT_GATT_ITER_STOP;
}

static uint8_t read_uuid_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_read_params *params,
			    const void *data, uint16_t length)
{
	struct gatt_cl_read_uuid_op *op =
		CONTAINER_OF(params, struct gatt_cl_read_uuid_op, params);

	ARG_UNUSED(conn);

	if (err != 0U) {
		op->status = err;
	}

	if (data != NULL) {
		uint16_t be_handle;
		size_t tuple_len;

		if (op->value_length == 0U) {
			op->value_length = (uint8_t)length;
		}

		if (length != op->value_length) {
			op->status = BT_ATT_ERR_UNLIKELY;
			send_read_uuid_ev(op);
			free_read_uuid_op(op);
			return BT_GATT_ITER_STOP;
		}

		tuple_len = sizeof(be_handle) + length;
		if ((op->data_length + tuple_len) > sizeof(op->data)) {
			op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
			send_read_uuid_ev(op);
			free_read_uuid_op(op);
			return BT_GATT_ITER_STOP;
		}

		/*
		 * Auto-PTS GATTC parser decodes each tuple as big-endian handle
		 * followed by fixed-length value.
		 */
		be_handle = sys_cpu_to_be16(params->by_uuid.start_handle);
		memcpy(&op->data[op->data_length], &be_handle, sizeof(be_handle));
		op->data_length += sizeof(be_handle);
		memcpy(&op->data[op->data_length], data, length);
		op->data_length += length;
		return BT_GATT_ITER_CONTINUE;
	}

	send_read_uuid_ev(op);
	free_read_uuid_op(op);

	return BT_GATT_ITER_STOP;
}

static void exchange_mtu_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	struct gatt_cl_exchange_mtu_op *op =
		CONTAINER_OF(params, struct gatt_cl_exchange_mtu_op, params);

	ARG_UNUSED(conn);

	send_mtu_exchanged_ev(op, err);
	free_exchange_mtu_op(op);
}

static uint8_t disc_prim_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    struct bt_gatt_discover_params *params)
{
	struct gatt_cl_discovery_op *op =
		CONTAINER_OF(params, struct gatt_cl_discovery_op, params);
	struct bt_gatt_service_val *data;
	uint16_t start_handle;
	uint16_t end_handle;

	ARG_UNUSED(conn);

	if (attr == NULL) {
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	data = attr->user_data;
	start_handle = sys_cpu_to_le16(attr->handle);
	end_handle = sys_cpu_to_le16(data->end_handle);

	if (!append_discovery_data(op, &start_handle, sizeof(start_handle)) ||
	    !append_discovery_data(op, &end_handle, sizeof(end_handle)) ||
	    !append_discovery_uuid(op, data->uuid)) {
		op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	op->count++;
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t find_included_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params)
{
	struct gatt_cl_discovery_op *op =
		CONTAINER_OF(params, struct gatt_cl_discovery_op, params);
	struct bt_gatt_include *data;
	uint16_t included_handle;
	uint16_t start_handle;
	uint16_t end_handle;

	ARG_UNUSED(conn);

	if (attr == NULL) {
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	data = attr->user_data;
	included_handle = sys_cpu_to_le16(attr->handle);
	start_handle = sys_cpu_to_le16(data->start_handle);
	end_handle = sys_cpu_to_le16(data->end_handle);

	if (!append_discovery_data(op, &included_handle, sizeof(included_handle)) ||
	    !append_discovery_data(op, &start_handle, sizeof(start_handle)) ||
	    !append_discovery_data(op, &end_handle, sizeof(end_handle)) ||
	    !append_discovery_uuid(op, data->uuid)) {
		op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	op->count++;
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t disc_chrc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    struct bt_gatt_discover_params *params)
{
	struct gatt_cl_discovery_op *op =
		CONTAINER_OF(params, struct gatt_cl_discovery_op, params);
	struct bt_gatt_chrc *data;
	uint16_t characteristic_handle;
	uint16_t value_handle;

	ARG_UNUSED(conn);

	if (attr == NULL) {
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	data = attr->user_data;
	characteristic_handle = sys_cpu_to_le16(attr->handle);
	value_handle = sys_cpu_to_le16(data->value_handle);

	if (!append_discovery_data(op, &characteristic_handle, sizeof(characteristic_handle)) ||
	    !append_discovery_data(op, &value_handle, sizeof(value_handle)) ||
	    !append_discovery_data(op, &data->properties, sizeof(data->properties)) ||
	    !append_discovery_uuid(op, data->uuid)) {
		op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	op->count++;
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t disc_desc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    struct bt_gatt_discover_params *params)
{
	struct gatt_cl_discovery_op *op =
		CONTAINER_OF(params, struct gatt_cl_discovery_op, params);
	uint16_t descriptor_handle;

	ARG_UNUSED(conn);

	if (attr == NULL) {
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	descriptor_handle = sys_cpu_to_le16(attr->handle);
	if (!append_discovery_data(op, &descriptor_handle, sizeof(descriptor_handle)) ||
	    !append_discovery_uuid(op, attr->uuid)) {
		op->status = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		send_discovery_ev(op);
		free_discovery_op(op);
		return BT_GATT_ITER_STOP;
	}

	op->count++;
	return BT_GATT_ITER_CONTINUE;
}

static void write_cb(struct bt_conn *conn, uint8_t err,
		     struct bt_gatt_write_params *params)
{
	struct gatt_cl_write_op *op =
		CONTAINER_OF(params, struct gatt_cl_write_op, params);

	ARG_UNUSED(conn);

	send_status_ev(op->event_opcode, &op->address, err);
	free_write_op(op);
}

static int disable_subscription(struct bt_conn *conn, uint16_t ccc_handle)
{
	struct bt_gatt_subscribe_params *subscription;

	subscription = find_subscription(ccc_handle);
	if (subscription == NULL) {
		return -EINVAL;
	}

	if (bt_gatt_unsubscribe(conn, subscription) < 0) {
		return -EBUSY;
	}

	(void)memset(subscription, 0, sizeof(*subscription));
	return 0;
}

static void cfg_discover_complete(struct bt_conn *conn,
				  struct bt_gatt_discover_params *params)
{
	struct gatt_cl_cfg_subscribe_op *op =
		CONTAINER_OF(params, struct gatt_cl_cfg_subscribe_op, params);
	struct bt_gatt_subscribe_params *subscription;
	uint8_t status = BTP_STATUS_FAILED;

	subscription = find_subscription(op->ccc_handle);
	if ((subscription != NULL) && (subscription->value_handle != 0U)) {
#if defined(CONFIG_BT_EATT)
		subscription->chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif
		if (bt_gatt_subscribe(conn, subscription) == 0) {
			status = BTP_STATUS_SUCCESS;
		}
	}

	if ((status != BTP_STATUS_SUCCESS) && (subscription != NULL)) {
		(void)memset(subscription, 0, sizeof(*subscription));
	}

	send_status_ev(op->event_opcode, &op->address, status);
	free_cfg_op(op);
}

static uint8_t cfg_discover_func(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 struct bt_gatt_discover_params *params)
{
	struct gatt_cl_cfg_subscribe_op *op =
		CONTAINER_OF(params, struct gatt_cl_cfg_subscribe_op, params);
	struct bt_gatt_subscribe_params *subscription;

	if (attr == NULL) {
		cfg_discover_complete(conn, params);
		return BT_GATT_ITER_STOP;
	}

	subscription = find_subscription(op->ccc_handle);
	if (subscription == NULL) {
		cfg_discover_complete(conn, params);
		return BT_GATT_ITER_STOP;
	}

	/* Characteristic Value Handle is next handle beyond declaration. */
	subscription->value_handle = attr->handle + 1U;
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t start_subscription(struct bt_conn *conn,
				  struct gatt_cl_cfg_subscribe_op *op)
{
	struct bt_gatt_subscribe_params *subscription;

	subscription = find_subscription(UNUSED_SUBSCRIBE_CCC_HANDLE);
	if (subscription == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->params.uuid = NULL;
	op->params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	op->params.end_handle = op->ccc_handle;
	op->params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
	op->params.func = cfg_discover_func;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	subscription->ccc_handle = op->ccc_handle;
	subscription->value = op->value;
	subscription->notify = notify_func;
	subscription->min_security = bt_conn_get_security(conn);

	if (bt_gatt_discover(conn, &op->params) < 0) {
		(void)memset(subscription, 0, sizeof(*subscription));
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t supported_commands(const void *cmd, uint16_t cmd_len,
				  void *rsp, uint16_t *rsp_len)
{
	struct btp_gatt_cl_read_supported_commands_rp *rp = rsp;

	ARG_UNUSED(cmd);
	ARG_UNUSED(cmd_len);

	*rsp_len = tester_supported_commands(BTP_SERVICE_ID_GATTC, rp->data);
	*rsp_len += sizeof(*rp);

	return BTP_STATUS_SUCCESS;
}

static uint8_t read_multiple_var(const void *cmd, uint16_t cmd_len,
				 void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_read_multiple_var_cmd *cp = cmd;
	struct gatt_cl_read_multiple_var_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + (cp->handles_count * sizeof(cp->handles[0])))) {
		return BTP_STATUS_FAILED;
	}

	/* BTP API requires at least two handles for READ_MULTIPLE_VAR. */
	if ((cp->handles_count < 2U) || (cp->handles_count > GATT_CL_MAX_HANDLES)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->status = BTP_STATUS_SUCCESS;
	op->att_response = 0U;
	op->handles_count = cp->handles_count;

	for (size_t i = 0; i < cp->handles_count; i++) {
		op->handles[i] = sys_le16_to_cpu(cp->handles[i]);
	}

	op->params.func = read_multiple_var_cb;
	op->params.handle_count = op->handles_count;
	op->params.multiple.handles = op->handles;
	op->params.multiple.variable = true;
#if defined(CONFIG_BT_EATT)
#if defined(CONFIG_BTTESTER_GATTC_FORCE_EATT_READ_MULT_VAR)
	op->params.chan_opt = BT_ATT_CHAN_OPT_ENHANCED_ONLY;
#else
	/* Alternate bearer selection for back-to-back requests used by GAR WID 147. */
	if ((atomic_inc(&read_multiple_var_seq) & 0x1) == 0) {
		op->params.chan_opt = BT_ATT_CHAN_OPT_UNENHANCED_ONLY;
	} else {
		op->params.chan_opt = BT_ATT_CHAN_OPT_ENHANCED_ONLY;
	}
#endif
#endif

	if (bt_gatt_read(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_read failed");
		free_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t exchange_mtu(const void *cmd, uint16_t cmd_len,
			    void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_exchange_mtu_cmd *cp = cmd;
	struct gatt_cl_exchange_mtu_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_exchange_mtu_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_exchange_mtu_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->params.func = exchange_mtu_cb;

	if (bt_gatt_exchange_mtu(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_exchange_mtu failed");
		free_exchange_mtu_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t disc_all_prim(const void *cmd, uint16_t cmd_len,
			     void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_disc_all_prim_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_DISC_ALL_PRIM_RP;
	op->params.uuid = NULL;
	op->params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	op->params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	op->params.type = BT_GATT_DISCOVER_PRIMARY;
	op->params.func = disc_prim_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (DISC_ALL_PRIM) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t disc_prim_uuid(const void *cmd, uint16_t cmd_len,
			      void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_disc_prim_uuid_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) || (cmd_len != sizeof(*cp) + cp->uuid_length)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	if (btp2bt_uuid(cp->uuid, cp->uuid_length, &op->uuid.uuid)) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_DISC_PRIM_UUID_RP;
	op->params.uuid = &op->uuid.uuid;
	op->params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	op->params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	op->params.type = BT_GATT_DISCOVER_PRIMARY;
	op->params.func = disc_prim_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (DISC_PRIM_UUID) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t find_included(const void *cmd, uint16_t cmd_len,
			     void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_find_included_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_FIND_INCLUDED_RP;
	op->params.uuid = NULL;
	op->params.start_handle = sys_le16_to_cpu(cp->start_handle);
	op->params.end_handle = sys_le16_to_cpu(cp->end_handle);
	op->params.type = BT_GATT_DISCOVER_INCLUDE;
	op->params.func = find_included_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (FIND_INCLUDED) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t disc_all_chrc(const void *cmd, uint16_t cmd_len,
			     void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_disc_all_chrc_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_DISC_ALL_CHRC_RP;
	op->params.uuid = NULL;
	op->params.start_handle = sys_le16_to_cpu(cp->start_handle);
	op->params.end_handle = sys_le16_to_cpu(cp->end_handle);
	op->params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
	op->params.func = disc_chrc_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (DISC_ALL_CHRC) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t disc_chrc_uuid(const void *cmd, uint16_t cmd_len,
			      void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_disc_chrc_uuid_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) || (cmd_len != sizeof(*cp) + cp->uuid_length)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	if (btp2bt_uuid(cp->uuid, cp->uuid_length, &op->uuid.uuid)) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_DISC_CHRC_UUID_RP;
	op->params.uuid = &op->uuid.uuid;
	op->params.start_handle = sys_le16_to_cpu(cp->start_handle);
	op->params.end_handle = sys_le16_to_cpu(cp->end_handle);
	op->params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
	op->params.func = disc_chrc_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (DISC_CHRC_UUID) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t disc_all_desc(const void *cmd, uint16_t cmd_len,
			     void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_disc_all_desc_cmd *cp = cmd;
	struct gatt_cl_discovery_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_discovery_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_DISC_ALL_DESC_RP;
	op->params.uuid = NULL;
	op->params.start_handle = sys_le16_to_cpu(cp->start_handle);
	op->params.end_handle = sys_le16_to_cpu(cp->end_handle);
	op->params.type = BT_GATT_DISCOVER_DESCRIPTOR;
	op->params.func = disc_desc_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_discover(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_discover (DISC_ALL_DESC) failed");
		free_discovery_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t write_without_rsp(const void *cmd, uint16_t cmd_len,
				 void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_write_without_rsp_cmd *cp = cmd;
	struct bt_conn *conn;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + sys_le16_to_cpu(cp->data_length))) {
		return BTP_STATUS_FAILED;
	}

	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (conn == NULL) {
		return BTP_STATUS_FAILED;
	}

	if (bt_gatt_write_without_response(conn, sys_le16_to_cpu(cp->handle), cp->data,
					   sys_le16_to_cpu(cp->data_length), false) < 0) {
		bt_conn_unref(conn);
		return BTP_STATUS_FAILED;
	}

	bt_conn_unref(conn);
	return BTP_STATUS_SUCCESS;
}

static uint8_t signed_write_without_rsp(const void *cmd, uint16_t cmd_len,
					void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_signed_write_without_rsp_cmd *cp = cmd;
	struct bt_conn *conn;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + sys_le16_to_cpu(cp->data_length))) {
		return BTP_STATUS_FAILED;
	}

	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (conn == NULL) {
		return BTP_STATUS_FAILED;
	}

	if (bt_gatt_write_without_response(conn, sys_le16_to_cpu(cp->handle), cp->data,
					   sys_le16_to_cpu(cp->data_length), true) < 0) {
		bt_conn_unref(conn);
		return BTP_STATUS_FAILED;
	}

	bt_conn_unref(conn);
	return BTP_STATUS_SUCCESS;
}

static uint8_t write_cmd(const void *cmd, uint16_t cmd_len,
			 void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_write_cmd *cp = cmd;
	struct gatt_cl_write_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + sys_le16_to_cpu(cp->data_length))) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_write_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_WRITE_RP;
	op->params.handle = sys_le16_to_cpu(cp->handle);
	op->params.offset = 0U;
	op->params.data = cp->data;
	op->params.length = sys_le16_to_cpu(cp->data_length);
	op->params.func = write_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_write(op->conn, &op->params) < 0) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t write_long(const void *cmd, uint16_t cmd_len,
			  void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_write_long_cmd *cp = cmd;
	struct gatt_cl_write_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + sys_le16_to_cpu(cp->data_length))) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_write_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_WRITE_LONG_RP;
	op->params.handle = sys_le16_to_cpu(cp->handle);
	op->params.offset = sys_le16_to_cpu(cp->offset);
	op->params.data = cp->data;
	op->params.length = sys_le16_to_cpu(cp->data_length);
	op->params.func = write_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_write(op->conn, &op->params) < 0) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t write_reliable(const void *cmd, uint16_t cmd_len,
			      void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_write_reliable_cmd *cp = cmd;
	struct gatt_cl_write_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + sys_le16_to_cpu(cp->data_length))) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_write_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = BTP_GATT_CL_EV_RELIABLE_WRITE_RP;
	op->params.handle = sys_le16_to_cpu(cp->handle);
	op->params.offset = sys_le16_to_cpu(cp->offset);
	op->params.data = cp->data;
	op->params.length = sys_le16_to_cpu(cp->data_length);
	op->params.func = write_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_write(op->conn, &op->params) < 0) {
		free_write_op(op);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static uint8_t config_subscription(const void *cmd, uint16_t cmd_len,
				   uint8_t event_opcode, uint16_t ccc_value)
{
	const struct btp_gatt_cl_cfg_notify_cmd *cp = cmd;
	struct gatt_cl_cfg_subscribe_op *op;

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_cfg_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_cfg_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->event_opcode = event_opcode;
	op->ccc_handle = sys_le16_to_cpu(cp->ccc_handle);
	op->value = ccc_value;

	if (cp->enable == 0U) {
		if (disable_subscription(op->conn, op->ccc_handle) < 0) {
			send_status_ev(event_opcode, &op->address, BTP_STATUS_FAILED);
		} else {
			send_status_ev(event_opcode, &op->address, BTP_STATUS_SUCCESS);
		}

		free_cfg_op(op);
		return BTP_STATUS_SUCCESS;
	}

	if (start_subscription(op->conn, op) != BTP_STATUS_SUCCESS) {
		send_status_ev(event_opcode, &op->address, BTP_STATUS_FAILED);
		free_cfg_op(op);
		return BTP_STATUS_SUCCESS;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t cfg_notify(const void *cmd, uint16_t cmd_len,
			  void *rsp, uint16_t *rsp_len)
{
	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	return config_subscription(cmd, cmd_len, BTP_GATT_CL_EV_CFG_NOTIFY_RP,
				   BT_GATT_CCC_NOTIFY);
}

static uint8_t cfg_indicate(const void *cmd, uint16_t cmd_len,
			    void *rsp, uint16_t *rsp_len)
{
	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	return config_subscription(cmd, cmd_len, BTP_GATT_CL_EV_CFG_INDICATE_RP,
				   BT_GATT_CCC_INDICATE);
}

static uint8_t read_long(const void *cmd, uint16_t cmd_len,
			 void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_read_long_cmd *cp = cmd;
	struct gatt_cl_read_long_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_read_long_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_read_long_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->status = 0U;
	op->params.func = read_long_cb;
	op->params.handle_count = 1;
	op->params.single.handle = sys_le16_to_cpu(cp->handle);
	op->params.single.offset = sys_le16_to_cpu(cp->offset);
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_read(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_read (READ_LONG) failed");
		free_read_long_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t read_cmd(const void *cmd, uint16_t cmd_len,
			void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_read_cmd *cp = cmd;
	struct gatt_cl_read_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_read_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_read_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->status = 0U;
	op->params.func = read_cb;
	op->params.handle_count = 1;
	op->params.single.handle = sys_le16_to_cpu(cp->handle);
	op->params.single.offset = 0U;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_read(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_read (READ) failed");
		free_read_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t read_multiple(const void *cmd, uint16_t cmd_len,
			     void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_read_multiple_cmd *cp = cmd;
	struct gatt_cl_read_multiple_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) ||
	    (cmd_len != sizeof(*cp) + (cp->handles_count * sizeof(cp->handles[0])))) {
		return BTP_STATUS_FAILED;
	}

	if ((cp->handles_count == 0U) || (cp->handles_count > GATT_CL_MAX_HANDLES)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_read_multiple_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_read_multiple_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->status = 0U;
	op->handles_count = cp->handles_count;

	for (size_t i = 0; i < cp->handles_count; i++) {
		op->handles[i] = sys_le16_to_cpu(cp->handles[i]);
	}

	op->params.func = read_multiple_cb;
	op->params.handle_count = op->handles_count;
	op->params.multiple.handles = op->handles;
	op->params.multiple.variable = false;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_read(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_read (READ_MULTIPLE) failed");
		free_read_multiple_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t read_uuid(const void *cmd, uint16_t cmd_len,
			 void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_read_uuid_cmd *cp = cmd;
	struct gatt_cl_read_uuid_op *op;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if ((cmd_len < sizeof(*cp)) || (cmd_len != sizeof(*cp) + cp->uuid_length)) {
		return BTP_STATUS_FAILED;
	}

	op = alloc_read_uuid_op();
	if (op == NULL) {
		return BTP_STATUS_FAILED;
	}

	if (btp2bt_uuid(cp->uuid, cp->uuid_length, &op->uuid.uuid)) {
		free_read_uuid_op(op);
		return BTP_STATUS_FAILED;
	}

	op->conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (op->conn == NULL) {
		free_read_uuid_op(op);
		return BTP_STATUS_FAILED;
	}

	op->address = cp->address;
	op->status = 0U;
	op->params.by_uuid.uuid = &op->uuid.uuid;
	op->params.handle_count = 0;
	op->params.by_uuid.start_handle = sys_le16_to_cpu(cp->start_handle);
	op->params.by_uuid.end_handle = sys_le16_to_cpu(cp->end_handle);
	op->params.func = read_uuid_cb;
#if defined(CONFIG_BT_EATT)
	op->params.chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

	if (bt_gatt_read(op->conn, &op->params) < 0) {
		LOG_ERR("bt_gatt_read (READ_UUID) failed");
		free_read_uuid_op(op);
		return BTP_STATUS_FAILED;
	}

	/* Command ACK first, result is emitted as async event. */
	return BTP_STATUS_SUCCESS;
}

static uint8_t eatt_connect(const void *cmd, uint16_t cmd_len,
			    void *rsp, uint16_t *rsp_len)
{
	const struct btp_gatt_cl_eatt_connect_cmd *cp = cmd;
	struct bt_conn *conn;
	int err;
	int retries_left;

	ARG_UNUSED(rsp);
	ARG_UNUSED(rsp_len);

	if (cmd_len != sizeof(*cp)) {
		return BTP_STATUS_FAILED;
	}

#if defined(CONFIG_BT_EATT)
#if !defined(CONFIG_BTTESTER_GATTC_FORCE_EATT_READ_MULT_VAR)
	/* Start each WID 400/WID 147 sequence deterministically on ATT first. */
	atomic_set(&read_multiple_var_seq, 0);
#endif
#endif

	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &cp->address);
	if (conn == NULL) {
		LOG_ERR("EATT_CONNECT connection not found");
		return BTP_STATUS_FAILED;
	}

	/*
	 * Right after pairing/security procedures the host can temporarily run
	 * out of buffers for ECRED setup. Retry for a few seconds before
	 * reporting command failure.
	 */
	err = -ENOMEM;
	retries_left = GATT_CL_EATT_CONNECT_RETRIES;
	while ((retries_left > 0) && (err == -ENOMEM)) {
		err = bt_eatt_connect(conn, cp->num_channels);
		if (err == -ENOMEM) {
			k_sleep(K_MSEC(GATT_CL_EATT_CONNECT_RETRY_DELAY_MS));
		}
		retries_left--;
	}

	bt_conn_unref(conn);
	if ((err != 0) && (err != -EALREADY) && (err != -EINPROGRESS)) {
		LOG_ERR("EATT_CONNECT failed (%d)", err);
		return BTP_STATUS_FAILED;
	}

	return BTP_STATUS_SUCCESS;
}

static const struct btp_handler handlers[] = {
	{
		.opcode = BTP_GATT_CL_READ_SUPPORTED_COMMANDS,
		.index = BTP_INDEX_NONE,
		.expect_len = 0,
		.func = supported_commands,
	},
	{
		.opcode = BTP_GATT_CL_EATT_CONNECT,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_eatt_connect_cmd),
		.func = eatt_connect,
	},
	{
		.opcode = BTP_GATT_CL_EXCHANGE_MTU,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_exchange_mtu_cmd),
		.func = exchange_mtu,
	},
	{
		.opcode = BTP_GATT_CL_DISC_ALL_PRIM,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_disc_all_prim_cmd),
		.func = disc_all_prim,
	},
	{
		.opcode = BTP_GATT_CL_DISC_PRIM_UUID,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = disc_prim_uuid,
	},
	{
		.opcode = BTP_GATT_CL_FIND_INCLUDED,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_find_included_cmd),
		.func = find_included,
	},
	{
		.opcode = BTP_GATT_CL_DISC_ALL_CHRC,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_disc_all_chrc_cmd),
		.func = disc_all_chrc,
	},
	{
		.opcode = BTP_GATT_CL_DISC_CHRC_UUID,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = disc_chrc_uuid,
	},
	{
		.opcode = BTP_GATT_CL_DISC_ALL_DESC,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_disc_all_desc_cmd),
		.func = disc_all_desc,
	},
	{
		.opcode = BTP_GATT_CL_WRITE_WITHOUT_RSP,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = write_without_rsp,
	},
	{
		.opcode = BTP_GATT_CL_SIGNED_WRITE_WITHOUT_RSP,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = signed_write_without_rsp,
	},
	{
		.opcode = BTP_GATT_CL_WRITE,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = write_cmd,
	},
	{
		.opcode = BTP_GATT_CL_WRITE_LONG,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = write_long,
	},
	{
		.opcode = BTP_GATT_CL_WRITE_RELIABLE,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = write_reliable,
	},
	{
		.opcode = BTP_GATT_CL_CFG_NOTIFY,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_cfg_notify_cmd),
		.func = cfg_notify,
	},
	{
		.opcode = BTP_GATT_CL_CFG_INDICATE,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_cfg_notify_cmd),
		.func = cfg_indicate,
	},
	{
		.opcode = BTP_GATT_CL_READ,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_read_cmd),
		.func = read_cmd,
	},
	{
		.opcode = BTP_GATT_CL_READ_UUID,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = read_uuid,
	},
	{
		.opcode = BTP_GATT_CL_READ_LONG,
		.index = BTP_INDEX,
		.expect_len = sizeof(struct btp_gatt_cl_read_long_cmd),
		.func = read_long,
	},
	{
		.opcode = BTP_GATT_CL_READ_MULTIPLE,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = read_multiple,
	},
	{
		.opcode = BTP_GATT_CL_READ_MULTIPLE_VAR,
		.index = BTP_INDEX,
		.expect_len = BTP_HANDLER_LENGTH_VARIABLE,
		.func = read_multiple_var,
	},
};

uint8_t tester_init_gatt_cl(void)
{
	tester_register_command_handlers(BTP_SERVICE_ID_GATTC, handlers,
					 ARRAY_SIZE(handlers));

	return BTP_STATUS_SUCCESS;
}

uint8_t tester_unregister_gatt_cl(void)
{
	return BTP_STATUS_SUCCESS;
}
