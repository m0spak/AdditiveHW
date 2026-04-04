// Zephyr BLE — GATT service: write sensor float, notify forecast.
// west build -b nrf52840dongle/nrf52840
// Test: nRF Connect app → write float to 12340001, read notify on 12340002.
//
// Thread safety:
//   GATT callbacks run in the BT RX thread, while ahw_process() and
//   the notify path run in the main thread.  A mutex serialises all
//   model access so neither side sees inconsistent state.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>
#include "additive_hw_api.h"

LOG_MODULE_REGISTER(ahw_ble, LOG_LEVEL_INF);

#define SEASON 12

// Mutex protecting all ahw_*() calls
K_MUTEX_DEFINE(ahw_mtx);

// UUIDs
#define SVC_UUID_VAL  BT_UUID_128_ENCODE(0x12340000,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define DATA_UUID_VAL BT_UUID_128_ENCODE(0x12340001,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define FC_UUID_VAL   BT_UUID_128_ENCODE(0x12340002,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define SVC_UUID      BT_UUID_DECLARE_128(SVC_UUID_VAL)
#define DATA_UUID     BT_UUID_DECLARE_128(DATA_UUID_VAL)
#define FC_UUID       BT_UUID_DECLARE_128(FC_UUID_VAL)

static bool notify_on;

// Client writes a sensor float
static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len,
                        uint16_t offset, uint8_t flags) {
    ARG_UNUSED(conn); ARG_UNUSED(attr);
    ARG_UNUSED(offset); ARG_UNUSED(flags);
    if (len != sizeof(float)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    float y;
    memcpy(&y, buf, sizeof(y));

    k_mutex_lock(&ahw_mtx, K_FOREVER);
    ahw_update(y);
    k_mutex_unlock(&ahw_mtx);

    return len;
}

// Client reads the current forecast
static ssize_t on_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset) {
    ahw_forecast_t f = {0};

    k_mutex_lock(&ahw_mtx, K_FOREVER);
    ahw_forecast(1, &f);
    k_mutex_unlock(&ahw_mtx);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &f, sizeof(f));
}

// Client enables/disables notifications
static void on_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    notify_on = (value == BT_GATT_CCC_NOTIFY);
}

// GATT service
BT_GATT_SERVICE_DEFINE(ahw_svc,
    BT_GATT_PRIMARY_SERVICE(SVC_UUID),
    BT_GATT_CHARACTERISTIC(DATA_UUID, BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE, NULL, on_write, NULL),
    BT_GATT_CHARACTERISTIC(FC_UUID, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, on_read, NULL, NULL),
    BT_GATT_CCC(on_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SVC_UUID_VAL),
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "AHW-Forecast", sizeof("AHW-Forecast") - 1),
};

// Connection callbacks
static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) LOG_ERR("Connect err %u", err);
    else LOG_INF("Connected");
}
static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(conn);
    notify_on = false;
}
BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

int main(void) {
    ahw_init(SEASON, true);

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BT init err %d", err);
        for (;;) { k_sleep(K_FOREVER); }
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Adv err %d", err);
        for (;;) { k_sleep(K_FOREVER); }
    }
    LOG_INF("Advertising: AHW-Forecast");

    for (;;) {
        k_mutex_lock(&ahw_mtx, K_FOREVER);
        ahw_process();

        if (notify_on) {
            ahw_forecast_t f = {0};
            if (ahw_forecast(1, &f)) {
                k_mutex_unlock(&ahw_mtx);
                bt_gatt_notify(NULL, &ahw_svc.attrs[4], &f, sizeof(f));
            } else {
                k_mutex_unlock(&ahw_mtx);
            }
        } else {
            k_mutex_unlock(&ahw_mtx);
        }

        k_sleep(K_MSEC(100));
    }
}
