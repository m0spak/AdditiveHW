// Zephyr BLE — GATT service: write sensor value (UTF-8), notify forecast.
// west build -b nrf52840dongle/nrf52840
// Test: nRF Connect app → write UTF-8 string to 12340001 (e.g. "23.6"),
//       subscribe to 12340002 for human-readable forecast notifications.
//
// Concurrency model:
//   * BT RX thread runs only on_write / on_read / on_ccc — never touches
//     the model.  Sensor values land in a Zephyr message queue; forecast
//     reads pull from a single-element seqlock-published snapshot.
//   * Main thread is the SOLE owner of the AdditiveHW model.  It drains
//     the queue, advances the model, runs ahw_process(), captures a
//     forecast, and publishes a fresh snapshot.
//   * No mutex is held across snprintf or bt_gatt_notify.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include <stdlib.h>
#include "additive_hw_api.h"

LOG_MODULE_REGISTER(ahw_ble, LOG_LEVEL_INF);

#define SEASON 12

// ── Queue: BT RX thread → main ────────────────────────────────────────
//  Single-producer / single-consumer (BT RX → main).  Producer uses
//  K_NO_WAIT so the radio stack never blocks on a slow consumer.  If
//  the queue overflows the sample is dropped and logged.
K_MSGQ_DEFINE(sensor_q, sizeof(float), 16, 4);

// ── Forecast snapshot (seqlock, single writer = main) ─────────────────
//  Writer bumps `version` even → odd → even around the stores.
//  Readers retry while version is odd or differs across the read.
//  No mutex; on a single-core MCU compile-time fences are sufficient.
typedef struct {
    atomic_t version;           // even = stable, odd = writer in progress
    bool     ready;
    float    yhat, lower, upper;
} forecast_snap_t;
static forecast_snap_t snap;

static atomic_t notify_on;

// ── UUIDs ──────────────────────────────────────────────────────────────
#define SVC_UUID_VAL  BT_UUID_128_ENCODE(0x12340000,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define DATA_UUID_VAL BT_UUID_128_ENCODE(0x12340001,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define FC_UUID_VAL   BT_UUID_128_ENCODE(0x12340002,0x0000,0x1000,0x8000,0x00805f9b34fb)
#define SVC_UUID      BT_UUID_DECLARE_128(SVC_UUID_VAL)
#define DATA_UUID     BT_UUID_DECLARE_128(DATA_UUID_VAL)
#define FC_UUID       BT_UUID_DECLARE_128(FC_UUID_VAL)

// ── float → UTF-8 helper (no %f, often unavailable on embedded) ───────
static int fmt_float(char *buf, size_t sz, const char *label, float val) {
    int integer = (int)val;
    int frac = abs((int)(val * 10.0f) % 10);
    if (val < 0.0f && integer == 0) {
        return snprintf(buf, sz, "%s=-%d.%d", label, integer, frac);
    }
    return snprintf(buf, sz, "%s=%d.%d", label, integer, frac);
}

// ── Snapshot publish (called only from main thread) ───────────────────
static void publish_snapshot(bool ready, float y, float lo, float hi) {
    int v = atomic_get(&snap.version);
    atomic_set(&snap.version, v + 1);                // → odd
    __atomic_signal_fence(__ATOMIC_RELEASE);
    snap.ready = ready;
    snap.yhat  = y;
    snap.lower = lo;
    snap.upper = hi;
    __atomic_signal_fence(__ATOMIC_RELEASE);
    atomic_set(&snap.version, v + 2);                // → even
}

// ── Snapshot read with retry (any context) ────────────────────────────
static bool read_snapshot(forecast_snap_t *out) {
    for (int retries = 0; retries < 8; ++retries) {
        int v0 = atomic_get(&snap.version);
        if (v0 & 1) continue;                         // writer mid-update
        __atomic_signal_fence(__ATOMIC_ACQUIRE);
        forecast_snap_t local = snap;
        __atomic_signal_fence(__ATOMIC_ACQUIRE);
        int v1 = atomic_get(&snap.version);
        if (v0 == v1) { *out = local; return true; }
    }
    return false;
}

// ── Format the snapshot as UTF-8 for a notify or read ─────────────────
static int format_snapshot(char *buf, size_t sz, const forecast_snap_t *s) {
    if (s->ready) {
        char s1[16], s2[16], s3[16];
        fmt_float(s1, sizeof(s1), "fc", s->yhat);
        fmt_float(s2, sizeof(s2), "lo", s->lower);
        fmt_float(s3, sizeof(s3), "hi", s->upper);
        return snprintf(buf, sz, "%s %s %s", s1, s2, s3);
    }
    return snprintf(buf, sz, "waiting for data...");
}

// ── BLE write callback (BT RX thread) ─────────────────────────────────
//  Parses the UTF-8 number string and drops the float into the queue.
//  Never touches the model.  Wait-free w.r.t. main.
static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len,
                        uint16_t offset, uint8_t flags) {
    ARG_UNUSED(conn); ARG_UNUSED(attr);
    ARG_UNUSED(offset); ARG_UNUSED(flags);

    char tmp[16];
    if (len == 0 || len >= sizeof(tmp)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    memcpy(tmp, buf, len);
    tmp[len] = '\0';

    char *end;
    float y = strtof(tmp, &end);
    if (end == tmp) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (k_msgq_put(&sensor_q, &y, K_NO_WAIT) != 0) {
        LOG_WRN("sensor_q full, dropped %d.%01d",
                (int)y, abs((int)(y * 10.0f) % 10));
    }

    LOG_INF("Received: \"%s\" -> %d.%01d",
            tmp, (int)y, abs((int)(y * 10.0f) % 10));
    return len;
}

// ── BLE read callback (BT RX thread) ──────────────────────────────────
//  Reads only the snapshot — never touches the model.
static ssize_t on_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset) {
    char out[80];
    forecast_snap_t s = {0};
    int n;

    if (read_snapshot(&s)) {
        n = format_snapshot(out, sizeof(out), &s);
    } else {
        n = snprintf(out, sizeof(out), "waiting for data...");
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, out, n);
}

// ── Notify-on-change CCC ──────────────────────────────────────────────
static void on_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    atomic_set(&notify_on, (value == BT_GATT_CCC_NOTIFY));
}

// ── GATT service table ────────────────────────────────────────────────
//  Index: [0] service  [1] char-decl  [2] char-val (DATA/write)
//         [3] char-decl  [4] char-val (FC/read+notify)  [5] CCC
BT_GATT_SERVICE_DEFINE(ahw_svc,
    BT_GATT_PRIMARY_SERVICE(SVC_UUID),
    BT_GATT_CHARACTERISTIC(DATA_UUID, BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE, NULL, on_write, NULL),
    BT_GATT_CHARACTERISTIC(FC_UUID, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, on_read, NULL, NULL),
    BT_GATT_CCC(on_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SVC_UUID_VAL),
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "AHW-Forecast", sizeof("AHW-Forecast") - 1),
};

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) LOG_ERR("Connect err %u", err);
    else LOG_INF("Connected");
}
static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(conn);
    atomic_set(&notify_on, false);
    LOG_INF("Disconnected (reason %u)", reason);
}
BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

// ── Notify from snapshot — no mutex held across the radio op ──────────
static void notify_from_snapshot(void) {
    forecast_snap_t s = {0};
    if (!read_snapshot(&s)) return;

    char out[80];
    int n = format_snapshot(out, sizeof(out), &s);
    bt_gatt_notify(NULL, &ahw_svc.attrs[4], out, n);
}

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

    // Main loop is the SOLE owner of the model: drain the queue, advance
    // the model, run any pending refit, publish a snapshot, optionally
    // notify.  No mutex anywhere.
    for (;;) {
        float y;
        while (k_msgq_get(&sensor_q, &y, K_NO_WAIT) == 0) {
            ahw_update(y);
        }
        ahw_process();

        ahw_forecast_t f = {0};
        bool ok = ahw_forecast(1, &f);
        publish_snapshot(ok, f.yhat, f.lower, f.upper);

        if (atomic_get(&notify_on)) {
            notify_from_snapshot();
        }
        k_sleep(K_MSEC(100));
    }
}
