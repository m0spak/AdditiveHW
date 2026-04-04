// Zephyr example — timer + forecast.
// west build -b nrf52840dongle/nrf52840
// west build -b bbc_microbit_v2
//
// Thread safety:
//   The timer callback pushes samples into a k_msgq.
//   The main loop drains the queue and calls all model functions
//   from a single thread context — no lock needed.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include "additive_hw_api.h"

LOG_MODULE_REGISTER(ahw, LOG_LEVEL_INF);

#define SEASON 12
#define PERIOD 1000

// Message queue: timer → main loop
K_MSGQ_DEFINE(sensor_q, sizeof(float), 8, 4);

// Replace with your actual sensor read
static float read_sensor(void) {
    static int t = 0;
    t++;
    return 20.0f + 0.1f * t + 5.0f * ((t % SEASON) < SEASON / 2 ? 1.0f : -1.0f);
}

static void timer_fn(struct k_timer *tm) {
    ARG_UNUSED(tm);
    float y = read_sensor();
    k_msgq_put(&sensor_q, &y, K_NO_WAIT);
}
K_TIMER_DEFINE(sensor_timer, timer_fn, NULL);

// Zephyr LOG has no %f — print as fixed point.
// Handles negative values correctly, including the −0.xx case.
static void log_fixed(const char *label, float v) {
    int  whole = (int)v;
    int  frac  = abs((int)(v * 100.0f) % 100);
    bool neg   = (v < 0.0f);

    if (neg && whole == 0) {
        LOG_PRINTK("%s-0.%02d", label, frac);
    } else {
        LOG_PRINTK("%s%d.%02d", label, whole, frac);
    }
}

static void log_forecast(ahw_forecast_t *f) {
    log_fixed("fc=", f->yhat);
    log_fixed(" [", f->lower);
    log_fixed("..", f->upper);
    LOG_PRINTK("]\n");
}

int main(void) {
    ahw_init(SEASON, true);
    k_timer_start(&sensor_timer, K_MSEC(0), K_MSEC(PERIOD));

    for (;;) {
        // Drain all queued samples
        float y;
        while (k_msgq_get(&sensor_q, &y, K_NO_WAIT) == 0) {
            ahw_update(y);
        }

        ahw_process();

        ahw_forecast_t f;
        if (ahw_forecast(1, &f)) {
            log_forecast(&f);
        }

        k_sleep(K_MSEC(PERIOD));
    }
}
