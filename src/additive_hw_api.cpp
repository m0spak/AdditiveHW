/**
 * @file    additive_hw_api.cpp
 * @brief   C-callable wrapper around the AdditiveHW C++ class.
 */

#include "AdditiveHW.h"
#include "additive_hw_api.h"

static AdditiveHW s_hw;

extern "C" {

bool ahw_init(int season_len, bool defer_refit) {
    bool ok = s_hw.begin(season_len);
    s_hw.deferRefit(defer_refit);
    return ok;
}

void ahw_update(float y) {
    s_hw.update(y);
}

bool ahw_process(void) {
    return s_hw.processRefit();
}

bool ahw_forecast(int h, ahw_forecast_t* f) {
    if (!s_hw.isReady() || !f) return false;

    HWForecast r = s_hw.forecastPI(h, 1.96f);
    f->yhat  = r.yhat;
    f->lower = r.lower;
    f->upper = r.upper;
    f->sigma = r.sigma;
    return true;
}

bool ahw_is_ready(void) {
    return s_hw.isReady();
}

float ahw_level(void) {
    return s_hw.level();
}

float ahw_trend(void) {
    return s_hw.trend();
}

float ahw_sigma2(void) {
    return s_hw.sigma2();
}

} /* extern "C" */
