/**
 * @file    additive_hw_api.h
 * @brief   C-callable API for AdditiveHW forecasting library.
 *
 * Use this header from C or C++ code in Zephyr RTOS, nRF Connect SDK,
 * or any embedded project.  The underlying implementation is C++ but
 * this interface uses extern "C" linkage.
 *
 * Usage:
 *   1. Copy src/AdditiveHW.h, src/additive_hw_api.h, src/additive_hw_api.cpp
 *      into your project.
 *   2. Add additive_hw_api.cpp to your build (Makefile, CMakeLists, SES).
 *   3. #include "additive_hw_api.h" from your C code.
 */

#ifndef ADDITIVE_HW_API_H
#define ADDITIVE_HW_API_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Forecast result returned by ahw_forecast(). */
typedef struct {
    float yhat;    /**< Point forecast. */
    float lower;   /**< Lower prediction interval bound. */
    float upper;   /**< Upper prediction interval bound. */
    float sigma;   /**< Forecast standard deviation. */
} ahw_forecast_t;

/**
 * Initialise the forecast model.
 *
 * @param season_len  Seasonal period (1–24).
 * @param defer_refit true = update() is always O(1); call ahw_process() to refit.
 *                    false = update() runs the optimizer inline when due.
 */
void ahw_init(int season_len, bool defer_refit);

/**
 * Feed one new observation.
 * O(1) when defer_refit = true, but NOT ISR-safe without external
 * synchronisation — use a message queue or mutex to protect concurrent
 * access to the model (see examples/zephyr/ and examples/zephyr_ble/).
 */
void ahw_update(float y);

/**
 * Run pending parameter refit.
 * Call from main loop / idle thread.
 * @return true if a refit was executed, false if nothing was pending.
 */
bool ahw_process(void);

/**
 * Get h-step-ahead forecast with 95% prediction interval.
 * @param h       Forecast horizon (≥ 1).
 * @param[out] f  Result struct.  Unchanged if model is not ready.
 * @return true if the model is ready and f is valid.
 */
bool ahw_forecast(int h, ahw_forecast_t* f);

/** @return true after 2 × season_len observations. */
bool ahw_is_ready(void);

/** @return Current level component (l_t). */
float ahw_level(void);

/** @return Current trend component (b_t). */
float ahw_trend(void);

/** @return One-step residual variance (σ²). */
float ahw_sigma2(void);

#ifdef __cplusplus
}
#endif

#endif /* ADDITIVE_HW_API_H */
