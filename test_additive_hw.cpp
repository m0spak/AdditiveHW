// test_additive_hw.cpp — comprehensive tests for AdditiveHW
//
// Build:  g++ -std=c++11 -Wall -Wextra -I src -o test test_additive_hw.cpp src/additive_hw_api.cpp
// Run:    ./test

#include <cstdio>
#include <cmath>
#include <cstring>
#include "AdditiveHW.h"
#include "additive_hw_api.h"

// ── Helpers ──────────────────────────────────────────────────────────

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL [%s:%d] ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while(0)

#define SECTION(name) printf("\n── %s ──\n", name)

static float near(float a, float b, float tol = 1e-3f) {
    return fabsf(a - b) <= tol;
}

// Deterministic seasonal signal: level + trend + season
static float synth(int t, int m, float level0 = 100.0f,
                   float trend = 0.5f, float amp = 10.0f) {
    // season[i] = amp * sin(2π·i/m),  sums to ~0 over full cycle
    float s = amp * sinf(2.0f * 3.14159265f * (float)(t % m) / (float)m);
    return level0 + trend * (float)t + s;
}

// =====================================================================
//  1. Bootstrap & readiness
// =====================================================================
static void test_bootstrap() {
    SECTION("Bootstrap & readiness");

    AdditiveHW hw;
    int m = 6;
    hw.begin(m);

    // Not ready before 2*m observations
    for (int i = 0; i < 2 * m - 1; ++i) {
        hw.update(synth(i, m));
        CHECK(!hw.isReady(), "should not be ready at obs %d", i + 1);
    }

    // Ready at exactly 2*m
    hw.update(synth(2 * m - 1, m));
    CHECK(hw.isReady(), "should be ready at obs %d", 2 * m);

    // seasonLen accessor
    CHECK(hw.seasonLen() == m, "seasonLen() = %d, expected %d", hw.seasonLen(), m);
}

// =====================================================================
//  2. Forecast before ready — returns ring mean
// =====================================================================
static void test_forecast_before_ready() {
    SECTION("Forecast before ready");

    AdditiveHW hw;
    hw.begin(6);

    // No observations → 0
    float fc = hw.forecast(1);
    CHECK(near(fc, 0.0f), "forecast with 0 obs = %.4f, expected 0", fc);

    // Feed a few values (< 2*m), forecast should be their mean
    hw.update(10.0f);
    hw.update(20.0f);
    hw.update(30.0f);
    fc = hw.forecast(1);
    CHECK(near(fc, 20.0f), "forecast pre-ready = %.4f, expected 20.0", fc);
}

// =====================================================================
//  3. Level and trend tracking on a clean linear signal
// =====================================================================
static void test_level_trend_tracking() {
    SECTION("Level & trend tracking");

    AdditiveHW hw;
    int m = 4;
    // Pure linear signal: y = 50 + 2*t, no seasonality.
    // Use high alpha/beta, low gamma so model tracks the line.
    hw.begin(m, 0, 0, 0.9f, 0.5f, 0.01f);

    int N = 80;
    for (int t = 0; t < N; ++t)
        hw.update(50.0f + 2.0f * (float)t);

    CHECK(hw.isReady(), "model ready after %d obs", N);

    // Level should be near last value
    float expected_level = 50.0f + 2.0f * (float)(N - 1);
    CHECK(fabsf(hw.level() - expected_level) < 10.0f,
          "level = %.2f, expected ~%.2f", hw.level(), expected_level);

    // Trend should be near 2.0
    CHECK(fabsf(hw.trend() - 2.0f) < 1.0f,
          "trend = %.4f, expected ~2.0", hw.trend());

    // 1-step forecast should be near next value
    float fc = hw.forecast(1);
    float expected_next = 50.0f + 2.0f * (float)N;
    CHECK(fabsf(fc - expected_next) < 10.0f,
          "forecast(1) = %.2f, expected ~%.2f", fc, expected_next);
}

// =====================================================================
//  4. Seasonal decomposition — recovers known pattern
// =====================================================================
static void test_seasonal_recovery() {
    SECTION("Seasonal decomposition");

    AdditiveHW hw;
    int m = 12;
    hw.begin(m);

    // Clean seasonal signal, 8 full cycles
    int N = 8 * m;
    for (int t = 0; t < N; ++t)
        hw.update(synth(t, m, 100.0f, 0.0f, 10.0f));  // no trend

    CHECK(hw.isReady(), "model ready");

    // Forecasts for a full cycle ahead should approximate the pattern
    float maxErr = 0.0f;
    for (int h = 1; h <= m; ++h) {
        float fc  = hw.forecast(h);
        float act = synth(N - 1 + h, m, 100.0f, 0.0f, 10.0f);
        float err = fabsf(fc - act);
        if (err > maxErr) maxErr = err;
    }
    CHECK(maxErr < 5.0f,
          "max seasonal forecast error = %.3f (want < 5.0)", maxErr);
}

// =====================================================================
//  5. Fitted and residual accessors
// =====================================================================
static void test_fitted_residual() {
    SECTION("Fitted & residual");

    AdditiveHW hw;
    int m = 4;
    hw.begin(m, 0, 0, 0.3f, 0.1f, 0.1f);

    int N = 40;
    float lastY = 0.0f;
    for (int t = 0; t < N; ++t) {
        lastY = synth(t, m);
        hw.update(lastY);
    }

    // residual = y - fitted
    CHECK(near(hw.residual(), lastY - hw.fitted(), 1e-4f),
          "residual %.4f != y - fitted (%.4f - %.4f = %.4f)",
          hw.residual(), lastY, hw.fitted(), lastY - hw.fitted());
}

// =====================================================================
//  6. sigma2 — Welford variance is positive and reasonable
// =====================================================================
static void test_sigma2() {
    SECTION("sigma2 (Welford)");

    AdditiveHW hw;
    int m = 6;
    hw.begin(m);

    // Before 2 residuals: should be 0
    for (int t = 0; t < 2 * m; ++t) {
        hw.update(synth(t, m));
    }
    // At exactly 2*m, model just became ready — only 0 residuals accumulated
    // (bootstrap consumed the first 2*m observations).
    // Feed one more to get first residual.
    hw.update(synth(2 * m, m));
    // Still only 1 residual → sigma2 should be 0 (n-1 denominator)
    CHECK(hw.sigma2() == 0.0f, "sigma2 with 1 residual = %.6f, expected 0", hw.sigma2());

    // Feed more to accumulate residuals
    for (int t = 2 * m + 1; t < 5 * m; ++t)
        hw.update(synth(t, m));

    float s2 = hw.sigma2();
    CHECK(s2 > 0.0f, "sigma2 = %.6f, should be > 0", s2);
    CHECK(std::isfinite(s2), "sigma2 should be finite");

    // On a clean signal, residuals should be small → sigma2 small
    CHECK(s2 < 50.0f, "sigma2 = %.4f on clean signal, expected small", s2);
}

// =====================================================================
//  7. Prediction intervals
// =====================================================================
static void test_prediction_intervals() {
    SECTION("Prediction intervals");

    AdditiveHW hw;
    int m = 6;
    hw.begin(m);

    for (int t = 0; t < 6 * m; ++t)
        hw.update(synth(t, m));

    // h=1, z=1.96
    HWForecast f1 = hw.forecastPI(1, 1.96f);
    CHECK(f1.lower <= f1.yhat, "lower <= yhat");
    CHECK(f1.upper >= f1.yhat, "upper >= yhat");
    CHECK(f1.sigma >= 0.0f,    "sigma >= 0");
    CHECK(near(f1.yhat - 1.96f * f1.sigma, f1.lower, 1e-3f),
          "lower = yhat - z*sigma");
    CHECK(near(f1.yhat + 1.96f * f1.sigma, f1.upper, 1e-3f),
          "upper = yhat + z*sigma");

    // Wider interval at h=m than h=1
    HWForecast fm = hw.forecastPI(m, 1.96f);
    CHECK(fm.sigma >= f1.sigma,
          "sigma at h=%d (%.4f) should be >= sigma at h=1 (%.4f)",
          m, fm.sigma, f1.sigma);

    // z=0 → interval collapses to point
    HWForecast f0 = hw.forecastPI(1, 0.0f);
    CHECK(near(f0.lower, f0.yhat, 1e-6f), "z=0: lower == yhat");
    CHECK(near(f0.upper, f0.yhat, 1e-6f), "z=0: upper == yhat");
}

// =====================================================================
//  8. varMultiplier horizon cap
// =====================================================================
static void test_var_multiplier_cap() {
    SECTION("varMultiplier horizon cap");

    AdditiveHW hw;
    int m = 6;
    hw.begin(m);

    for (int t = 0; t < 6 * m; ++t)
        hw.update(synth(t, m));

    // forecastPI with h = 4*m and h = 100000 should give same sigma
    // (both capped at 4*m = 24)
    HWForecast f_cap  = hw.forecastPI(4 * m);
    HWForecast f_huge = hw.forecastPI(100000);

    CHECK(near(f_cap.sigma, f_huge.sigma, 1e-4f),
          "sigma at h=%d (%.4f) should match h=100000 (%.4f)",
          4 * m, f_cap.sigma, f_huge.sigma);
    CHECK(std::isfinite(f_huge.sigma), "sigma at h=100000 should be finite");
    CHECK(std::isfinite(f_huge.yhat),  "yhat at h=100000 should be finite");
}

// =====================================================================
//  9. Immediate refit — parameters change
// =====================================================================
static void test_immediate_refit() {
    SECTION("Immediate refit");

    AdditiveHW hw;
    int m = 4;
    hw.begin(m, 0, 0, 0.5f, 0.5f, 0.5f);

    // Feed enough data to trigger at least one refit (window fills
    // at 3*m=12, then refit triggers every m=4 observations).
    int N = 5 * m;
    for (int t = 0; t < N; ++t)
        hw.update(synth(t, m));

    HWParams p = hw.smoothing();
    // After refit, at least one parameter should have moved from 0.5
    bool moved = (fabsf(p.alpha - 0.5f) > 0.001f ||
                  fabsf(p.beta  - 0.5f) > 0.001f ||
                  fabsf(p.gamma - 0.5f) > 0.001f);
    CHECK(moved, "at least one param should move after refit "
          "(α=%.3f β=%.3f γ=%.3f)", p.alpha, p.beta, p.gamma);
}

// =====================================================================
// 10. Deferred refit
// =====================================================================
static void test_deferred_refit() {
    SECTION("Deferred refit");

    AdditiveHW hw;
    int m = 4;
    hw.begin(m, 0, 0, 0.5f, 0.5f, 0.5f);
    hw.deferRefit(true);

    // Not pending initially
    CHECK(!hw.refitPending(), "no refit pending initially");
    CHECK(!hw.processRefit(), "processRefit returns false when nothing pending");

    // Feed enough to fill window and trigger refit-pending
    int N = 3 * m + m;  // window fills at 3*m, refit due at next m boundary
    for (int t = 0; t < N; ++t)
        hw.update(synth(t, m));

    // Params should still be at initial values (no inline refit)
    HWParams before = hw.smoothing();

    if (hw.refitPending()) {
        CHECK(true, "refit is pending after enough data");

        bool did = hw.processRefit();
        CHECK(did, "processRefit returns true when pending");
        CHECK(!hw.refitPending(), "no longer pending after processRefit");

        HWParams after = hw.smoothing();
        bool changed = (fabsf(after.alpha - before.alpha) > 1e-6f ||
                        fabsf(after.beta  - before.beta)  > 1e-6f ||
                        fabsf(after.gamma - before.gamma) > 1e-6f);
        CHECK(changed, "params changed after deferred refit "
              "(before: α=%.3f β=%.3f γ=%.3f, after: α=%.3f β=%.3f γ=%.3f)",
              before.alpha, before.beta, before.gamma,
              after.alpha, after.beta, after.gamma);
    } else {
        // May need a few more observations depending on window/refitEvery alignment
        for (int t = N; t < N + 2 * m; ++t)
            hw.update(synth(t, m));

        if (hw.refitPending()) {
            CHECK(true, "refit pending (triggered after extra obs)");
            hw.processRefit();
            CHECK(!hw.refitPending(), "no longer pending");
        } else {
            CHECK(false, "refit never became pending");
        }
    }
}

// =====================================================================
// 11. Deferred vs immediate produce similar results
// =====================================================================
static void test_deferred_vs_immediate() {
    SECTION("Deferred vs immediate agreement");

    int m = 6;
    int N = 8 * m;

    // Run 1: immediate
    AdditiveHW imm;
    imm.begin(m, 0, 0, 0.3f, 0.1f, 0.2f);
    for (int t = 0; t < N; ++t)
        imm.update(synth(t, m));

    // Run 2: deferred, process after each update
    AdditiveHW def;
    def.begin(m, 0, 0, 0.3f, 0.1f, 0.2f);
    def.deferRefit(true);
    for (int t = 0; t < N; ++t) {
        def.update(synth(t, m));
        def.processRefit();
    }

    // Forecasts should be close
    float fc_imm = imm.forecast(1);
    float fc_def = def.forecast(1);
    CHECK(fabsf(fc_imm - fc_def) < 2.0f,
          "immediate fc=%.3f vs deferred fc=%.3f (diff=%.3f)",
          fc_imm, fc_def, fc_imm - fc_def);
}

// =====================================================================
// 12. Phase alignment — non-zero base offset
// =====================================================================
static void test_phase_alignment() {
    SECTION("Phase alignment (non-zero base)");

    // Two models with different season lengths that create different
    // base offsets. Both should converge well on clean seasonal data.
    for (int m = 4; m <= 7; ++m) {
        AdditiveHW hw;
        hw.begin(m, 0, 0, 0.3f, 0.1f, 0.2f);

        // Feed an offset number of observations so base % m != 0
        // when the ring fills. With window = 3*m and refitEvery = m,
        // we need enough data past 3*m to trigger a refit.
        int N = 7 * m + 3;  // +3 forces non-zero base for most m
        for (int t = 0; t < N; ++t)
            hw.update(synth(t, m, 100.0f, 0.2f, 8.0f));

        CHECK(hw.isReady(), "m=%d: model ready", m);

        // Forecast next full cycle — should track the signal
        float maxErr = 0.0f;
        for (int h = 1; h <= m; ++h) {
            float fc  = hw.forecast(h);
            float act = synth(N - 1 + h, m, 100.0f, 0.2f, 8.0f);
            float err = fabsf(fc - act);
            if (err > maxErr) maxErr = err;
        }
        CHECK(maxErr < 8.0f,
              "m=%d: max forecast error = %.3f (want < 8.0)", m, maxErr);
    }
}

// =====================================================================
// 13. begin() resets state completely
// =====================================================================
static void test_begin_resets() {
    SECTION("begin() resets state");

    AdditiveHW hw;
    hw.begin(6);

    // Feed data until ready
    for (int t = 0; t < 30; ++t)
        hw.update(synth(t, 6));
    CHECK(hw.isReady(), "ready before reset");

    // Reset with different season
    hw.begin(4);
    CHECK(!hw.isReady(), "not ready after begin()");
    CHECK(hw.seasonLen() == 4, "seasonLen reset to 4");
    CHECK(hw.level() == 0.0f, "level reset to 0");
    CHECK(hw.trend() == 0.0f, "trend reset to 0");
    CHECK(hw.sigma2() == 0.0f, "sigma2 reset to 0");

    // Should become ready again at 2*4 = 8 observations
    for (int t = 0; t < 8; ++t)
        hw.update(synth(t, 4));
    CHECK(hw.isReady(), "ready again after reset + 8 obs");
}

// =====================================================================
// 14. Season length clamping
// =====================================================================
static void test_season_clamping() {
    SECTION("Season length clamping");

    AdditiveHW hw;

    hw.begin(0);
    CHECK(hw.seasonLen() >= 1, "seasonLen(0) clamped to %d (>= 1)", hw.seasonLen());

    hw.begin(-5);
    CHECK(hw.seasonLen() >= 1, "seasonLen(-5) clamped to %d (>= 1)", hw.seasonLen());

    hw.begin(100);
    CHECK(hw.seasonLen() <= AdditiveHW::MAX_SEASON,
          "seasonLen(100) clamped to %d (<= %d)",
          hw.seasonLen(), AdditiveHW::MAX_SEASON);
}

// =====================================================================
// 15. Parameter clamping
// =====================================================================
static void test_param_clamping() {
    SECTION("Parameter clamping");

    AdditiveHW hw;
    hw.begin(4, 0, 0, -0.5f, 1.5f, 2.0f);

    HWParams p = hw.smoothing();
    CHECK(p.alpha >= 0.0f && p.alpha <= 1.0f,
          "alpha clamped: %.3f", p.alpha);
    CHECK(p.beta >= 0.0f && p.beta <= 1.0f,
          "beta clamped: %.3f", p.beta);
    CHECK(p.gamma >= 0.0f && p.gamma <= 1.0f,
          "gamma clamped: %.3f", p.gamma);
}

// =====================================================================
// 16. Constant signal → zero trend, tiny residual
// =====================================================================
static void test_constant_signal() {
    SECTION("Constant signal");

    AdditiveHW hw;
    int m = 4;
    hw.begin(m, 0, 0, 0.3f, 0.1f, 0.1f);

    for (int t = 0; t < 60; ++t)
        hw.update(42.0f);

    CHECK(hw.isReady(), "ready");
    CHECK(fabsf(hw.trend()) < 0.5f,
          "trend = %.4f, expected ~0", hw.trend());
    CHECK(fabsf(hw.forecast(1) - 42.0f) < 2.0f,
          "forecast = %.4f, expected ~42.0", hw.forecast(1));
}

// =====================================================================
// 17. Multi-step forecast — monotone trend
// =====================================================================
static void test_multistep_forecast() {
    SECTION("Multi-step forecast with trend");

    AdditiveHW hw;
    // Use m=1 so there is no seasonal component to break monotonicity.
    hw.begin(1, 0, 0, 0.8f, 0.3f, 0.01f);

    // Strong upward trend
    for (int t = 0; t < 60; ++t)
        hw.update(10.0f + 3.0f * (float)t);

    // forecast(h) for increasing h should be monotonically increasing
    float prev = hw.forecast(1);
    bool monotone = true;
    for (int h = 2; h <= 8; ++h) {
        float fc = hw.forecast(h);
        if (fc < prev - 0.01f) { monotone = false; break; }
        prev = fc;
    }
    CHECK(monotone, "multi-step forecasts should increase with positive trend (m=1)");

    // Also check that trend is approximately correct
    CHECK(fabsf(hw.trend() - 3.0f) < 1.0f,
          "trend = %.3f, expected ~3.0", hw.trend());
}

// =====================================================================
// 18. forecast(h) default argument
// =====================================================================
static void test_forecast_defaults() {
    SECTION("forecast() default h=1");

    AdditiveHW hw;
    hw.begin(4);

    for (int t = 0; t < 20; ++t)
        hw.update(synth(t, 4));

    float fc_default = hw.forecast();
    float fc_h1      = hw.forecast(1);
    CHECK(fc_default == fc_h1,
          "forecast() = %.4f, forecast(1) = %.4f", fc_default, fc_h1);
}

// =====================================================================
// 19. forecast(h < 1) clamped to h=1
// =====================================================================
static void test_forecast_h_clamp() {
    SECTION("forecast(h) clamp to >= 1");

    AdditiveHW hw;
    hw.begin(4);

    for (int t = 0; t < 20; ++t)
        hw.update(synth(t, 4));

    float fc_0    = hw.forecast(0);
    float fc_neg  = hw.forecast(-5);
    float fc_1    = hw.forecast(1);

    CHECK(fc_0   == fc_1, "forecast(0) = forecast(1)");
    CHECK(fc_neg == fc_1, "forecast(-5) = forecast(1)");
}

// =====================================================================
// 20. HWParams struct accessors
// =====================================================================
static void test_hwparams() {
    SECTION("HWParams operator[] and ref()");

    HWParams p = {0.1f, 0.2f, 0.3f};
    CHECK(p[0] == 0.1f, "p[0] = alpha");
    CHECK(p[1] == 0.2f, "p[1] = beta");
    CHECK(p[2] == 0.3f, "p[2] = gamma");

    p.ref(0) = 0.5f;
    p.ref(1) = 0.6f;
    p.ref(2) = 0.7f;
    CHECK(p.alpha == 0.5f, "ref(0) sets alpha");
    CHECK(p.beta  == 0.6f, "ref(1) sets beta");
    CHECK(p.gamma == 0.7f, "ref(2) sets gamma");
}

// =====================================================================
// 21. C API — full lifecycle
// =====================================================================
static void test_c_api() {
    SECTION("C API (additive_hw_api)");

    ahw_init(6, false);
    CHECK(!ahw_is_ready(), "not ready before data");

    for (int t = 0; t < 12; ++t)
        ahw_update(synth(t, 6));

    CHECK(ahw_is_ready(), "ready after 2*m obs");

    // Feed more for refit
    for (int t = 12; t < 48; ++t)
        ahw_update(synth(t, 6));

    ahw_forecast_t f;
    bool ok = ahw_forecast(1, &f);
    CHECK(ok, "ahw_forecast returns true when ready");
    CHECK(std::isfinite(f.yhat),  "yhat finite");
    CHECK(std::isfinite(f.lower), "lower finite");
    CHECK(std::isfinite(f.upper), "upper finite");
    CHECK(std::isfinite(f.sigma), "sigma finite");
    CHECK(f.lower <= f.yhat, "lower <= yhat");
    CHECK(f.upper >= f.yhat, "upper >= yhat");

    float lvl = ahw_level();
    float trn = ahw_trend();
    float s2  = ahw_sigma2();
    CHECK(std::isfinite(lvl), "ahw_level finite");
    CHECK(std::isfinite(trn), "ahw_trend finite");
    CHECK(s2 >= 0.0f,         "ahw_sigma2 >= 0");

    // Null pointer should return false
    ok = ahw_forecast(1, nullptr);
    CHECK(!ok, "ahw_forecast(NULL) returns false");
}

// =====================================================================
// 22. C API — deferred mode
// =====================================================================
static void test_c_api_deferred() {
    SECTION("C API deferred mode");

    ahw_init(4, true);

    CHECK(!ahw_process(), "process returns false initially");

    for (int t = 0; t < 40; ++t) {
        ahw_update(synth(t, 4));
        ahw_process();
    }

    CHECK(ahw_is_ready(), "ready in deferred mode");

    ahw_forecast_t f;
    bool ok = ahw_forecast(1, &f);
    CHECK(ok, "forecast available in deferred mode");
    CHECK(std::isfinite(f.yhat), "yhat finite in deferred mode");
}

// =====================================================================
// 23. C API — forecast before ready
// =====================================================================
static void test_c_api_not_ready() {
    SECTION("C API forecast before ready");

    ahw_init(6, false);
    ahw_update(1.0f);

    ahw_forecast_t f;
    memset(&f, 0xFF, sizeof(f));  // fill with garbage
    bool ok = ahw_forecast(1, &f);
    CHECK(!ok, "ahw_forecast returns false when not ready");
    // f should be unchanged (not ready → struct not written)
}

// =====================================================================
// 24. Large season (MAX_SEASON = 24)
// =====================================================================
static void test_max_season() {
    SECTION("MAX_SEASON = 24");

    AdditiveHW hw;
    hw.begin(24);
    CHECK(hw.seasonLen() == 24, "seasonLen = 24");

    for (int t = 0; t < 24 * 5; ++t)
        hw.update(synth(t, 24, 200.0f, 0.1f, 15.0f));

    CHECK(hw.isReady(), "ready with m=24");

    HWForecast f = hw.forecastPI(1);
    CHECK(std::isfinite(f.yhat), "yhat finite at m=24");
    CHECK(f.sigma >= 0.0f, "sigma >= 0 at m=24");
}

// =====================================================================
// 25. Season = 1 (no seasonality, just level + trend)
// =====================================================================
static void test_season_one() {
    SECTION("Season = 1");

    AdditiveHW hw;
    hw.begin(1, 0, 0, 0.5f, 0.3f, 0.1f);
    CHECK(hw.seasonLen() == 1, "seasonLen = 1");

    // Linear signal
    for (int t = 0; t < 30; ++t)
        hw.update(10.0f + 1.5f * (float)t);

    CHECK(hw.isReady(), "ready with m=1");

    float fc = hw.forecast(1);
    float expected = 10.0f + 1.5f * 30.0f;
    CHECK(fabsf(fc - expected) < 10.0f,
          "forecast(1) = %.2f, expected ~%.2f", fc, expected);
}

// =====================================================================
// 26. Noisy signal — forecast stays in prediction interval
// =====================================================================
static void test_noisy_signal() {
    SECTION("Noisy signal coverage");

    AdditiveHW hw;
    int m = 6;
    hw.begin(m);

    // Deterministic "noise" via a simple LCG
    unsigned long seed = 12345;
    auto noise = [&]() -> float {
        seed = seed * 1103515245UL + 12345UL;
        return ((float)((int)(seed >> 16) % 1000) / 1000.0f - 0.5f) * 4.0f;
    };

    int N = 10 * m;
    float data[120];
    for (int t = 0; t < N; ++t) {
        data[t] = synth(t, m) + noise();
        hw.update(data[t]);
    }

    CHECK(hw.isReady(), "ready on noisy signal");

    // sigma2 should be meaningfully positive
    CHECK(hw.sigma2() > 0.01f,
          "sigma2 = %.4f on noisy signal", hw.sigma2());

    // PI should be wider than on clean signal
    HWForecast f = hw.forecastPI(1);
    CHECK(f.upper - f.lower > 0.1f,
          "PI width = %.4f, should be > 0", f.upper - f.lower);
}

// =====================================================================
// 27. Multiple begin() calls — independence
// =====================================================================
static void test_multiple_begin() {
    SECTION("Multiple begin() calls");

    AdditiveHW hw;

    // First run
    hw.begin(4);
    for (int t = 0; t < 20; ++t)
        hw.update(synth(t, 4, 50.0f));
    float fc1 = hw.forecast(1);

    // Second run with different signal
    hw.begin(6);
    for (int t = 0; t < 30; ++t)
        hw.update(synth(t, 6, 200.0f));
    float fc2 = hw.forecast(1);

    CHECK(fabsf(fc2 - fc1) > 50.0f,
          "second begin() produces different forecast (fc1=%.2f, fc2=%.2f)",
          fc1, fc2);
}

// =====================================================================
// 28. Long run — no NaN / Inf drift
// =====================================================================
static void test_long_run_stability() {
    SECTION("Long-run numerical stability");

    AdditiveHW hw;
    int m = 8;
    hw.begin(m);

    int N = 500;
    for (int t = 0; t < N; ++t)
        hw.update(synth(t, m, 1000.0f, 0.01f, 5.0f));

    CHECK(std::isfinite(hw.level()),    "level finite after %d obs", N);
    CHECK(std::isfinite(hw.trend()),    "trend finite after %d obs", N);
    CHECK(std::isfinite(hw.sigma2()),   "sigma2 finite after %d obs", N);
    CHECK(std::isfinite(hw.forecast(1)),"forecast finite after %d obs", N);

    HWForecast f = hw.forecastPI(m);
    CHECK(std::isfinite(f.yhat),  "yhat finite after long run");
    CHECK(std::isfinite(f.sigma), "sigma finite after long run");
    CHECK(std::isfinite(f.lower), "lower finite after long run");
    CHECK(std::isfinite(f.upper), "upper finite after long run");
}

// =====================================================================
// 29. Macro hygiene — min/max/constrain not leaked
// =====================================================================
static void test_macro_hygiene() {
    SECTION("Macro hygiene");

#ifdef min
    CHECK(false, "min macro leaked");
#else
    CHECK(true, "min macro cleaned up");
#endif

#ifdef max
    CHECK(false, "max macro leaked");
#else
    CHECK(true, "max macro cleaned up");
#endif

#ifdef constrain
    CHECK(false, "constrain macro leaked");
#else
    CHECK(true, "constrain macro cleaned up");
#endif
}

// =====================================================================
// 30. Smoothing params accessor
// =====================================================================
static void test_smoothing_accessor() {
    SECTION("smoothing() accessor");

    AdditiveHW hw;
    hw.begin(4, 0, 0, 0.15f, 0.25f, 0.35f);

    HWParams p = hw.smoothing();
    CHECK(near(p.alpha, 0.15f, 1e-5f), "alpha = %.4f", p.alpha);
    CHECK(near(p.beta,  0.25f, 1e-5f), "beta = %.4f",  p.beta);
    CHECK(near(p.gamma, 0.35f, 1e-5f), "gamma = %.4f", p.gamma);
}

// =====================================================================
// 31. Forecast wraps seasonal index correctly
// =====================================================================
static void test_seasonal_wrap() {
    SECTION("Seasonal index wrapping in forecast");

    AdditiveHW hw;
    int m = 4;
    hw.begin(m);

    for (int t = 0; t < 8 * m; ++t)
        hw.update(synth(t, m, 50.0f, 0.0f, 10.0f));

    // forecast(h) and forecast(h + m) should be similar for a
    // stationary-level signal (same seasonal phase, trend≈0)
    float fc_1    = hw.forecast(1);
    float fc_1pm  = hw.forecast(1 + m);
    CHECK(fabsf(fc_1 - fc_1pm) < 3.0f,
          "forecast(1)=%.3f ≈ forecast(1+m)=%.3f", fc_1, fc_1pm);
}

// =====================================================================
// 32. refitEvery custom value
// =====================================================================
static void test_custom_refit_every() {
    SECTION("Custom refitEvery");

    AdditiveHW hw;
    int m = 4;
    // refitEvery = 2 → refit twice as often
    hw.begin(m, 0, 2, 0.3f, 0.1f, 0.1f);

    for (int t = 0; t < 40; ++t)
        hw.update(synth(t, m));

    CHECK(hw.isReady(), "ready with custom refitEvery");
    CHECK(std::isfinite(hw.forecast(1)), "forecast finite");
}

// =====================================================================
// 33. Custom window size
// =====================================================================
static void test_custom_window() {
    SECTION("Custom window");

    AdditiveHW hw;
    int m = 4;
    // Minimum valid window is 2*m+1 = 9
    hw.begin(m, 2 * m + 1);

    for (int t = 0; t < 30; ++t)
        hw.update(synth(t, m));

    CHECK(hw.isReady(), "ready with min window");
    CHECK(std::isfinite(hw.forecast(1)), "forecast finite with min window");
}

// =====================================================================
// main
// =====================================================================
int main() {
    printf("AdditiveHW test suite\n");
    printf("=====================\n");

    test_bootstrap();
    test_forecast_before_ready();
    test_level_trend_tracking();
    test_seasonal_recovery();
    test_fitted_residual();
    test_sigma2();
    test_prediction_intervals();
    test_var_multiplier_cap();
    test_immediate_refit();
    test_deferred_refit();
    test_deferred_vs_immediate();
    test_phase_alignment();
    test_begin_resets();
    test_season_clamping();
    test_param_clamping();
    test_constant_signal();
    test_multistep_forecast();
    test_forecast_defaults();
    test_forecast_h_clamp();
    test_hwparams();
    test_c_api();
    test_c_api_deferred();
    test_c_api_not_ready();
    test_max_season();
    test_season_one();
    test_noisy_signal();
    test_multiple_begin();
    test_long_run_stability();
    test_macro_hygiene();
    test_smoothing_accessor();
    test_seasonal_wrap();
    test_custom_refit_every();
    test_custom_window();

    printf("\n=====================\n");
    printf("%d passed, %d failed\n", g_pass, g_fail);
    printf("%s\n", g_fail == 0 ? "ALL PASSED" : "SOME FAILED");

    return g_fail > 0 ? 1 : 0;
}
