#ifndef ADDITIVE_HW_H
#define ADDITIVE_HW_H

//  Additive Holt-Winters (ETS A,A,A) triple exponential smoothing
//  with periodic coordinate-descent parameter refit and prediction intervals.
//
//  Equations:
//
//    l_t  = α·(y_t − s_{t−m})  + (1−α)·(l_{t−1} + b_{t−1})     level
//    b_t  = β·(l_t − l_{t−1})  + (1−β)·b_{t−1}                  trend
//    s_t  = γ·(y_t − l_t)      + (1−γ)·s_{t−m}                  seasonal
//    ŷ_{t+h} = l_t + h·b_t + s_{t+h−m}                          forecast
//
//  Prediction intervals  (Hyndman et al. 2008, §6.4):
//
//    σ²_h = σ² · Σ_{j=0}^{h−1} c_j²
//    c_0  = 1,   c_j = α + α·β·j + γ·𝟙(j mod m = 0)   j ≥ 1
//    PI   = ŷ ± z · σ_h
//
//  σ² is the variance of genuine one-step-ahead residuals, estimated
//  online via Welford's algorithm.  Residuals are captured in update()
//  before each step (online or refit); rebuild replays do not contribute.
//
//  Deferred refit:
//    By default update() runs the optimizer inline (immediate mode).
//    Call deferRefit(true) to switch to deferred mode: update() only
//    sets a flag, and the heavy work happens in processRefit(), which
//    you call from loop() when you have time.  Forecasts remain valid
//    (with old parameters) while a refit is pending.
//
//  RAM budget  (MAX_SEASON = 24, MAX_WINDOW = 128):
//    ring[]            512 B   scratch[]         512 B
//    s[] state          96 B   seasonScratch[]    96 B
//    varTable[]        384 B   scalars           ~76 B
//    Total           ~1676 B   (Arduino Uno: 2 KB SRAM, ~82% used)
//
//  MAX_WINDOW is a power of two so ring wraps are a single AND
//  (`& windowMask_`) instead of a software-divide on AVR.  The
//  effective window is rounded UP to the next power of two in begin().
//  varTable[] precomputes the cumulative-sum variance multiplier
//  so forecastPI(h) is an O(1) lookup.  seasonScratch[] is the
//  lifted MSE seasonal buffer — keeps coord descent off the stack
//  across ~240 calls per refit.

// ── Platform ─────────────────────────────────────────────────────────
#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <string.h>
  #include <stdint.h>
#endif
#include <math.h>
#include "ahw_trace.h"

#ifndef constrain
  #define constrain(x, lo, hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
  #define AHW_UNDEF_CONSTRAIN_
#endif
#ifndef min
  #define min(a, b) ((a)<(b)?(a):(b))
  #define AHW_UNDEF_MIN_
#endif
#ifndef max
  #define max(a, b) ((a)>(b)?(a):(b))
  #define AHW_UNDEF_MAX_
#endif

// ── Smoothing parameters ─────────────────────────────────────────────

struct HWParams {
    float alpha;   // level     [0, 1]
    float beta;    // trend     [0, 1]
    float gamma;   // seasonal  [0, 1]

    // Index: 0 → alpha, 1 → beta, 2 → gamma.
    // Out-of-range indices silently map to gamma (internal use only).
    float  operator[](int i) const { return i == 0 ? alpha : i == 1 ? beta : gamma; }
    float& ref(int i)               { return i == 0 ? alpha : i == 1 ? beta : gamma; }
};

// ── Forecast result ──────────────────────────────────────────────────

struct HWForecast {
    float yhat;    // point forecast  ŷ_{t+h}
    float lower;   // lower PI bound
    float upper;   // upper PI bound
    float sigma;   // forecast std dev  σ_h
};

// ── Compile-time helper ──────────────────────────────────────────────
//   next_pow2(n) rounds n up to the nearest power of two so MAX_WINDOW
//   can be derived from MAX_SEASON instead of hand-tuned.
namespace ahw_detail_ {
    constexpr int next_pow2(int n, int p = 1) {
        return p >= n ? p : next_pow2(n, p << 1);
    }
}

// ── Main class ───────────────────────────────────────────────────────

class AdditiveHW {
public:
    static const int MAX_SEASON = 24;
    static const int MAX_WINDOW = ahw_detail_::next_pow2(3 * MAX_SEASON);
    static_assert((MAX_WINDOW & (MAX_WINDOW - 1)) == 0,
                  "MAX_WINDOW must be a power of two for bitmask wrap");
    static_assert(MAX_WINDOW >= 2 * MAX_SEASON + 1,
                  "MAX_WINDOW must be ≥ 2·MAX_SEASON+1 for bootstrap");

    AdditiveHW() { begin(12); }

    /// (Re)configure the model.
    ///   seasonLen  – seasonal period       (1 … MAX_SEASON)
    ///   window     – ring size for refit;  0 → 3 × seasonLen.
    ///                Rounded UP to the next power of two so wraps are a
    ///                single AND.  Effective window may exceed the request.
    ///   refitEvery – refit period in obs;  0 → seasonLen
    ///
    /// Returns true if all arguments were accepted as given, false if any
    /// were clamped to a valid range (seasonLen outside 1…MAX_SEASON,
    /// window outside 2·m+1…MAX_WINDOW, refitEvery negative, or α/β/γ
    /// outside [0, 1]).  Power-of-two rounding of `window` is by design
    /// and does NOT count as a clamp.
    bool begin(int   seasonLen,
               int   window     = 0,
               int   refitEvery = 0,
               float alpha      = 0.20f,
               float beta       = 0.05f,
               float gamma      = 0.10f)
    {
        bool ok = true;

        if (seasonLen < 1 || seasonLen > MAX_SEASON) ok = false;
        m_          = constrain(seasonLen, 1, MAX_SEASON);

        int reqW = window <= 0 ? 3 * m_ : window;
        if (window > 0 && (window < 2 * m_ + 1 || window > MAX_WINDOW)) ok = false;
        int w = constrain(reqW, 2 * m_ + 1, MAX_WINDOW);
        // Round up to the next power of two (≤ MAX_WINDOW, also a pow2).
        int p = 1;
        while (p < w) p <<= 1;
        if (p > MAX_WINDOW) p = MAX_WINDOW;
        window_     = p;
        windowMask_ = window_ - 1;

        if (refitEvery < 0) ok = false;
        refitEvery_ = refitEvery <= 0 ? m_ : max(1, refitEvery);

        if (alpha < 0.0f || alpha > 1.0f) ok = false;
        if (beta  < 0.0f || beta  > 1.0f) ok = false;
        if (gamma < 0.0f || gamma > 1.0f) ok = false;
        alpha_      = clamp01(alpha);
        beta_       = clamp01(beta);
        gamma_      = clamp01(gamma);

        deferred_   = false;
        reset();
        rebuildVarTable();
        return ok;
    }

    /// Enable or disable deferred refit mode.
    ///   true  → update() is always fast; call processRefit() from loop().
    ///   false → update() runs the optimizer inline (default).
    void deferRefit(bool on) { deferred_ = on; }

    /// Feed one new observation.
    ///   Always O(1) in deferred mode.  In immediate mode, may run the
    ///   optimizer (~1 ms on nRF52840, ~200 ms on AVR Uno).
    void update(float y) {
        ringPush(y);
        nObs_++;

        if (ready_ && ringN_ == window_ && (nObs_ % refitEvery_) == 0) {
            onlineStep(y);              // advance state with current params

            if (deferred_) {
                refitPending_ = true;
            } else {
                doRefit();
            }
        } else {
            onlineStep(y);
        }
    }

    /// Run a pending refit.  Call from loop() when in deferred mode.
    ///   Returns true if work was done, false if nothing was pending.
    bool processRefit() {
        if (!refitPending_) return false;
        doRefit();
        return true;
    }

    /// True if a refit is waiting to be processed.
    bool refitPending() const { return refitPending_; }

    /// Point forecast, h steps ahead  (h ≥ 1).
    /// Note: uses the *configured* seasonLen() — if begin() was called
    /// with seasonLen > MAX_SEASON, that argument was silently clamped.
    /// Check begin()'s return or seasonLen() to detect this.
    float forecast(int h = 1) const {
        h = max(h, 1);
        if (!ready_) return ringN_ > 0 ? ringMean() : 0.0f;
        return l_ + (float)h * b_ + s_[sIdx(nObs_ + h - 1)];
    }

    /// Forecast with prediction interval.
    ///   z = 1.96 → 95 %,  1.28 → 80 %,  2.576 → 99 %
    ///   Note: interval width is capped at h = 4·m.  For larger h the
    ///   point forecast keeps moving but sigma stays at the 4·m value.
    HWForecast forecastPI(int h = 1, float z = 1.96f) const {
        HWForecast f;
        f.yhat = forecast(h);

        if (!ready_ || nResid_ < 2) {
            f.sigma = 0.0f;
            f.lower = f.upper = f.yhat;
            return f;
        }

        float s2h = sigma2() * varMultiplier(h);
        f.sigma   = s2h > 0.0f ? sqrtf(s2h) : 0.0f;
        f.lower   = f.yhat - z * f.sigma;
        f.upper   = f.yhat + z * f.sigma;
        return f;
    }

    // ── Accessors ────────────────────────────────────────────────────
    bool     isReady()    const { return ready_; }
    float    fitted()     const { return fitted_; }    // last online one-step-ahead prediction (ŷ_t)
    float    residual()   const { return residual_; }  // last online one-step-ahead error (y_t − ŷ_t)
    float    level()      const { return l_; }
    float    trend()      const { return b_; }
    int      seasonLen()  const { return m_; }
    HWParams smoothing()  const { return { alpha_, beta_, gamma_ }; }

    /// One-step residual variance  σ² (unbiased, n−1 denominator).
    /// Cached after every welfordPush; getter is a single load.
    float sigma2() const { return sigma2Cached_; }

private:
    // ── Config ───────────────────────────────────────────────────────
    int   m_;
    int   window_;
    int   windowMask_;          // window_ - 1; valid because window_ is a power of two
    int   refitEvery_;
    float alpha_, beta_, gamma_;
    bool  deferred_;

    // ── Ring buffer ──────────────────────────────────────────────────
    float ring_[MAX_WINDOW];
    int   ringHead_;
    int   ringN_;

    void ringPush(float y) {
        if (ringN_ < window_) {
            ring_[(ringHead_ + ringN_++) & windowMask_] = y;
            ringSum_ += y;
        } else {
            ringSum_ -= ring_[ringHead_];
            ring_[ringHead_] = y;
            ringSum_ += y;
            ringHead_ = (ringHead_ + 1) & windowMask_;
        }
    }

    void ringCopyTo(float* dst) const {
        for (int i = 0; i < ringN_; ++i)
            dst[i] = ring_[(ringHead_ + i) & windowMask_];
    }

    float ringAt(int i) const {
        return ring_[(ringHead_ + i) & windowMask_];
    }

    float ringMean() const {
        return ringN_ > 0 ? ringSum_ / (float)ringN_ : 0.0f;
    }

    // ── Model state ──────────────────────────────────────────────────
    bool           ready_;
    bool           refitPending_;
    unsigned long  nObs_;
    float          l_;                      // level
    float          b_;                      // trend
    float          s_[MAX_SEASON];          // seasonal factors  (Σ = 0)
    float          fitted_;                 // ŷ_t
    float          residual_;               // e_t = y_t − ŷ_t

    // Welford online variance — only genuine one-step-ahead residuals.
    // Never reset by rebuildState; only reset by begin().
    unsigned long  nResid_;
    float          resMean_;
    float          resM2_;

    mutable float  scratch_[MAX_WINDOW];        // refit work buffer
    mutable float  seasonScratch_[MAX_SEASON];  // computeMSE seasonal scratch
    float          ringSum_;                    // running Σ ring_[] for O(1) ringMean
    float          sigma2Cached_;               // cached σ², refreshed in welfordPush
    float          varTable_[4 * MAX_SEASON];   // Σ_{j=0}^{h−1} c_j² lookup, rebuilt on refit

    int sIdx(unsigned long t) const {
        return (int)(t % (unsigned long)m_);
    }

    void reset() {
        ringHead_ = ringN_ = 0;
        nObs_     = 0;
        nResid_   = 0;
        resMean_  = resM2_ = 0.0f;
        ringSum_  = 0.0f;
        sigma2Cached_ = 0.0f;
        ready_    = false;
        refitPending_ = false;
        l_ = b_ = fitted_ = residual_ = 0.0f;
        memset(s_,             0, sizeof(s_));
        memset(ring_,          0, sizeof(ring_));
        memset(scratch_,       0, sizeof(scratch_));
        memset(seasonScratch_, 0, sizeof(seasonScratch_));
        memset(varTable_,      0, sizeof(varTable_));
    }

    // ── Refit (optimize + rebuild) ───────────────────────────────────
    //  Rebuild overwrites fitted_/residual_ with in-sample replay values.
    //  We preserve the genuine online one-step prediction from onlineStep().
    void doRefit() {
        AHW_TRACE(AHW_SLOT_REFIT);
        float saveFit = fitted_, saveRes = residual_;

        HWParams p = smoothing();
        optimizeParams(p);
        alpha_ = p.alpha;  beta_ = p.beta;  gamma_ = p.gamma;
        rebuildVarTable();
        rebuildState();

        fitted_ = saveFit;  residual_ = saveRes;
        refitPending_ = false;
    }

    // ── Core ETS recursion ───────────────────────────────────────────
    //  Updates l, b, s[si] and sets fitted_/residual_.
    //  Does NOT touch the Welford accumulator — caller decides.
    void etsStep(float y, int si) {
        float sOld = s_[si];

        fitted_   = l_ + b_ + sOld;
        residual_ = y - fitted_;

        float lNew = alpha_ * (y - sOld) + (1.0f - alpha_) * (l_ + b_);
        float bNew = beta_  * (lNew - l_) + (1.0f - beta_)  * b_;

        l_     = lNew;
        b_     = bNew;
        s_[si] = gamma_ * (y - lNew) + (1.0f - gamma_) * sOld;
    }

    // ── Welford update ───────────────────────────────────────────────
    void welfordPush(float e) {
        nResid_++;
        float d1  = e - resMean_;
        resMean_ += d1 / (float)nResid_;
        float d2  = e - resMean_;
        resM2_   += d1 * d2;
        sigma2Cached_ = nResid_ > 1 ? resM2_ / (float)(nResid_ - 1) : 0.0f;
    }

    // ── h-step variance multiplier ───────────────────────────────────
    //
    //   c_0 = 1,   c_j = α + α·β·j + γ·𝟙(j mod m = 0)
    //   varTable_[h−1] = Σ_{j=0}^{h−1}  c_j²
    //   Horizon is capped at 4·m to bound computation on small MCUs.
    //   Table is rebuilt by rebuildVarTable() whenever α, β, γ, or m change.
    //
    float varMultiplier(int h) const {
        h = max(h, 1);
        int cap = 4 * m_;
        if (h > cap) h = cap;
        return varTable_[h - 1];
    }

    void rebuildVarTable() {
        int cap = 4 * m_;
        float sum = 1.0f;                          // c_0² = 1
        varTable_[0] = sum;
        for (int j = 1; j < cap; ++j) {
            float c = alpha_ + alpha_ * beta_ * (float)j;
            if (j % m_ == 0) c += gamma_;
            sum += c * c;
            varTable_[j] = sum;
        }
    }

    // ── Online step ──────────────────────────────────────────────────
    //  Genuine one-step-ahead prediction: compute forecast THEN update.
    //  Residuals feed into Welford for σ² estimation.
    void onlineStep(float y) {
        AHW_TRACE(AHW_SLOT_ONLINE);
        if (!ready_) {
            if (ringN_ >= 2 * m_)
                bootstrap(nObs_ - (unsigned long)ringN_);
            return;
        }
        etsStep(y, sIdx(nObs_ - 1));
        welfordPush(residual_);
    }

    // ── Bootstrap ────────────────────────────────────────────────────
    //  Two-cycle-average initialisation from ring[0 .. 2m−1].
    //  base = absolute time index of ring[0], used to align seasonal
    //  slots so s_[sIdx(t)] always corresponds to the same phase.
    //
    //    b_0  = (avg2 − avg1) / m
    //    l_0  = avg2 + b_0·(m−1)/2       (trend-adjusted to end of cycle 2)
    //    s_{sIdx(base+i)} = ½ [(y_i − avg1) + (y_{m+i} − avg2)]
    //
    //  The level is shifted forward from the cycle-2 midpoint to its
    //  trailing edge using the estimated trend.  This reduces the
    //  transient forecast error after each refit/rebuild.
    //
    void bootstrap(unsigned long base = 0) {
        float avg1 = 0.0f, avg2 = 0.0f;
        for (int i = 0; i < m_; ++i) {
            avg1 += ringAt(i);
            avg2 += ringAt(m_ + i);
        }
        avg1 /= (float)m_;
        avg2 /= (float)m_;

        b_ = (avg2 - avg1) / (float)m_;
        l_ = avg2 + b_ * (float)(m_ - 1) * 0.5f;

        memset(s_, 0, sizeof(s_));
        for (int i = 0; i < m_; ++i)
            s_[sIdx(base + i)] = 0.5f * ((ringAt(i) - avg1) + (ringAt(m_ + i) - avg2));

        ready_ = true;
    }

    // ── Season normalization ─────────────────────────────────────────
    //  Enforce Σ s_i = 0, absorb offset into level.
    void normalizeSeasons() {
        float mean = 0.0f;
        for (int i = 0; i < m_; ++i) mean += s_[i];
        mean /= (float)m_;
        for (int i = 0; i < m_; ++i) s_[i] -= mean;
        l_ += mean;
    }

    // ── Rebuild after refit ──────────────────────────────────────────
    //  Re-bootstrap + replay history with new parameters.
    //  Does NOT touch the Welford accumulator — it tracks only genuine
    //  out-of-sample residuals captured in update() before each refit.
    void rebuildState() {
        ready_ = false;
        l_ = b_ = fitted_ = residual_ = 0.0f;
        memset(s_, 0, sizeof(s_));

        unsigned long base = nObs_ - (unsigned long)ringN_;
        bootstrap(base);

        for (int i = 2 * m_; i < ringN_; ++i)
            etsStep(ringAt(i), sIdx(base + i));

        normalizeSeasons();
    }

    // ── MSE objective  (static, pure) ────────────────────────────────
    //  phase = base % m  so that seasonal slots match rebuildState().
    //  s    = caller-provided seasonal scratch of size ≥ m (lifted out
    //         of the stack so coordinate descent reuses one buffer).
    static float computeMSE(const float* buf, int n, int m,
                            const HWParams& p, int phase, float* s) {
        AHW_TRACE(AHW_SLOT_MSE);
        if (n < 2 * m + 1) return 1e30f;

        float avg1 = 0.0f, avg2 = 0.0f;
        for (int i = 0; i < m; ++i) { avg1 += buf[i]; avg2 += buf[m + i]; }
        avg1 /= (float)m;
        avg2 /= (float)m;

        float b = (avg2 - avg1) / (float)m;
        float l = avg2 + b * (float)(m - 1) * 0.5f;
        for (int i = 0; i < m; ++i)
            s[(phase + i) % m] = 0.5f * ((buf[i] - avg1) + (buf[m + i] - avg2));

        float sumSq = 0.0f;
        int   count = n - 2 * m;
        for (int t = 2 * m; t < n; ++t) {
            int   si   = (phase + t) % m;
            float sOld = s[si];
            float err  = buf[t] - (l + b + sOld);
            sumSq += err * err;

            float lNew = p.alpha * (buf[t] - sOld) + (1.0f - p.alpha) * (l + b);
            float bNew = p.beta  * (lNew - l)      + (1.0f - p.beta)  * b;

            l     = lNew;
            b     = bNew;
            s[si] = p.gamma * (buf[t] - lNew) + (1.0f - p.gamma) * sOld;
        }
        return count > 0 ? sumSq / (float)count : 1e30f;
    }

    // ── Coordinate-descent optimizer ─────────────────────────────────
    void optimizeParams(HWParams& p) const {
        AHW_TRACE(AHW_SLOT_OPTIMIZE);
        ringCopyTo(scratch_);

        int phase = (int)((nObs_ - (unsigned long)ringN_) % (unsigned long)m_);

        float step[3]      = { 0.05f, 0.03f, 0.05f };
        bool  converged[3] = { false, false, false };
        float bestMSE      = computeMSE(scratch_, ringN_, m_, p, phase, seasonScratch_);

        for (int iter = 0; iter < 40; ++iter) {
            bool anyActive = false;

            for (int k = 0; k < 3; ++k) {
                if (converged[k]) continue;
                anyActive = true;

                float orig    = p[k];
                float bestVal = orig;

                for (int dir = -1; dir <= 1; dir += 2) {
                    p.ref(k) = clamp01(orig + (float)dir * step[k]);
                    float loss = computeMSE(scratch_, ringN_, m_, p, phase, seasonScratch_);
                    if (loss < bestMSE - 1e-6f) {
                        bestMSE = loss;
                        bestVal = p[k];
                    }
                    p.ref(k) = orig;
                }

                if (bestVal != orig) {
                    p.ref(k) = bestVal;
                    step[k]  = min(step[k] * 1.2f, 0.3f);
                } else {
                    step[k] *= 0.5f;
                    if (step[k] < 1e-4f) converged[k] = true;
                }
            }
            if (!anyActive) break;
        }
    }

    static float clamp01(float x) { return x < 0.0f ? 0.0f : x > 1.0f ? 1.0f : x; }
};

// ── Clean up macros we may have defined ─────────────────────────────
#ifdef AHW_UNDEF_CONSTRAIN_
  #undef constrain
  #undef AHW_UNDEF_CONSTRAIN_
#endif
#ifdef AHW_UNDEF_MIN_
  #undef min
  #undef AHW_UNDEF_MIN_
#endif
#ifdef AHW_UNDEF_MAX_
  #undef max
  #undef AHW_UNDEF_MAX_
#endif

#endif // ADDITIVE_HW_H
