#ifndef AHW_TRACE_H
#define AHW_TRACE_H

// Optional cycle/microsecond accounting for AdditiveHW.
//
// Compile-time toggle:
//   * Default                        — every macro is ((void)0); zero cost.
//   * -DAHW_INSTRUMENT               — RAII timer wraps four hot paths
//                                      (onlineStep / doRefit / computeMSE /
//                                      optimizeParams) and accumulates ticks
//                                      and call-counts per slot.
//
// Time source per platform (when instrumented):
//   * AHW_TRACE_USE_DWT defined      — Cortex-M DWT->CYCCNT (CPU cycles).
//                                      Caller must enable DWT first.
//   * ARDUINO defined                — micros()  (1 µs ticks).
//   * x86_64                         — __rdtsc() (TSC cycles).
//   * Otherwise                      — std::chrono::steady_clock (ns).
//
// Storage:
//   Header-only via the static-template-member trick.  No new .cpp
//   file is required even when instrumented — the linker dedups
//   AhwTraceStorage<>::cycles across translation units.
//
// Reading / resetting:
//   uint32_t v = AHW_TRACE_GET(AHW_SLOT_REFIT);
//   uint32_t n = AHW_TRACE_COUNT(AHW_SLOT_REFIT);
//   AHW_TRACE_RESET();      // clear all slots

#include <stdint.h>

enum AhwSlot {
    AHW_SLOT_ONLINE   = 0,   // onlineStep (per update)
    AHW_SLOT_REFIT    = 1,   // doRefit    (one per refit boundary)
    AHW_SLOT_MSE      = 2,   // computeMSE (~240 calls per refit)
    AHW_SLOT_OPTIMIZE = 3,   // optimizeParams (the coord-descent loop)
    AHW_SLOT_COUNT    = 4
};

#ifdef AHW_INSTRUMENT

// ── now_ticks() — platform-specific monotonic counter ────────────────
#  if defined(AHW_TRACE_USE_DWT)
     // Cortex-M DWT.  User must have enabled CoreDebug->DEMCR DWT_TRCENA
     // and DWT->CTRL CYCCNTENA bit before any trace fires.
     inline uint32_t ahw_now_ticks() {
         return *reinterpret_cast<volatile uint32_t*>(0xE0001004);
     }
#  elif defined(ARDUINO)
#    include <Arduino.h>
     inline uint32_t ahw_now_ticks() { return (uint32_t)micros(); }
#  elif defined(__x86_64__) || defined(_M_X64)
#    include <x86intrin.h>
     inline uint32_t ahw_now_ticks() { return (uint32_t)__rdtsc(); }
#  else
#    include <chrono>
     inline uint32_t ahw_now_ticks() {
         using clk = std::chrono::steady_clock;
         auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       clk::now().time_since_epoch()).count();
         return (uint32_t)ns;
     }
#  endif

// ── Storage (header-only via template static members) ───────────────
template <typename Tag = void>
struct AhwTraceStorage {
    static volatile uint32_t cycles[AHW_SLOT_COUNT];
    static volatile uint32_t hits[AHW_SLOT_COUNT];
};
template <typename Tag>
volatile uint32_t AhwTraceStorage<Tag>::cycles[AHW_SLOT_COUNT] = {0, 0, 0, 0};
template <typename Tag>
volatile uint32_t AhwTraceStorage<Tag>::hits  [AHW_SLOT_COUNT] = {0, 0, 0, 0};

// ── RAII timer ──────────────────────────────────────────────────────
struct AhwScopedTrace {
    int      slot;
    uint32_t start;
    explicit AhwScopedTrace(int s) : slot(s), start(ahw_now_ticks()) {}
    ~AhwScopedTrace() {
        AhwTraceStorage<>::cycles[slot] += ahw_now_ticks() - start;
        AhwTraceStorage<>::hits  [slot] += 1;
    }
    AhwScopedTrace(const AhwScopedTrace&)            = delete;
    AhwScopedTrace& operator=(const AhwScopedTrace&) = delete;
};

#  define AHW_TRACE_CAT2_(a, b) a##b
#  define AHW_TRACE_CAT_(a, b)  AHW_TRACE_CAT2_(a, b)
#  define AHW_TRACE(slot) \
       AhwScopedTrace AHW_TRACE_CAT_(_ahw_trace_, __LINE__)(slot)
#  define AHW_TRACE_GET(slot)   (AhwTraceStorage<>::cycles[slot])
#  define AHW_TRACE_COUNT(slot) (AhwTraceStorage<>::hits[slot])
#  define AHW_TRACE_RESET()     do {                              \
       for (int _ahw_i = 0; _ahw_i < AHW_SLOT_COUNT; ++_ahw_i) {  \
           AhwTraceStorage<>::cycles[_ahw_i] = 0;                 \
           AhwTraceStorage<>::hits  [_ahw_i] = 0;                 \
       }                                                          \
   } while (0)

#else // AHW_INSTRUMENT off — zero-cost stubs

#  define AHW_TRACE(slot)       ((void)0)
#  define AHW_TRACE_GET(slot)   ((uint32_t)0)
#  define AHW_TRACE_COUNT(slot) ((uint32_t)0)
#  define AHW_TRACE_RESET()     ((void)0)

#endif // AHW_INSTRUMENT

#endif // AHW_TRACE_H
