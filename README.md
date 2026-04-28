# AdditiveHW

Additive Holt-Winters forecasting for embedded systems — trend, seasonality, auto-refit, prediction intervals.

## Equations

```
l_t     = α·(y_t − s_{t−m})  + (1−α)·(l_{t−1} + b_{t−1})     level
b_t     = β·(l_t − l_{t−1})  + (1−β)·b_{t−1}                  trend
s_t     = γ·(y_t − l_t)      + (1−γ)·s_{t−m}                  seasonal
ŷ_{t+h} = l_t + h·b_t + s_{t+h−m}                             forecast
```

Where `m` is the seasonal period (e.g. 24 for hourly data with a daily cycle),
and `α`, `β`, `γ` are smoothing parameters in [0, 1].

## How `forecast(h)` works

`forecast(h)` returns **one value**: the point prediction `h` steps ahead.

- `forecast(1)` — next period
- `forecast(6)` — 6 periods from now
- `forecast(24)` — 24 periods from now (e.g. same hour tomorrow if m=24)

It does **not** return an array. If you want the next 24 forecast values:

```cpp
for (int h = 1; h <= 24; h++) {
    Serial.println(hw.forecast(h));
}
```

`forecastPI(h)` works the same way but also returns a prediction interval:

```cpp
HWForecast f = hw.forecastPI(1);  // one step ahead
// f.yhat  — point forecast
// f.lower — lower 95% bound
// f.upper — upper 95% bound
// f.sigma — forecast std deviation
```

## Features

- **Level + trend + seasonal** decomposition (additive model)
- **Automatic parameter refit** via coordinate-descent optimizer
- **Prediction intervals** — exact ETS(A,A,A) variance formula (Hyndman et al. 2008)
- **Deferred refit** — update() stays O(1); optimizer runs when you have time
- **~1.7 KB RAM** (on AVR; actual layout varies by platform) — fits on nRF52840 and Arduino Uno
- **No dependencies** — header-only core, no STL, no dynamic allocation, no RTOS required

## Supported Platforms

| Platform | Toolchain | Refit time | Interface |
|----------|-----------|------------|-----------|
| **nRF52840 Dongle** (Zephyr / nRF Connect SDK) | west / CMake | < 1 ms | C API (`additive_hw_api.h`) |
| **micro:bit v2** (Zephyr) | west / CMake | < 1 ms | C API (`additive_hw_api.h`) |
| **Arduino Uno** (ATmega328P) | Arduino IDE | ~200 ms | C++ class (`AdditiveHW.h`) |
| Any Cortex-M with FPU | GCC / Clang | < 1 ms | C or C++ |

## Repository Structure

```
src/
  AdditiveHW.h            Core C++ header (platform-independent)
  additive_hw_api.h       C-callable API (extern "C")
  additive_hw_api.cpp     C wrapper implementation

examples/
  zephyr/                 Zephyr — k_msgq + forecast
  zephyr_ble/             Zephyr BLE — GATT write + notify (UTF-8 strings, mutex-protected)
  Uno_BasicForecast/      Arduino Uno — immediate mode
  Uno_DeferredRefit/      Arduino Uno — deferred refit with timing
  Uno_RTC_HourlyForecast/ Arduino Uno — DS3231 + hourly aggregation
  Uno_SyntheticTest/      Arduino Uno — deterministic 72-sample bench, no hardware

test_additive_hw.cpp      Host test suite (128 tests, g++ -std=c++11)
```

## Quick Start — Zephyr / nRF Connect SDK

1. Copy `src/AdditiveHW.h`, `src/additive_hw_api.h`, `src/additive_hw_api.cpp`
   into your application's `src/` directory.
2. In `prj.conf`:
   ```
   CONFIG_CPP=y
   CONFIG_NEWLIB_LIBC=y
   ```
3. In `CMakeLists.txt`:
   ```cmake
   target_sources(app PRIVATE main.c additive_hw_api.cpp)
   ```

Build for your board:
```
west build -b nrf52840dongle/nrf52840       # Nordic nRF52840 Dongle
west build -b bbc_microbit_v2               # micro:bit v2
```

See `examples/zephyr/` for basic usage, `examples/zephyr_ble/` for BLE.

### Testing the BLE example

The BLE example uses human-readable UTF-8 strings — no hex conversion needed.

1. Install **nRF Connect** on your phone (iOS / Android).
2. Scan and connect to **"AHW-Forecast"**.
3. On characteristic `12340001`, select **UTF-8**, type a number (e.g. `23.6`), tap Write.
4. On characteristic `12340002`, enable notifications and set display to **UTF-8**.
5. After 24 observations the forecast appears as: `fc=23.4 lo=19.1 hi=27.8`
   (before that you'll see `waiting for data...`).

```c
#include "additive_hw_api.h"

ahw_init(12, true);              /* season=12, deferred refit */

/* Feed data — O(1) when deferred, but not ISR-safe without
   synchronisation (see examples/zephyr/ for safe patterns). */
ahw_update(sensor_value);

/* In main loop: */
ahw_process();

ahw_forecast_t f;
if (ahw_forecast(1, &f)) {      /* 1-step ahead */
    LOG_INF("fc=%d  [%d .. %d]",
            (int)f.yhat, (int)f.lower, (int)f.upper);
}
```

## Quick Start — Arduino Uno

Install via **Sketch → Include Library → Add .ZIP Library**, or copy to your
Arduino libraries folder.

```cpp
#include <avr/sleep.h>
#include <AdditiveHW.h>

AdditiveHW hw;
unsigned long lastMs = 0;

void lowPowerSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void setup() {
    Serial.begin(9600);
    hw.begin(12);
    hw.deferRefit(true);
}

void loop() {
    if (millis() - lastMs < 5000) { lowPowerSleep(); return; }
    lastMs = millis();

    hw.update(analogRead(A0) * (5.0 / 1023.0));

    if (hw.isReady())
        Serial.println(hw.forecast(1));  // 1-step ahead

    if (!hw.processRefit())
        lowPowerSleep();
}
```

## C API Reference (`additive_hw_api.h`)

| Function | Description |
|----------|-------------|
| `ahw_init(season_len, defer)` | Initialise model. Returns `bool` — `false` if `season_len` was clamped to `MAX_SEASON`. |
| `ahw_update(y)` | Feed one observation (O(1) when deferred) |
| `ahw_process()` | Run pending refit. Returns `true` if work done |
| `ahw_forecast(h, &f)` | h-step-ahead forecast + 95% PI. Returns `true` if ready |
| `ahw_is_ready()` | Model bootstrapped? |
| `ahw_level()` | Current level (l_t) |
| `ahw_trend()` | Current trend (b_t) |
| `ahw_sigma2()` | Residual variance (σ²) |

## C++ API Reference (`AdditiveHW.h`)

| Method | Description |
|--------|-------------|
| `begin(seasonLen, window, refitEvery, α, β, γ)` | Configure model. Returns `bool` — `false` if any argument was clamped (e.g. `seasonLen > MAX_SEASON`, α/β/γ outside `[0, 1]`). |
| `deferRefit(bool)` | Enable/disable deferred mode |
| `update(y)` | Feed one observation |
| `processRefit()` | Run pending refit |
| `forecast(h)` | h-step-ahead point forecast (returns one value) |
| `forecastPI(h, z)` | h-step-ahead forecast + prediction interval (PI width capped at h = 4·m) |
| `isReady()` | Model bootstrapped? |
| `level()`, `trend()` | Current state |
| `fitted()`, `residual()` | Last online one-step-ahead prediction and error |
| `sigma2()` | Residual variance |
| `smoothing()` | Current α, β, γ as `HWParams` |

## Profiling (optional)

Compile with `-DAHW_INSTRUMENT` to enable a lightweight RAII tracer on
the four hot paths.  Zero cost (every macro is `((void)0)`) when undefined.

```cpp
#include <AdditiveHW.h>

// ... after some updates ...
Serial.print(F("refit ticks=")); Serial.print(AHW_TRACE_GET(AHW_SLOT_REFIT));
Serial.print(F(" hits="));        Serial.println(AHW_TRACE_COUNT(AHW_SLOT_REFIT));
AHW_TRACE_RESET();
```

Slots: `AHW_SLOT_ONLINE`, `AHW_SLOT_REFIT`, `AHW_SLOT_OPTIMIZE`, `AHW_SLOT_MSE`.

Time source picked automatically: Cortex-M `DWT->CYCCNT` (define
`AHW_TRACE_USE_DWT` first), Arduino `micros()`, x86_64 `__rdtsc()`,
or `std::chrono::steady_clock` ns elsewhere.

## Limitations

- **Season length** — `MAX_SEASON` defaults to 24.  You can increase it by editing
  the constant in `AdditiveHW.h`.  `MAX_WINDOW` is a power of two so ring
  wraps are a single AND; bump it (also to a power of two) when raising
  `MAX_SEASON` past 32 so that `MAX_WINDOW ≥ 3·MAX_SEASON`.  Approximate
  RAM usage:

  | `MAX_SEASON` | `MAX_WINDOW` | `sizeof` (approx) | Stack (refit) | Arduino Uno? |
  |-------------:|-------------:|------------------:|--------------:|:-------------|
  |           24 |          128 |          1.7 KB   |         ~80 B | Yes (default) |
  |           32 |          128 |          1.9 KB   |         ~80 B | Tight |
  |           48 |          256 |          3.4 KB   |         ~80 B | No — use Cortex-M |
  |           96 |          512 |          6.5 KB   |         ~80 B | No — use Cortex-M |
  |          168 |          512 |          8.2 KB   |         ~80 B | No — use Cortex-M |

  Refit-time stack frame is nearly constant: the seasonal scratch used
  by `computeMSE` was lifted to a class member, so coord-descent no
  longer allocates `MAX_SEASON × 4 B` on every call.

  On Zephyr / nRF52840 / any Cortex-M with ≥ 16 KB RAM, values up to 168 or
  higher work fine.  On Arduino Uno (2 KB SRAM), keep `MAX_SEASON` ≤ 32.

- **Prediction interval cap** — the variance multiplier inside `forecastPI(h)` is
  capped at `h = 4·m`.  For larger horizons the point forecast keeps moving but
  the prediction interval width stays at its `4·m` value.  This prevents
  unbounded iteration on small MCUs.
- **C API is a singleton** — `additive_hw_api.cpp` uses a single global `AdditiveHW`
  instance.  If you need multiple models, use the C++ class directly.
- **C API surface** — `ahw_init()` only exposes `season_len` and `defer_refit`.
  To set `window`, `refitEvery`, or initial α/β/γ, use the C++ `begin()` method.

## License

MIT
