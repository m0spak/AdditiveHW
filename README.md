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
- **~736 B RAM** (on AVR; actual layout varies by platform) — fits on nRF52840 and even Arduino Uno
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
  zephyr_ble/             Zephyr BLE — GATT write + notify (mutex-protected)
  Uno_BasicForecast/      Arduino Uno — immediate mode
  Uno_DeferredRefit/      Arduino Uno — deferred refit with timing
  Uno_RTC_HourlyForecast/ Arduino Uno — DS3231 + hourly aggregation

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
| `ahw_init(season_len, defer)` | Initialise model |
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
| `begin(seasonLen, window, refitEvery, α, β, γ)` | Configure model |
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

## Limitations

- **Season length** — `MAX_SEASON` defaults to 24.  You can increase it by editing
  the constant in `AdditiveHW.h`.  `MAX_WINDOW` (= 3 × `MAX_SEASON`) and the
  `computeMSE` stack frame scale with it.  Approximate RAM usage:

  | `MAX_SEASON` | `sizeof` (approx) | Stack (refit) | Arduino Uno? |
  |-------------:|------------------:|--------------:|:-------------|
  |           24 |            752 B  |         96 B  | Yes (default) |
  |           32 |            976 B  |        128 B  | Yes          |
  |           48 |          1.4 KB   |        192 B  | Tight        |
  |           96 |          2.8 KB   |        384 B  | No — use Cortex-M |
  |          168 |          4.8 KB   |        672 B  | No — use Cortex-M |

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
