// Uno_SyntheticTest.ino — deterministic bench test, no hardware needed.
//
// Based on Uno_BasicForecast.ino with these changes:
//   1. analogRead(A0) replaced with a synthetic signal:
//      y = 20.0 + 0.5*t + 5.0*sin(2π·t/12)
//   2. Sampling interval reduced from 5000 ms to 1000 ms (faster test).
//   3. Stops after 72 samples (6 full cycles of m=12).
//   4. Prints expected next value alongside the forecast so you can
//      compare them visually in Serial Monitor.
//
// Expected behaviour:
//   - Model becomes ready at sample 24 (2×m).
//   - Forecasts start rough, then converge over the next cycles.
//   - By the last 1–2 cycles, forecast error should be small (< 1.0).
//   - Total run time: ~72 seconds.
//
// Upload, open Serial Monitor at 9600, watch it run.

#include <avr/sleep.h>
#include <AdditiveHW.h>

AdditiveHW hw;
unsigned long lastMs = 0;
unsigned long n = 0;

// ── Synthetic signal instead of analogRead(A0) ──────────────────────
// y = 20.0 + 0.5*t + 5.0*sin(2π·t/12)
// Known level=20, trend=0.5/step, seasonal amplitude=5, period=12.
static float synth(unsigned long t) {
    return 20.0f + 0.5f * (float)t
         + 5.0f * sin(2.0f * 3.14159265f * (float)t / 12.0f);
}

// ── Total number of samples: 6 full cycles ──────────────────────────
#define N_SAMPLES 72

void lowPowerSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void setup() {
    Serial.begin(9600);
    hw.begin(12);

    // ── Header so Serial Monitor output is easy to read ─────────────
    Serial.println(F("Uno_SyntheticTest — 72 samples, m=12"));
    Serial.println(F("n\ty\tfc\texpected\terr"));
}

void loop() {
    // ── Stop after N_SAMPLES ────────────────────────────────────────
    if (n >= N_SAMPLES) {
        lowPowerSleep();
        return;
    }

    // ── 1-second interval instead of 5 seconds (faster bench test) ──
    if (millis() - lastMs < 1000) {
        lowPowerSleep();
        return;
    }
    lastMs = millis();

    float y = synth(n);
    hw.update(y);
    n++;

    if (hw.isReady()) {
        // ── Print forecast AND the known expected next value ─────────
        float fc       = hw.forecast(1);
        float expected  = synth(n);       // what the next sample will be
        float err       = fc - expected;

        Serial.print(n);
        Serial.print(F("\t"));
        Serial.print(y, 2);
        Serial.print(F("\t"));
        Serial.print(fc, 2);
        Serial.print(F("\t"));
        Serial.print(expected, 2);
        Serial.print(F("\t"));
        Serial.println(err, 2);
    } else {
        // ── Bootstrap progress ──────────────────────────────────────
        Serial.print(n);
        Serial.print(F("/"));
        Serial.println(2 * hw.seasonLen());
    }
}
