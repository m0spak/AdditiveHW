// Uno_DeferredRefit.ino — update() always < 1 ms, refit runs separately.
// Shows timing for update and refit.

#include <avr/sleep.h>
#include <AdditiveHW.h>

AdditiveHW hw;
unsigned long lastMs = 0;
unsigned long n = 0;

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
    // Sample every 5 seconds
    if (millis() - lastMs >= 5000) {
        lastMs = millis();
        n++;

        float y = analogRead(A0) * (5.0f / 1023.0f);

        unsigned long t0 = micros();
        hw.update(y);
        unsigned long dt = micros() - t0;

        if (hw.isReady()) {
            Serial.print(F("y="));   Serial.print(y, 2);
            Serial.print(F(" fc=")); Serial.print(hw.forecast(1), 2);
            Serial.print(F(" "));    Serial.print(dt);
            Serial.print(F("us"));
            if (hw.refitPending()) Serial.print(F(" *"));
            Serial.println();
        } else {
            Serial.print(n);
            Serial.print(F("/"));
            Serial.println(2 * hw.seasonLen());
        }
    }

    // Refit when pending, sleep when idle
    if (hw.refitPending()) {
        unsigned long t0 = millis();
        hw.processRefit();
        Serial.print(F("Refit "));
        Serial.print(millis() - t0);
        Serial.println(F("ms"));
    } else {
        lowPowerSleep();
    }
}
