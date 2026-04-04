// Uno_BasicForecast.ino — read A0 every 5 s, forecast, print.
// Immediate mode: refit runs inline (~200 ms every seasonLen obs).

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
}

void loop() {
    if (millis() - lastMs < 5000) {
        lowPowerSleep();
        return;
    }
    lastMs = millis();

    float y = analogRead(A0) * (5.0f / 1023.0f);
    hw.update(y);
    n++;

    if (hw.isReady()) {
        HWForecast f = hw.forecastPI(1);
        Serial.print(F("y="));   Serial.print(y, 2);
        Serial.print(F(" fc=")); Serial.print(f.yhat, 2);
        Serial.print(F(" ["));   Serial.print(f.lower, 2);
        Serial.print(F(".."));   Serial.print(f.upper, 2);
        Serial.println(F("]"));
    } else {
        Serial.print(n);
        Serial.print(F("/"));
        Serial.println(2 * hw.seasonLen());
    }
}
