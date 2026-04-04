// Uno_RTC_HourlyForecast.ino
//
// Arduino Uno + DS3231 — Holt-Winters hourly temperature forecast.
//
// Samples a TMP36 on A0 every 5 seconds, computes the hourly mean,
// and feeds one value per completed hour into the model.
//
// Alignment:
//   The model does not start immediately. It waits until it observes
//   a real transition into hour 0 (midnight), then starts collecting
//   a clean full hour 0. This keeps s[0]=midnight, s[12]=noon, etc.
//
// If the board starts mid-day, it still computes and prints hourly
// means, but marks them as "(skip)" until alignment at midnight.
//
// Ready after 48 hourly observations (2 full days).

#include <Wire.h>
#include <RTClib.h>
#include <AdditiveHW.h>
#include <avr/sleep.h>

RTC_DS3231 rtc;
AdditiveHW hw;

bool          running     = false;
int           prevHour    = -1;
uint16_t      sampleCount = 0;
float         hourlyMean  = 0.0f;
unsigned long lastMs      = 0;
unsigned long hourCount   = 0;

float readTMP36() {
    return (analogRead(A0) * (5.0f / 1023.0f) - 0.5f) * 100.0f;
}

void lowPowerSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void setup() {
    Serial.begin(9600);

    if (!rtc.begin()) {
        Serial.println(F("RTC not found"));
        while (1) {}
    }

    if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    hw.begin(24);
    hw.deferRefit(true);

    prevHour = rtc.now().hour();
    running  = false;   // wait for a real transition into h=0

    Serial.print(F("AdditiveHW RTC h="));
    Serial.print(prevHour);
    Serial.println(F(" waiting for transition to h=0"));
}

void loop() {
    if (millis() - lastMs < 5000UL) {
        if (!hw.processRefit()) {
            lowPowerSleep();
        }
        return;
    }
    lastMs = millis();

    DateTime now = rtc.now();
    int h = now.hour();
    float t = readTMP36();

    // Handle hour boundary first.
    // This ensures the first sample of the new hour is not included
    // in the previous hour's mean.
    if (h != prevHour) {
        if (sampleCount > 0) {
            if (!running) {
                Serial.print(F("h="));
                Serial.print(prevHour);
                Serial.print(F(" y="));
                Serial.print(hourlyMean, 1);
                Serial.println(F(" (skip)"));
            } else {
                hw.update(hourlyMean);
                hourCount++;

                Serial.print(hourCount);
                Serial.print(F(" h="));
                Serial.print(prevHour);
                Serial.print(F(" y="));
                Serial.print(hourlyMean, 1);

                if (hw.isReady()) {
                    HWForecast f = hw.forecastPI(1);
                    Serial.print(F(" fc="));
                    Serial.print(f.yhat, 1);
                    Serial.print(F(" ["));
                    Serial.print(f.lower, 1);
                    Serial.print(F(".."));
                    Serial.print(f.upper, 1);
                    Serial.print(F("]"));
                } else {
                    Serial.print(F(" "));
                    Serial.print(hourCount);
                    Serial.print(F("/48"));
                }
                Serial.println();
            }
        }

        // Start only when we see a real boundary into hour 0.
        if (!running && h == 0) {
            running = true;
            hourCount = 0;
            Serial.println(F("Aligned at h=0"));
        }

        prevHour = h;
        sampleCount = 0;
        hourlyMean = 0.0f;
    }

    // Accumulate the current sample into the current hour.
    sampleCount++;
    hourlyMean += (t - hourlyMean) / (float)sampleCount;
}
