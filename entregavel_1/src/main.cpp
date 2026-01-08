/* This code is released under CC0 1.0 Universal (Public Domain).
 * You can copy, modify, and distribute it for any purpose.
 */

// This code controls a bathtube

#include <Arduino.h>

// Pin definitions
constexpr int SYSTEM_SWITCH_PIN = 23;
constexpr int TEMPERATURE_PIN_1 = 32;
constexpr int TEMPERATURE_PIN_2 = 33;
constexpr int WATER_LEVEL_PIN = 35;
constexpr int HOT_WATER_PIN = 18;
constexpr int COLD_WATER_PIN = 19;
constexpr int DRAIN_PIN = 5;

constexpr float THERMISTOR_BETA = 3950;

void setup() {
    Serial.begin(115200);

    // Configure GPIOs
    pinMode(SYSTEM_SWITCH_PIN, INPUT);
    pinMode(HOT_WATER_PIN, OUTPUT);
    pinMode(COLD_WATER_PIN, OUTPUT);
    pinMode(DRAIN_PIN, OUTPUT);
}

void loop() {
    // This code only prints the analog values and
    // blink the leds when the power on swith is turned on
    static int leds_control = LOW;

    int temperature_reading_1 = analogRead(TEMPERATURE_PIN_1);
    int temperature_reading_2 = analogRead(TEMPERATURE_PIN_2);
    float temperature_1 = 1 /
            (log(1 / (4095. / temperature_reading_1 - 1)) / THERMISTOR_BETA +
                1.0 / 298.15) -
        273.15;
    float temperature_2 = 1 /
            (log(1 / (4095. / temperature_reading_2 - 1)) / THERMISTOR_BETA +
                1.0 / 298.15) -
        273.15;

    int water_level_reading = analogRead(WATER_LEVEL_PIN);

    Serial.printf("Reading 1  : %d - Temp = %.2f\r\n", temperature_reading_1,
        temperature_1);
    Serial.printf("Reading 2  : %d - Temp = %.2f\r\n", temperature_reading_2,
        temperature_2);
    Serial.printf("Water level: %d\r\n", water_level_reading);

    if (digitalRead(SYSTEM_SWITCH_PIN) == LOW) {
        leds_control = LOW;
    } else {
        leds_control = (leds_control == LOW ? HIGH : LOW);
    }

    digitalWrite(HOT_WATER_PIN, leds_control);
    digitalWrite(COLD_WATER_PIN, leds_control);
    digitalWrite(DRAIN_PIN, leds_control);

    delay(1000);
}
