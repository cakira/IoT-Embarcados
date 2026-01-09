/* This code is released under CC0 1.0 Universal (Public Domain).
 * You can copy, modify, and distribute it for any purpose.
 */

// This code controls a bathtube

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

// Pin definitions
constexpr int SYSTEM_SWITCH_PIN = 23;
constexpr int TEMPERATURE_PIN_1 = 32;
constexpr int TEMPERATURE_PIN_2 = 33;
constexpr int WATER_LEVEL_PIN = 35;
constexpr int HOT_WATER_PIN = 18;
constexpr int COLD_WATER_PIN = 19;
constexpr int DRAIN_PIN = 5;

constexpr float THERMISTOR_BETA = 3950;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT / Ubidots Configuration
constexpr const char* MQTT_BROKER = "industrial.api.ubidots.com";
constexpr int MQTT_PORT = 1883;
constexpr const char* MQTT_CLIENTID = "Bathtub";
constexpr const char* UBIDOTS_TOKEN = "BBUS-6aZS0jNXGwWelkXlikrfk96WVKKuCI";

// Device Label (The 'Device' in Ubidots)
constexpr const char* DEVICE_LABEL = "bathtub";

constexpr unsigned long UPDATE_DELAY_MS = 1000;

// Global Variables
WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long next_update_time;

void connectWiFi() {
    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[WiFi] Connected!");
}

// Callback executed when a subscribed variable changes in the broker
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Convert payload to a C-string
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    // Convert to float value
    float value = atof(message);

    Serial.print("[MQTT] Received on [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(value);
}

void connectMQTT() {
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);

    while (!mqtt.connected()) {
        Serial.print("[MQTT] Connecting to Ubidots...");

        // Connect with Token as username, empty password
        if (mqtt.connect(MQTT_CLIENTID, UBIDOTS_TOKEN, "")) {
            Serial.println(" Connected!");
        } else {
            Serial.print(" failed (rc=");
            Serial.print(mqtt.state());
            Serial.println("). Retrying in 2s...");
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Configure GPIOs
    pinMode(SYSTEM_SWITCH_PIN, INPUT);
    pinMode(HOT_WATER_PIN, OUTPUT);
    pinMode(COLD_WATER_PIN, OUTPUT);
    pinMode(DRAIN_PIN, OUTPUT);

    connectWiFi();
    connectMQTT();

    next_update_time = millis();
}

void loop() {
    static int leds_control = LOW;

    // Reconnect if connection is lost
    if (!mqtt.connected()) {
        connectMQTT();
    }
    mqtt.loop(); // Handle incoming messages and keepalive

    unsigned long now = millis();

    if (now > next_update_time) {
        next_update_time += UPDATE_DELAY_MS;

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
    }
}
