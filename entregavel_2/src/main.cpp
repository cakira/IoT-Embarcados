/* This code is released under CC0 1.0 Universal (Public Domain).
 * You can copy, modify, and distribute it for any purpose.
 */

// This code controls a bathtube

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

// Pin definitions
constexpr int PIN_ID0 = 15;
constexpr int PIN_ID1 = 23;
constexpr int PIN_TEMPERATURE = 32;

// Temperature sensor definitions
constexpr int ADC_RESOLUTION = 4095;
constexpr float THERMISTOR_BETA = 3950;

// WiFi definitions
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT Server (broker) Configuration
constexpr const char* MQTT_BROKER = "broker.emqx.io";
constexpr int MQTT_PORT = 1883;

// MQTT variable Labels from device to broker
constexpr const char* VAR_LATITUDE = "lat";
constexpr const char* VAR_LONGITUDE = "lon";
constexpr const char* VAR_TEMPERATURE = "temp";

constexpr unsigned long UPDATE_DELAY_MS = 2000;

// Group monitored data into a single structure
struct SystemState {
    float latitude;
    float longitude;
    float temp;
};

// Global Variables
WiFiClient espClient;
PubSubClient mqtt(espClient);
SystemState current_state;

unsigned int device_id = 0;
unsigned long next_update_time;

unsigned int readDeviceId() {
    // Note:
    // Due to the hardware construction, when the switch pin is
    // in the ON position, the pin level is LOW
    unsigned int id = 0;
    id += digitalRead(PIN_ID0) == LOW ? 1 : 0;
    id += digitalRead(PIN_ID1) == LOW ? 2 : 0;
    return id;
}

// Convert analog NTC reading to Celsius
float readTemperature(int pin) {
    int analog_value = analogRead(pin);
    // Avoid division by zero if reading is 0 or 4095
    if (analog_value == 0 || analog_value >= ADC_RESOLUTION)
        return -273.15;

    return 1.0 /
        (log(1.0 / (ADC_RESOLUTION / (float)analog_value - 1)) /
                THERMISTOR_BETA +
            1.0 / 298.15) -
        273.15;
}

void updatePosition(SystemState& state) {
    // Ideally, the position should be set by a GPS, but there's no GPS in Wokwi
    switch (device_id) {
    case 0:
        state.latitude = -22.8162268;
        state.longitude = -47.0451902;
        break;
    case 1:
        state.latitude = -22.902735;
        state.longitude = -47.0563132;
        break;
    case 2:
        state.latitude = -22.8517996;
        state.longitude = -47.1284529;
        break;
    default:
        Serial.printf("Device ID %u is not accepted. Freezing.\r\n", device_id);
        // Freezing
        while (true) {
        };
    }
}

void connectWiFi() {
    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\r\n[WiFi] Connected!");
}

void connectMQTT() {
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);

    while (!mqtt.connected()) {
        Serial.print("[MQTT] Connecting to MQTTx...");

        if (mqtt.connect("")) {
            Serial.println(" Connected!");
        } else {
            Serial.print(" failed (rc=");
            Serial.print(mqtt.state());
            Serial.println("). Retrying in 2s...");
            delay(2000);
        }
    }
}

void publishToBroker(const SystemState& state) {
    const size_t MSG_BUFFER_SIZE = 256;
    char payload[MSG_BUFFER_SIZE];

    // Format the JSON using the struct members
    int ret = snprintf(payload, MSG_BUFFER_SIZE,
        "{\"%s\": %f, \"%s\": %f, \"%s\": %.2f}", VAR_LATITUDE, state.latitude,
        VAR_LONGITUDE, state.longitude, VAR_TEMPERATURE, state.temp);

    // Validate and Publish
    if (ret >= 0 && ret < MSG_BUFFER_SIZE) {
        char topic[100];
        snprintf(topic, sizeof(topic),
            "IoT-Embarcados/akira/entrega/2/sensor/%u", device_id);

        Serial.print("[MQTT] Topic: ");
        Serial.println(topic);
        Serial.print("[MQTT] Payload: ");
        Serial.println(payload);

        mqtt.publish(topic, payload);
    } else {
        Serial.println("[Error] Payload truncation occurred.");
    }
}

// Setup called automatically at initialization
void setup() {
    Serial.begin(115200);

    Serial.println("\r\n");
    Serial.println("*");
    Serial.println("* Program start");
    Serial.println("*");

    // Configure GPIOs
    pinMode(PIN_ID0, INPUT_PULLUP);
    pinMode(PIN_ID1, INPUT_PULLUP);

    device_id = readDeviceId();
    if (device_id >= 3) {
        Serial.printf(
            "[CONF] Device ID %u is not accepted. Freezing.\r\n", device_id);
        // Freezing
        while (true) {
        };
    }
    Serial.printf("[CONF] Device ID: %u\r\n", device_id);

    connectWiFi();
    connectMQTT();

    next_update_time = millis();
}

// Main loop, executed automatically and constantly
void loop() {
    // Reconnect if connection is lost
    if (!mqtt.connected()) {
        connectMQTT();
    }
    mqtt.loop(); // Handle incoming messages and keepalive

    unsigned long now = millis();

    // Non-blocking delay for publishing data
    if (now > next_update_time) {
        next_update_time += UPDATE_DELAY_MS;

        current_state.temp = readTemperature(PIN_TEMPERATURE);
        updatePosition(current_state);

        Serial.printf(
            "[SENS] Current temperature: %.2f.C\r\n", current_state.temp);
        publishToBroker(current_state);
    }
}
