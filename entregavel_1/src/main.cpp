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

constexpr int ADC_RESOLUTION = 4095;

constexpr float THERMISTOR_BETA = 3950;

constexpr float WATER_MIN_MEASUREMENT_CM = 0.0;
constexpr float WATER_MAX_MEASUREMENT_CM = 100.0;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT / Ubidots Configuration
constexpr const char* MQTT_BROKER = "industrial.api.ubidots.com";
constexpr int MQTT_PORT = 1883;
constexpr const char* MQTT_CLIENTID = "Bathtub";
constexpr const char* UBIDOTS_TOKEN = "BBUS-6aZS0jNXGwWelkXlikrfk96WVKKuCI";

// Device Label (The 'Device' in Ubidots)
constexpr const char* DEVICE_LABEL = "bathtub";

// MQTT variable Labels from device to broker
constexpr const char* VAR_TEMP_AVG = "avg-temperature";
constexpr const char* VAR_LEVEL = "water-level";
constexpr const char* VAR_HOT_WATER = "hot-water-status";
constexpr const char* VAR_COLD_WATER = "cold-water-status";
constexpr const char* VAR_DRAIN = "drain-status";
constexpr const char* VAR_SYSTEM_ACTIVE = "system-active";

// MQTT variable Labels from broker to device
// Note: We append "/lv" in the topic string to get the "Last Value" only
constexpr const char* VAR_CONF_TEMP_MAX = "temp-max";
constexpr const char* VAR_CONF_TEMP_MIN = "temp-min";
constexpr const char* VAR_CONF_LEVEL_MAX = "level-max";
constexpr const char* VAR_CONF_LEVEL_MIN = "level-min";

constexpr unsigned long UPDATE_DELAY_MS = 2000;

// Group monitored data into a single structure
struct SystemState {
    float avg_temp;
    float water_level;
    int hot_status;
    int cold_status;
    int drain_status;
    int system_switch_state;
};

// Global Variables
WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long next_update_time;

// Control Configuration (Default values, updated via MQTT)
float config_temp_max = 40.0;
float config_temp_min = 35.0;
float config_level_max = 40.0; // in cm
float config_level_min = 50.0; // in cm

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

float readWaterLevel(int pin) {
    int analog_value = analogRead(pin);

    return WATER_MIN_MEASUREMENT_CM +
        ((float)analog_value / ADC_RESOLUTION) *
        (WATER_MAX_MEASUREMENT_CM - WATER_MIN_MEASUREMENT_CM);
}

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

    // Identify which variable this message belongs to and update globals
    // We check if the topic string contains the variable label
    if (strstr(topic, VAR_CONF_TEMP_MAX) != nullptr) {
        config_temp_max = value;
        Serial.printf(
            "[CONFIG] Max Temp updated to: %.2f.C\r\n", config_temp_max);
    } else if (strstr(topic, VAR_CONF_TEMP_MIN) != nullptr) {
        config_temp_min = value;
        Serial.printf(
            "[CONFIG] Min Temp updated to: %.2f.C\r\n", config_temp_min);
    } else if (strstr(topic, VAR_CONF_LEVEL_MAX) != nullptr) {
        config_level_max = value;
        Serial.printf(
            "[CONFIG] Max Level updated to: %fcm\r\n", config_level_max);
    } else if (strstr(topic, VAR_CONF_LEVEL_MIN) != nullptr) {
        config_level_min = value;
        Serial.printf(
            "[CONFIG] Min Level updated to: %fcm\r\n", config_level_min);
    }
}

void connectMQTT() {
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);

    while (!mqtt.connected()) {
        Serial.print("[MQTT] Connecting to Ubidots...");

        // Connect with Token as username, empty password
        if (mqtt.connect(MQTT_CLIENTID, UBIDOTS_TOKEN, "")) {
            Serial.println(" Connected!");

            // Subscribe to Configuration Topics
            char sub_topic[100];

            // Subscribe Max Temp
            snprintf(sub_topic, sizeof(sub_topic), "/v1.6/devices/%s/%s/lv",
                DEVICE_LABEL, VAR_CONF_TEMP_MAX);
            mqtt.subscribe(sub_topic);

            // Subscribe Min Temp
            snprintf(sub_topic, sizeof(sub_topic), "/v1.6/devices/%s/%s/lv",
                DEVICE_LABEL, VAR_CONF_TEMP_MIN);
            mqtt.subscribe(sub_topic);

            // Subscribe Max Level
            snprintf(sub_topic, sizeof(sub_topic), "/v1.6/devices/%s/%s/lv",
                DEVICE_LABEL, VAR_CONF_LEVEL_MAX);
            mqtt.subscribe(sub_topic);

            // Subscribe Min Level
            snprintf(sub_topic, sizeof(sub_topic), "/v1.6/devices/%s/%s/lv",
                DEVICE_LABEL, VAR_CONF_LEVEL_MIN);
            mqtt.subscribe(sub_topic);

            Serial.println("[MQTT] Subscribed to configuration topics.");

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
        "{\"%s\": %.2f, \"%s\": %.2f, \"%s\": %d, \"%s\": %d, \"%s\": %d, "
        "\"%s\": %d}",
        VAR_TEMP_AVG, state.avg_temp, VAR_LEVEL, state.water_level,
        VAR_HOT_WATER, state.hot_status, VAR_COLD_WATER, state.cold_status,
        VAR_DRAIN, state.drain_status, VAR_SYSTEM_ACTIVE,
        state.system_switch_state);

    // Validate and Publish
    if (ret >= 0 && ret < MSG_BUFFER_SIZE) {
        char topic[100];
        snprintf(topic, sizeof(topic), "/v1.6/devices/%s", DEVICE_LABEL);

        Serial.print("[MQTT] Publishing: ");
        Serial.println(payload);

        mqtt.publish(topic, payload);
    } else {
        Serial.println("[Error] Payload truncation occurred.");
    }
}

// Setup called automatically at initialization
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

// Main loop, executed automatically and constantly
void loop() {
    static int leds_control = LOW;

    // Reconnect if connection is lost
    if (!mqtt.connected()) {
        connectMQTT();
    }
    mqtt.loop(); // Handle incoming messages and keepalive

    unsigned long now = millis();

    // Non-blocking delay for publishing data
    if (now > next_update_time) {
        next_update_time += UPDATE_DELAY_MS;

        SystemState current_state;

        // Read Inputs
        float temp1 = readTemperature(TEMPERATURE_PIN_1);
        float temp2 = readTemperature(TEMPERATURE_PIN_2);
        current_state.avg_temp = (temp1 + temp2) / 2.0;

        current_state.water_level = readWaterLevel(WATER_LEVEL_PIN);
        current_state.system_switch_state = digitalRead(SYSTEM_SWITCH_PIN);

        Serial.printf("Temperature 1 : %.2f.C\r\n", temp1);
        Serial.printf("Temperature 2 : %.2f.C\r\n", temp2);
        Serial.printf("Water level: %.1fcm\r\n", current_state.water_level);

        // Control Logic - currently, we just test the leds
        if (current_state.system_switch_state == LOW) {
            leds_control = LOW;
        } else {
            leds_control = (leds_control == LOW ? HIGH : LOW);
        }

        digitalWrite(HOT_WATER_PIN, leds_control);
        digitalWrite(COLD_WATER_PIN, leds_control);
        digitalWrite(DRAIN_PIN, leds_control);

        // Read Actuator States (Feedback)
        current_state.hot_status = digitalRead(HOT_WATER_PIN);
        current_state.cold_status = digitalRead(COLD_WATER_PIN);
        current_state.drain_status = digitalRead(DRAIN_PIN);

        publishToBroker(current_state);
    }
}
