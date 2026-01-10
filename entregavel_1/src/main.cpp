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
constexpr int SIMULATOR_PIN = 15;

constexpr int ADC_RESOLUTION = 4095;

constexpr float THERMISTOR_BETA = 3950;

constexpr float WATER_MIN_MEASUREMENT_CM = 0.0;
constexpr float WATER_MAX_MEASUREMENT_CM = 100.0;

// Hysteresis settings
constexpr float HYSTERESIS_TEMP = 0.5;  // degrees Celsius
constexpr float HYSTERESIS_LEVEL = 2.0; // cm

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

// Logic Constants
constexpr int STATE_LOW = 0;
constexpr int STATE_OPTIMAL = 1;
constexpr int STATE_HIGH = 2;

constexpr int CLOSED = 0;
constexpr int OPEN = 1;

// Group monitored data into a single structure
struct SystemState {
    float temp1;
    float temp2;
    float water_level;
    int hot_status;
    int cold_status;
    int drain_status;
    int system_switch_state;
};

struct SimulationState {
    int active;
    float temp1;
    float temp2;
    float water_level;
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

// Hysteresis State Tracking
int current_temp_state = STATE_OPTIMAL;
int current_level_state = STATE_OPTIMAL;

SimulationState simulation_state = {0, 25.0, 25.0, 0.0};

// Convert analog NTC reading to Celsius
float readTemperature(int pin) {
    if (!simulation_state.active) {
        int analog_value = analogRead(pin);
        // Avoid division by zero if reading is 0 or 4095
        if (analog_value == 0 || analog_value >= ADC_RESOLUTION)
            return -273.15;

        return 1.0 /
            (log(1.0 / (ADC_RESOLUTION / (float)analog_value - 1)) /
                    THERMISTOR_BETA +
                1.0 / 298.15) -
            273.15;
    } else {
        // simulation
        if (pin == TEMPERATURE_PIN_1) {
            return simulation_state.temp1;
        } else {
            // pin == TEMPERATURE_PIN_2
            return simulation_state.temp2;
        }
    }
}

float readWaterLevel(int pin) {
    if (!simulation_state.active) {
        int analog_value = analogRead(pin);

        return WATER_MIN_MEASUREMENT_CM +
            ((float)analog_value / ADC_RESOLUTION) *
            (WATER_MAX_MEASUREMENT_CM - WATER_MIN_MEASUREMENT_CM);
    } else {
        // simulation
        return simulation_state.water_level;
    }
}

// Logic to determine Temperature State with Hysteresis
int getTemperatureState(float current_temp, int previous_state) {
    if (previous_state == STATE_LOW) {
        if (current_temp > config_temp_min + HYSTERESIS_TEMP) {
            return STATE_OPTIMAL;
        }
    } else if (previous_state == STATE_HIGH) {
        if (current_temp < config_temp_max - HYSTERESIS_TEMP) {
            return STATE_OPTIMAL;
        }
    } else { // previous_state == OPTIMAL
        if (current_temp < config_temp_min) {
            return STATE_LOW;
        }
        if (current_temp > config_temp_max)
            return STATE_HIGH;
    }
    return previous_state;
}

// Logic to determine Level State with Hysteresis
int getLevelState(float current_level, int previous_state) {
    if (previous_state == STATE_LOW) {
        if (current_level > config_level_min + HYSTERESIS_LEVEL) {
            return STATE_OPTIMAL;
        }
    } else if (previous_state == STATE_HIGH) {
        if (current_level < config_level_max - HYSTERESIS_LEVEL) {
            return STATE_OPTIMAL;
        }
    } else { // previous_state == OPTIMAL
        if (current_level <= config_level_min) {
            return STATE_LOW;
        }
        if (current_level >= config_level_max) {
            return STATE_HIGH;
        }
    }
    return previous_state;
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
    float avg_temp = (state.temp1 + state.temp2) / 2.0;
    int ret = snprintf(payload, MSG_BUFFER_SIZE,
        "{\"%s\": %.2f, \"%s\": %.2f, \"%s\": %d, \"%s\": %d, \"%s\": %d, "
        "\"%s\": %d}",
        VAR_TEMP_AVG, avg_temp, VAR_LEVEL, state.water_level, VAR_HOT_WATER,
        state.hot_status, VAR_COLD_WATER, state.cold_status, VAR_DRAIN,
        state.drain_status, VAR_SYSTEM_ACTIVE, state.system_switch_state);

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

// This function contains the algorithm for the actuators
void evaluateBusinessRules(SystemState& state) {
    // Safety Rule: If switch is OFF, everything is CLOSED.
    if (state.system_switch_state == LOW) {
        state.hot_status = CLOSED;
        state.cold_status = CLOSED;
        state.drain_status = CLOSED;
        return;
    }

    // Calculate the states taking in account the histheresy
    float avg_temp = (state.temp1 + state.temp2) / 2.0;
    current_temp_state = getTemperatureState(avg_temp, current_temp_state);
    current_level_state = getLevelState(state.water_level, current_level_state);

    // Define Decision Matrices
    const int HOT_MATRIX[3][3] = {
        {OPEN, OPEN, CLOSED},   // Level LOW
        {OPEN, CLOSED, CLOSED}, // Level OPTIMAL
        {OPEN, CLOSED, CLOSED}  // Level HIGH
    };

    const int COLD_MATRIX[3][3] = {
        {CLOSED, OPEN, OPEN},   // Level LOW
        {CLOSED, CLOSED, OPEN}, // Level OPTIMAL
        {CLOSED, CLOSED, OPEN}  // Level HIGH
    };

    const int DRAIN_MATRIX[3][3] = {
        {CLOSED, CLOSED, CLOSED}, // Level LOW
        {CLOSED, CLOSED, CLOSED}, // Level OPTIMAL
        {OPEN, OPEN, OPEN}        // Level HIGH
    };

    // Apply Logic
    state.hot_status = HOT_MATRIX[current_level_state][current_temp_state];
    state.cold_status = COLD_MATRIX[current_level_state][current_temp_state];
    state.drain_status = DRAIN_MATRIX[current_level_state][current_temp_state];
}

// This function updates the controls based on the algorithm defined elsewhere
void updateHardware(const SystemState& state) {
    digitalWrite(HOT_WATER_PIN, state.hot_status);
    digitalWrite(COLD_WATER_PIN, state.cold_status);
    digitalWrite(DRAIN_PIN, state.drain_status);
}

// Updates the physics simulation if active, or syncs state if inactive
void updateSimulation(const SystemState& physical_state) {
    if (simulation_state.active) {
        // Simulation Constants
        constexpr float TEMP_AMBIENT = 25.0;     // Environment temp
        constexpr float TEMP_HOT_SOURCE = 60.0;  // Hot faucet temp
        constexpr float TEMP_COLD_SOURCE = 20.0; // Cold faucet temp

        constexpr float FLOW_RATE_IN = 2.0;    // cm height increase per update
        constexpr float FLOW_RATE_DRAIN = 3.0; // cm height decrease per update

        constexpr float MIXING_FACTOR =
            0.15; // Speed of temp equalization between sensors
        constexpr float COOLING_FACTOR =
            0.02; // Speed of heat loss to environment

        // Calculate mass proxy (Volume per side = Level / 2)
        // We use a small epsilon to avoid division by zero
        float current_mass = max(simulation_state.water_level, 0.1f);

        // --- 1. Water Level Update ---
        float added_level = 0.0;

        if (physical_state.hot_status == OPEN) {
            added_level += FLOW_RATE_IN;
        }
        if (physical_state.cold_status == OPEN) {
            added_level += FLOW_RATE_IN;
        }

        // Apply level changes
        simulation_state.water_level += added_level;

        if (physical_state.drain_status == OPEN) {
            simulation_state.water_level -= FLOW_RATE_DRAIN;
        }

        // Clamp level limits
        if (simulation_state.water_level < 0.0)
            simulation_state.water_level = 0.0;
        if (simulation_state.water_level > 100.0)
            simulation_state.water_level = 100.0;

        // --- 2. Thermal Update (Mass weighted average) ---
        // New Temp = ((CurrentMass * CurrentTemp) + (AddedMass * SourceTemp)) /
        // TotalMass

        // Hot side (Sensor 1) receives Hot water directly
        if (physical_state.hot_status == OPEN) {
            simulation_state.temp1 = ((current_mass * simulation_state.temp1) +
                                         (FLOW_RATE_IN * TEMP_HOT_SOURCE)) /
                (current_mass + FLOW_RATE_IN);
        }

        // Cold side (Sensor 2) receives Cold water directly
        if (physical_state.cold_status == OPEN) {
            simulation_state.temp2 = ((current_mass * simulation_state.temp2) +
                                         (FLOW_RATE_IN * TEMP_COLD_SOURCE)) /
                (current_mass + FLOW_RATE_IN);
        }

        // --- 3. Mixing (Heat exchange between bodies) ---
        // Temperature flows from high to low
        float temp_diff = simulation_state.temp1 - simulation_state.temp2;
        simulation_state.temp1 -= temp_diff * MIXING_FACTOR;
        simulation_state.temp2 += temp_diff * MIXING_FACTOR;

        // --- 4. Ambient Cooling ---
        // Both sides lose heat to environment
        simulation_state.temp1 -=
            (simulation_state.temp1 - TEMP_AMBIENT) * COOLING_FACTOR;
        simulation_state.temp2 -=
            (simulation_state.temp2 - TEMP_AMBIENT) * COOLING_FACTOR;

    } else {
        // Simulation not active: Sync internal state with physical sensors
        // This ensures that if we toggle simulation ON, it starts from current
        // values
        simulation_state.temp1 = physical_state.temp1;
        simulation_state.temp2 = physical_state.temp2;
        simulation_state.water_level = physical_state.water_level;
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

        // Read Inputs (Hardware or Simulation)
        simulation_state.active = digitalRead(SIMULATOR_PIN);

        current_state.temp1 = readTemperature(TEMPERATURE_PIN_1);
        current_state.temp2 = readTemperature(TEMPERATURE_PIN_2);

        current_state.water_level = readWaterLevel(WATER_LEVEL_PIN);
        current_state.system_switch_state = digitalRead(SYSTEM_SWITCH_PIN);

        // Print data in the serial port
        float avg_temp = (current_state.temp1 + current_state.temp2) / 2.0;
        Serial.printf("\r\n");
        Serial.printf(
            "Current temperature: %.2f.C (Temp1 = %.2f, Temp2 = %.2f)\r\n",
            avg_temp, current_state.temp1, current_state.temp2);
        Serial.printf(
            "Current water level: %.2fcm\r\n", current_state.water_level);
        Serial.printf("Temperature control: %.1f.C - %.1f.C\r\n",
            config_temp_min, config_temp_max);
        Serial.printf("      Water control: %.1fcm - %.1fcm\r\n",
            config_level_min, config_level_max);

        evaluateBusinessRules(current_state);
        updateHardware(current_state);
        publishToBroker(current_state);
        updateSimulation(current_state);
    }
}
