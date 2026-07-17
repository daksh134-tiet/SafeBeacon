// ─────────────────────────────────────────────
//  SafeBeacon — connectivity.cpp
//  MQTT telemetry publishing + SOS voice call
//  via SIM7600EI AT command interface.
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "structs.h"

extern DeviceState        g_state;
extern SemaphoreHandle_t  g_state_mutex;
extern EventGroupHandle_t g_events;
extern QueueHandle_t      q_crash_events;

static HardwareSerial SIM_SERIAL(2);

// ── SIM7600 AT command helper ─────────────────
static String simSend(const char* cmd, uint32_t timeout_ms = 3000,
                      const char* expect = "OK") {
    SIM_SERIAL.flush();
    while (SIM_SERIAL.available()) SIM_SERIAL.read();
    SIM_SERIAL.print(cmd);
    SIM_SERIAL.print('\r');

    String resp;
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (SIM_SERIAL.available()) {
            resp += (char)SIM_SERIAL.read();
        }
        if (expect && resp.indexOf(expect) >= 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (DEBUG_OBD) Serial.printf("[SIM] %s → %s\n", cmd, resp.c_str());
    return resp;
}

// ── Power on SIM7600 ──────────────────────────
static bool powerOnSIM7600() {
    Serial.println("[SIM] Powering on SIM7600...");
    digitalWrite(SIM_RST_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(SIM_RST_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(300));

    digitalWrite(SIM_PWRKEY_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(SIM_PWRKEY_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));   // Boot time

    // Test AT
    for (int i = 0; i < 5; i++) {
        String r = simSend("AT", 1000);
        if (r.indexOf("OK") >= 0) {
            Serial.println("[SIM] SIM7600 responding");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

// ── Network registration ──────────────────────
static bool registerNetwork() {
    simSend("ATE0", 1000);
    simSend("AT+CMEE=2", 1000);   // Verbose errors

    // Check SIM
    String sim = simSend("AT+CIMI", 2000);
    if (sim.indexOf("ERROR") >= 0) {
        Serial.println("[SIM] No SIM or SIM error");
        return false;
    }

    // Wait for network registration
    for (int i = 0; i < 30; i++) {
        String reg = simSend("AT+CREG?", 1000);
        if (reg.indexOf(",1") >= 0 || reg.indexOf(",5") >= 0) {
            Serial.println("[SIM] Network registered");
            break;
        }
        Serial.printf("[SIM] Waiting for network... (%d/30)\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Set APN
    char apn_cmd[64];
    snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
    simSend(apn_cmd, 1000);

    // Activate PDP context
    simSend("AT+CGACT=1,1", 5000, "OK");

    // Verify IP
    String ip = simSend("AT+CGPADDR=1", 2000);
    Serial.println("[SIM] IP: " + ip);

    return ip.indexOf("+CGPADDR") >= 0;
}

// ── MQTT over SIM7600 (AT+CMQTT) ─────────────
static bool mqttConnect() {
    simSend("AT+CMQTTSTART", 3000, "OK");
    vTaskDelay(pdMS_TO_TICKS(500));

    char accq_cmd[64];
    snprintf(accq_cmd, sizeof(accq_cmd),
             "AT+CMQTTACCQ=0,\"%s\",1", DEVICE_ID);
    simSend(accq_cmd, 2000, "OK");

    // Connect to broker
    char conn_cmd[128];
    snprintf(conn_cmd, sizeof(conn_cmd),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1,\"%s\",\"%s\"",
             MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS);
    String r = simSend(conn_cmd, 10000, "+CMQTTCONNECT: 0,0");

    if (r.indexOf("+CMQTTCONNECT: 0,0") >= 0) {
        Serial.println("[MQTT] Connected");
        xEventGroupSetBits(g_events, EVT_MQTT_CONN);
        return true;
    }
    Serial.println("[MQTT] Connection failed: " + r);
    return false;
}

static bool mqttPublish(const char* topic, const char* payload, uint8_t qos = 0) {
    char topic_cmd[128];
    snprintf(topic_cmd, sizeof(topic_cmd),
             "AT+CMQTTTOPIC=0,%d", strlen(topic));
    simSend(topic_cmd, 1000, ">");
    SIM_SERIAL.print(topic);
    vTaskDelay(pdMS_TO_TICKS(200));

    char payload_cmd[32];
    snprintf(payload_cmd, sizeof(payload_cmd),
             "AT+CMQTTPAYLOAD=0,%d", strlen(payload));
    simSend(payload_cmd, 1000, ">");
    SIM_SERIAL.print(payload);
    vTaskDelay(pdMS_TO_TICKS(200));

    char pub_cmd[64];
    snprintf(pub_cmd, sizeof(pub_cmd),
             "AT+CMQTTPUB=0,%d,60,0", qos);
    String r = simSend(pub_cmd, 5000, "+CMQTTPUB: 0,0");
    return r.indexOf("+CMQTTPUB: 0,0") >= 0;
}

// ── Emergency voice call (GUARD) ─────────────
static void makeEmergencyCall(const CrashEvent& evt) {
    Serial.println("[SIM] ☎  Initiating GUARD emergency call...");

    // Enable audio
    simSend("AT+CPCMREG=1", 1000);

    char dial_cmd[32];
    snprintf(dial_cmd, sizeof(dial_cmd), "ATD%s;", EMERGENCY_NUMBER);
    simSend(dial_cmd, 5000, "OK");

    // Keep call active for 30 seconds minimum
    vTaskDelay(pdMS_TO_TICKS(30000));
    simSend("ATH", 2000);    // Hang up
    Serial.println("[SIM] Call ended");
}

// ── Build telemetry JSON ──────────────────────
static String buildTelemetryJSON(const DeviceState& state) {
    StaticJsonDocument<512> doc;
    doc["id"]  = DEVICE_ID;
    doc["ts"]  = state.last_gps.timestamp;
    doc["lat"] = state.last_gps.lat;
    doc["lon"] = state.last_gps.lon;
    doc["spd"] = state.last_obd.valid
                 ? state.last_obd.speed_kmh
                 : (uint8_t)state.last_gps.speed_kmh;
    doc["rpm"] = state.last_obd.rpm;
    doc["ect"] = state.last_obd.coolant_temp_c;
    doc["thr"] = state.last_obd.throttle_pct;
    doc["fuel"]= state.last_obd.fuel_level_pct;
    doc["vbat"]= state.last_obd.ctrl_module_v;
    doc["mil"] = state.last_obd.mil_on;
    doc["dtc"] = state.last_obd.dtc_count;
    doc["g"]   = state.last_imu.g_total;
    doc["sat"] = state.last_gps.satellites;
    doc["rssi"]= (int8_t)-70;  // TODO: AT+CSQ parse
    doc["score"] = state.current_trip.safety_score;

    String out;
    serializeJson(doc, out);
    return out;
}

// ── Build SOS JSON ────────────────────────────
static String buildSOSJSON(const CrashEvent& evt) {
    StaticJsonDocument<1024> doc;
    doc["id"]       = evt.device_id;
    doc["ts"]       = evt.timestamp;
    doc["lat"]      = evt.location.lat;
    doc["lon"]      = evt.location.lon;
    doc["peak_g"]   = evt.peak_g;
    doc["speed"]    = evt.speed_at_impact_kmh;
    doc["delta_v"]  = evt.speed_delta_kmh;
    doc["severity"] = (uint8_t)evt.severity;
    doc["sos_sent"] = evt.sos_sent;
    doc["cancelled"]= evt.cancelled_by_driver;
    doc["fw"]       = FIRMWARE_VERSION;

    // Pre-crash velocity trace
    JsonArray trace = doc.createNestedArray("trace");
    // Note: in full impl, populate from SosPacket.velocity_trace
    // Simplified here — add GPS speed samples
    doc["trace_note"] = "30s pre-crash velocity buffer";

    String out;
    serializeJson(doc, out);
    return out;
}

// ── Connectivity Task ─────────────────────────
void taskConnectivity(void* pvParams) {
    Serial.println("[CONN] Task started");

    SIM_SERIAL.begin(SIM7600_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool sim_ok = powerOnSIM7600();
    if (!sim_ok) {
        Serial.println("[CONN] SIM7600 power-on failed — connectivity disabled");
        vTaskDelete(nullptr);
        return;
    }

    bool net_ok = registerNetwork();
    if (!net_ok) {
        Serial.println("[CONN] Network registration failed");
        // Continue anyway — will retry in loop
    }

    bool mqtt_ok = false;
    if (net_ok) mqtt_ok = mqttConnect();

    TickType_t last_tele = xTaskGetTickCount();
    TickType_t last_hb   = xTaskGetTickCount();

    for (;;) {
        // ── Handle incoming SOS events (highest priority) ──
        CrashEvent evt;
        if (xQueueReceive(q_crash_events, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.println("[CONN] SOS event received — transmitting");

            if (!mqtt_ok) {
                // Try reconnect
                if (net_ok || registerNetwork()) mqtt_ok = mqttConnect();
            }

            if (mqtt_ok) {
                String sos_json = buildSOSJSON(evt);
                bool sent = mqttPublish(MQTT_TOPIC_SOS, sos_json.c_str(), MQTT_QOS_SOS);
                Serial.printf("[CONN] SOS MQTT publish: %s\n", sent ? "OK" : "FAILED");
            }

            // Always attempt voice call for severe crashes
            if (evt.severity >= CrashSeverity::SEVERE) {
                makeEmergencyCall(evt);
            }
        }

        // ── Periodic telemetry publish ──
        if (xTaskGetTickCount() - last_tele >= pdMS_TO_TICKS(TELE_PUBLISH_MS)) {
            if (mqtt_ok) {
                DeviceState snap;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    snap = g_state;
                    xSemaphoreGive(g_state_mutex);
                }
                String tele = buildTelemetryJSON(snap);
                mqttPublish(MQTT_TOPIC_TELE, tele.c_str(), MQTT_QOS_TELE);
                if (DEBUG_SERIAL) Serial.println("[CONN] Telemetry sent: " + tele);
            }
            last_tele = xTaskGetTickCount();
        }

        // ── Hourly MQTT keepalive + reconnect check ──
        if (xTaskGetTickCount() - last_hb >= pdMS_TO_TICKS(HEARTBEAT_MS)) {
            String ping = simSend("AT", 500);
            if (ping.indexOf("OK") < 0) {
                Serial.println("[CONN] SIM7600 not responding — reinit");
                mqtt_ok = false;
                net_ok  = false;
                powerOnSIM7600();
                net_ok  = registerNetwork();
                if (net_ok) mqtt_ok = mqttConnect();
            }
            last_hb = xTaskGetTickCount();
        }
    }
}
