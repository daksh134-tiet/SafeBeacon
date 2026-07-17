// ─────────────────────────────────────────────
//  SafeBeacon — obd_reader.cpp
//  Non-blocking OBD2 PID state machine.
//  Tested on Honda Brio 2013 (ISO 15765-4 CAN)
//  Uses ELM327 UART at 38400 baud.
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "structs.h"

extern DeviceState       g_state;
extern SemaphoreHandle_t g_state_mutex;
extern EventGroupHandle_t g_events;

// ELM327 UART instance
static HardwareSerial OBD_SERIAL(1);

// ── ELM327 init sequence ──────────────────────
static const char* ELM_INIT_CMDS[] = {
    "ATZ",      // Reset
    "ATE0",     // Echo off
    "ATL0",     // Linefeed off
    "ATS0",     // Spaces off
    "ATH0",     // Headers off
    "ATAL",     // Allow long messages
    "ATSP0",    // Auto protocol detect
    "ATAT2",    // Adaptive timing mode 2
    nullptr
};

// ── Send AT command, wait for response ────────
static String elmSend(const char* cmd, uint32_t timeout_ms = 1000) {
    OBD_SERIAL.flush();
    while (OBD_SERIAL.available()) OBD_SERIAL.read();  // Clear buffer

    OBD_SERIAL.print(cmd);
    OBD_SERIAL.print('\r');

    String resp = "";
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (OBD_SERIAL.available()) {
            char c = OBD_SERIAL.read();
            if (c == '>') return resp;   // Prompt = done
            if (c != '\r' && c != '\n') resp += c;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return resp;
}

// ── Query a single OBD2 PID ───────────────────
// Returns the A/B byte values, sets valid=false on NODATA
static bool queryPID(uint8_t mode, uint8_t pid, uint8_t* a, uint8_t* b = nullptr) {
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "%02X%02X", mode, pid);

    String resp = elmSend(cmd, 500);

    if (resp.length() == 0 || resp.indexOf("NODATA") >= 0 ||
        resp.indexOf("ERROR") >= 0) {
        return false;
    }

    // Parse hex response: "41 0C 1A F8" → skip mode+PID, read A (and B)
    // With ATS0+ATH0, response is just hex bytes concatenated
    // Expected format: 4 hex chars minimum for 1-byte PIDs
    if (resp.length() < 4) return false;

    // Find response header (mode + 0x40 response marker)
    // e.g. mode 01 → response starts with "41"
    int idx = resp.indexOf(String(mode + 0x40, HEX));
    if (idx < 0) idx = 0;

    // Skip "41" + PID byte (4 hex chars)
    idx += 4;
    if (idx + 2 > (int)resp.length()) return false;

    String byte_a = resp.substring(idx, idx + 2);
    *a = (uint8_t)strtol(byte_a.c_str(), nullptr, 16);

    if (b != nullptr && idx + 4 <= (int)resp.length()) {
        String byte_b = resp.substring(idx + 2, idx + 4);
        *b = (uint8_t)strtol(byte_b.c_str(), nullptr, 16);
    }
    return true;
}

// ── Parse full OBD snapshot ───────────────────
static ObdSnapshot readAllPIDs() {
    ObdSnapshot snap = {};
    snap.timestamp = (uint32_t)(millis() / 1000);
    uint8_t a = 0, b = 0;

    // RPM — PID 0x0C, formula: ((A*256)+B)/4
    if (queryPID(OBD_MODE_LIVE, PID_ENGINE_RPM, &a, &b))
        snap.rpm = (uint16_t)(((a * 256) + b) / 4);

    // Speed — PID 0x0D, formula: A km/h
    if (queryPID(OBD_MODE_LIVE, PID_VEHICLE_SPEED, &a))
        snap.speed_kmh = a;

    // Coolant temp — PID 0x05, formula: A - 40°C
    if (queryPID(OBD_MODE_LIVE, PID_COOLANT_TEMP, &a))
        snap.coolant_temp_c = (int8_t)(a - 40);

    // Throttle position — PID 0x11, formula: A*100/255 %
    if (queryPID(OBD_MODE_LIVE, PID_THROTTLE_POS, &a))
        snap.throttle_pct = (uint8_t)(a * 100 / 255);

    // Fuel level — PID 0x2F, formula: A*100/255 %
    if (queryPID(OBD_MODE_LIVE, PID_FUEL_LEVEL, &a))
        snap.fuel_level_pct = (uint8_t)(a * 100 / 255);

    // Control module voltage — PID 0x42, formula: ((A*256)+B)/1000
    if (queryPID(OBD_MODE_LIVE, PID_CTRL_MODULE_V, &a, &b))
        snap.ctrl_module_v = ((a * 256) + b) / 1000.0f;

    // Intake air temp — PID 0x0F, formula: A - 40°C
    if (queryPID(OBD_MODE_LIVE, PID_INTAKE_TEMP, &a))
        snap.intake_temp_c = (int8_t)(a - 40);

    // Intake manifold pressure — PID 0x0B, formula: A kPa
    if (queryPID(OBD_MODE_LIVE, PID_INTAKE_MAP, &a))
        snap.intake_map_kpa = a;

    // MIL + DTC count — PID 0x01
    if (queryPID(OBD_MODE_LIVE, PID_MIL_DTC, &a, &b)) {
        snap.mil_on   = (a & 0x80) != 0;
        snap.dtc_count = a & 0x7F;
    }

    // Oil temp — PID 0x5C
    if (queryPID(OBD_MODE_LIVE, PID_OIL_TEMP, &a))
        snap.oil_temp_c = (int8_t)(a - 40);

    snap.valid = (snap.rpm > 0 || snap.speed_kmh > 0);
    return snap;
}

// ── Retrieve and log DTCs ─────────────────────
static void readDTCs() {
    String resp = elmSend("03", 2000);   // Mode 03 = request DTCs
    if (resp.length() == 0 || resp.indexOf("NO") >= 0) {
        Serial.println("[OBD] No active DTCs");
        return;
    }
    Serial.println("[OBD] Active DTCs: " + resp);
    // TODO: decode DTC bytes to P/C/B/U codes and publish
}

// ── Honda Brio PID discovery (run once) ───────
static void discoverSupportedPIDs() {
    Serial.println("[OBD] Scanning supported PIDs...");
    uint8_t a, b, c, d;

    // PID 0x00 returns a bitmask of PIDs 01-20 supported
    String resp = elmSend("0100", 1000);
    Serial.println("[OBD] PID 00-20 support: " + resp);

    // PID 0x20 returns PIDs 21-40 support
    resp = elmSend("0120", 1000);
    Serial.println("[OBD] PID 21-40 support: " + resp);

    resp = elmSend("0140", 1000);
    Serial.println("[OBD] PID 41-60 support: " + resp);
}

// ── Driving behaviour analysis ────────────────
struct DriveMetrics {
    uint16_t harsh_brakes;
    uint16_t harsh_accels;
    uint16_t overspeed_count;
    float    idle_minutes;
};

static DriveMetrics trip_metrics = {};

static void analyseOBD(const ObdSnapshot& prev, const ObdSnapshot& curr) {
    if (!prev.valid || !curr.valid) return;

    float dt_s = 5.0f;  // Poll interval

    // Speed derivative → harsh braking / acceleration
    float dv = (float)curr.speed_kmh - (float)prev.speed_kmh;
    float accel_ms2 = (dv / 3.6f) / dt_s;

    if (accel_ms2 < -3.5f) {
        trip_metrics.harsh_brakes++;
        Serial.printf("[DRIVE] Harsh brake: %.1f m/s²\n", accel_ms2);
    } else if (accel_ms2 > 2.5f) {
        trip_metrics.harsh_accels++;
        Serial.printf("[DRIVE] Harsh accel: %.1f m/s²\n", accel_ms2);
    }

    // Overspeed (>80 km/h on city roads as default)
    if (curr.speed_kmh > 80) trip_metrics.overspeed_count++;

    // Idle detection: engine on, speed = 0
    if (curr.rpm > 500 && curr.speed_kmh == 0)
        trip_metrics.idle_minutes += (dt_s / 60.0f);

    // Coolant overheat warning
    if (curr.coolant_temp_c > 100) {
        Serial.printf("[OBD] ⚠ Coolant overheating: %d°C\n", curr.coolant_temp_c);
    }

    // Battery warning
    if (curr.ctrl_module_v < VBAT_WARN_V) {
        Serial.printf("[OBD] ⚠ Low battery voltage: %.2fV\n", curr.ctrl_module_v);
    }
}

// ── Driver safety score (0-100) ───────────────
static float computeSafetyScore(const DriveMetrics& m, uint32_t trip_minutes) {
    if (trip_minutes == 0) return 100.0f;
    float score = 100.0f;
    float normalised_brakes = (float)m.harsh_brakes / trip_minutes * 10.0f;
    float normalised_accels = (float)m.harsh_accels / trip_minutes * 10.0f;
    score -= min(normalised_brakes * 5.0f, 30.0f);
    score -= min(normalised_accels * 3.0f, 20.0f);
    score -= min((float)m.overspeed_count / trip_minutes * 2.0f, 20.0f);
    return max(0.0f, score);
}

// ── OBD Reader Task ───────────────────────────
void taskObdReader(void* pvParams) {
    Serial.println("[OBD] Task started");

    OBD_SERIAL.begin(OBD_BAUD, SERIAL_8N1, OBD_RX_PIN, OBD_TX_PIN);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ELM327 init
    bool elm_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        String resp = elmSend("ATZ", 2000);
        if (resp.indexOf("ELM") >= 0 || resp.length() > 2) {
            elm_ok = true;
            Serial.println("[OBD] ELM327 found: " + resp);
            break;
        }
        Serial.printf("[OBD] ELM327 init attempt %d failed\n", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (!elm_ok) {
        Serial.println("[OBD] ELM327 not found — OBD disabled");
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.obd_available = false;
            xSemaphoreGive(g_state_mutex);
        }
        vTaskDelete(nullptr);
        return;
    }

    // Send init sequence
    for (int i = 0; ELM_INIT_CMDS[i] != nullptr; i++) {
        String r = elmSend(ELM_INIT_CMDS[i], 1000);
        if (DEBUG_OBD) Serial.printf("[OBD] %s → %s\n", ELM_INIT_CMDS[i], r.c_str());
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    discoverSupportedPIDs();

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.obd_available = true;
        xSemaphoreGive(g_state_mutex);
    }
    xEventGroupSetBits(g_events, EVT_OBD_CONN);
    Serial.println("[OBD] Ready — polling every 5s");

    ObdSnapshot prev_snap = {};
    uint32_t trip_start_ms = millis();

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        ObdSnapshot snap = readAllPIDs();

        if (snap.valid) {
            analyseOBD(prev_snap, snap);

            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_state.last_obd = snap;

                // Update trip metrics
                g_state.current_trip.harsh_brake_count = trip_metrics.harsh_brakes;
                g_state.current_trip.harsh_accel_count = trip_metrics.harsh_accels;
                g_state.current_trip.idle_seconds = (uint16_t)(trip_metrics.idle_minutes * 60);
                if (snap.speed_kmh > g_state.current_trip.max_speed_kmh)
                    g_state.current_trip.max_speed_kmh = snap.speed_kmh;
                if (snap.rpm > g_state.current_trip.max_rpm)
                    g_state.current_trip.max_rpm = snap.rpm;

                uint32_t trip_min = (millis() - trip_start_ms) / 60000;
                g_state.current_trip.safety_score = computeSafetyScore(trip_metrics, trip_min);

                xSemaphoreGive(g_state_mutex);
            }

            if (DEBUG_SERIAL) {
                Serial.printf("[OBD] RPM:%u Spd:%u Clnt:%d° Fuel:%u%% V:%.2fV MIL:%s\n",
                              snap.rpm, snap.speed_kmh, snap.coolant_temp_c,
                              snap.fuel_level_pct, snap.ctrl_module_v,
                              snap.mil_on ? "ON" : "off");
            }

            prev_snap = snap;
        } else {
            if (DEBUG_SERIAL) Serial.println("[OBD] No data this cycle");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OBD_POLL_MS));
    }
}
