// ─────────────────────────────────────────────
//  SafeBeacon — gps_handler.cpp
//  Parses NMEA sentences from SIM7600 GNSS.
//  Shares SIM_SERIAL with connectivity.cpp
//  via AT+CGPS commands.
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <TinyGPS++.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "structs.h"

extern DeviceState        g_state;
extern SemaphoreHandle_t  g_state_mutex;
extern EventGroupHandle_t g_events;

static TinyGPSPlus gps;

// SIM7600 GNSS uses the same UART — we parse
// NMEA output from a secondary UART port on the
// SIM7600 (UART3) or via AT+CGPSINFOCMD polling.
// For prototype: use a dedicated NEO-6M on UART0
// or parse AT+CGPSINF=0 response.

// ── Parse AT+CGPSINF=0 response ──────────────
// Format: <mode>,<lat>,<lon>,<alt>,<utctime>,<ttff>,<num_sat>,<speed>,<course>
static bool parseGPSINF(const String& resp, GpsFix* fix) {
    // Example: "+CGPSINF: 0,2248.931757,N,11406.437730,E,020523174800.0,0,5,1.5,0.0"
    int idx = resp.indexOf("+CGPSINF:");
    if (idx < 0) return false;

    String data = resp.substring(idx + 10);
    // Parse comma-separated fields
    int f = 0;
    String fields[12];
    int start = 0;
    for (int i = 0; i <= data.length() && f < 12; i++) {
        if (i == data.length() || data[i] == ',') {
            fields[f++] = data.substring(start, i);
            start = i + 1;
        }
    }
    if (f < 8) return false;

    int fix_mode = fields[0].toInt();
    if (fix_mode == 0) return false;  // No fix

    // NMEA lat format: DDMM.MMMM → convert to decimal degrees
    float lat_raw = fields[1].toFloat();
    int lat_deg = (int)(lat_raw / 100);
    float lat_min = lat_raw - lat_deg * 100;
    fix->lat = lat_deg + lat_min / 60.0;
    if (fields[2] == "S") fix->lat = -fix->lat;

    float lon_raw = fields[3].toFloat();
    int lon_deg = (int)(lon_raw / 100);
    float lon_min = lon_raw - lon_deg * 100;
    fix->lon = lon_deg + lon_min / 60.0;
    if (fields[4] == "W") fix->lon = -fix->lon;

    fix->speed_kmh   = fields[7].toFloat() * 1.852f;   // knots → km/h
    fix->satellites  = (uint8_t)fields[6].toInt();
    fix->valid       = true;
    fix->timestamp   = (uint32_t)(millis() / 1000);

    return true;
}

void taskGpsHandler(void* pvParams) {
    Serial.println("[GPS] Task started — polling SIM7600 GNSS");
    vTaskDelay(pdMS_TO_TICKS(5000));   // Wait for connectivity task to init SIM

    // Enable GNSS via AT command (sent to connectivity's serial)
    // In production, GPS task sends AT commands on a separate serial
    // For this prototype: shared via semaphore on SIM_SERIAL

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // In full implementation: request AT+CGPSINF=0 via SIM_SERIAL
        // Here we simulate by updating with last known good fix
        // Replace with actual UART read + parseGPSINF in production

        GpsFix fix = {};

        // Placeholder — real implementation reads SIM7600 NMEA output
        // on secondary UART or AT+CGPSINF polling
        fix.lat        = 30.7333;   // Chandigarh default (replace with live)
        fix.lon        = 76.7794;
        fix.speed_kmh  = 0.0f;
        fix.satellites = 6;
        fix.valid      = true;
        fix.timestamp  = (uint32_t)(millis() / 1000);

        if (fix.valid) {
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_state.last_gps = fix;
                xSemaphoreGive(g_state_mutex);
            }
            xEventGroupSetBits(g_events, EVT_GPS_VALID);

            if (DEBUG_SERIAL) {
                Serial.printf("[GPS] Lat:%.6f Lon:%.6f Speed:%.1f km/h Sats:%d\n",
                              fix.lat, fix.lon, fix.speed_kmh, fix.satellites);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GPS_POLL_MS));
    }
}
