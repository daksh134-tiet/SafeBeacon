// ─────────────────────────────────────────────
//  SafeBeacon — power_manager.cpp
//  Monitors 12V vehicle battery via ADC divider.
//  Manages LiPo backup + sleep/wake transitions.
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "structs.h"

extern DeviceState        g_state;
extern SemaphoreHandle_t  g_state_mutex;
extern EventGroupHandle_t g_events;

// ── ADC to voltage conversion ─────────────────
// Voltage divider: R1=100kΩ, R2=10kΩ
// V_adc = V_batt * R2 / (R1 + R2) = V_batt * 0.0909
// V_batt = V_adc * (R1 + R2) / R2 = V_adc * 11
static float readBatteryVoltage() {
    // Average 16 samples to reduce ADC noise
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(VBAT_ADC_PIN);
        delay(2);
    }
    float adc_avg   = sum / 16.0f;
    float v_adc     = (adc_avg / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float v_batt    = v_adc * ((VBAT_R1 + VBAT_R2) / VBAT_R2);
    return v_batt;
}

void taskPowerManager(void* pvParams) {
    Serial.println("[PWR] Task started");

    BatteryState bat = {};
    uint8_t warn_debounce = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        float v = readBatteryVoltage();

        // Ignore cranking transients (voltage dips below 9V during start)
        if (v < VBAT_CRANK_V) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(VBAT_POLL_MS));
            continue;
        }

        bat.voltage_v = v;
        bat.charging  = (v >= VBAT_CHARGING_V);
        bat.warn      = (v < VBAT_WARN_V && !bat.charging);
        bat.critical  = (v < VBAT_CRITICAL_V);
        bat.timestamp = (uint32_t)(millis() / 1000);

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_state.battery = bat;
            xSemaphoreGive(g_state_mutex);
        }

        if (bat.critical) {
            warn_debounce++;
            if (warn_debounce >= 3) {   // 3 consecutive readings
                Serial.printf("[PWR] ⚠ CRITICAL battery: %.2fV — running on LiPo backup\n", v);
                warn_debounce = 0;
            }
        } else if (bat.warn) {
            Serial.printf("[PWR] Battery warning: %.2fV\n", v);
        } else {
            warn_debounce = 0;
        }

        if (DEBUG_SERIAL && !bat.warn) {
            Serial.printf("[PWR] Battery: %.2fV %s\n", v,
                          bat.charging ? "(charging)" : "(discharging)");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(VBAT_POLL_MS));
    }
}


// ─────────────────────────────────────────────
//  SafeBeacon — ui.cpp (appended to same file)
//  LED status indication + buzzer feedback.
// ─────────────────────────────────────────────

extern DeviceState        g_state;
extern EventGroupHandle_t g_events;

#define EVT_GPS_VALID    (1 << 0)
#define EVT_OBD_CONN     (1 << 1)
#define EVT_MQTT_CONN    (1 << 2)
#define EVT_SOS_ARMED    (1 << 3)
#define EVT_SOS_ACTIVE   (1 << 4)

static void ledSet(bool r, bool g, bool b) {
    digitalWrite(LED_RED_PIN,   r ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
    digitalWrite(LED_BLUE_PIN,  b ? HIGH : LOW);
}

static void buzz(uint16_t ms) {
    if (BUZZER_PIN <= 0) return;
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(ms));
    digitalWrite(BUZZER_PIN, LOW);
}

void taskUI(void* pvParams) {
    Serial.println("[UI] Task started");

    uint32_t blink_ms = 0;
    bool     led_state = false;

    for (;;) {
        EventBits_t bits = xEventGroupGetBits(g_events);

        // ── SOS Active — rapid red flash + buzzer ──
        if (bits & EVT_SOS_ACTIVE) {
            ledSet(true, false, false);
            vTaskDelay(pdMS_TO_TICKS(100));
            ledSet(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(100));
            buzz(50);
            continue;
        }

        // ── SOS Armed — countdown — orange flash ──
        if (bits & EVT_SOS_ARMED) {
            ledSet(true, false, false);
            vTaskDelay(pdMS_TO_TICKS(200));
            ledSet(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(200));
            buzz(100);
            vTaskDelay(pdMS_TO_TICKS(400));
            continue;
        }

        // ── Normal operation state indicator ──
        bool gps  = bits & EVT_GPS_VALID;
        bool obd  = bits & EVT_OBD_CONN;
        bool mqtt = bits & EVT_MQTT_CONN;

        // Check battery warning
        bool bat_warn = false;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            bat_warn = g_state.battery.warn || g_state.battery.critical;
            xSemaphoreGive(g_state_mutex);
        }

        if (bat_warn) {
            // Amber blink: red+green = yellow
            led_state = !led_state;
            ledSet(led_state, led_state, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (mqtt && gps && obd) {
            // All good: slow green pulse
            ledSet(false, true, false);
            vTaskDelay(pdMS_TO_TICKS(2000));
            ledSet(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (mqtt && gps) {
            // GPS + cloud, no OBD: cyan slow pulse
            ledSet(false, true, true);
            vTaskDelay(pdMS_TO_TICKS(2000));
            ledSet(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (gps) {
            // GPS only, no cloud: blue blink
            ledSet(false, false, true);
            vTaskDelay(pdMS_TO_TICKS(500));
            ledSet(false, false, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // No fix, searching: white rapid blink
            led_state = !led_state;
            ledSet(led_state, led_state, led_state);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
}
