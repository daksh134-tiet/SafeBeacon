// ─────────────────────────────────────────────
//  SafeBeacon — main.cpp
//  Entry point. Initialises hardware and spawns
//  FreeRTOS tasks for parallel operation.
//
//  Target: ESP32-WROOM-32
//  Vehicle: Honda Brio 2013
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"
#include "structs.h"

// ── Task forward declarations ─────────────────
void taskCrashDetector(void* pvParams);
void taskObdReader(void* pvParams);
void taskGpsHandler(void* pvParams);
void taskConnectivity(void* pvParams);
void taskPowerManager(void* pvParams);
void taskUI(void* pvParams);

// ── Global shared state ───────────────────────
DeviceState g_state;
SemaphoreHandle_t g_state_mutex;

// ── Task handles ──────────────────────────────
TaskHandle_t h_crash    = nullptr;
TaskHandle_t h_obd      = nullptr;
TaskHandle_t h_gps      = nullptr;
TaskHandle_t h_connect  = nullptr;
TaskHandle_t h_power    = nullptr;
TaskHandle_t h_ui       = nullptr;

// ── Event group bits ──────────────────────────
// Bit 0: GPS fix valid
// Bit 1: OBD connected
// Bit 2: MQTT connected
// Bit 3: SOS armed (crash detected, awaiting confirm)
// Bit 4: SOS active (transmitting)
EventGroupHandle_t g_events;
#define EVT_GPS_VALID    (1 << 0)
#define EVT_OBD_CONN     (1 << 1)
#define EVT_MQTT_CONN    (1 << 2)
#define EVT_SOS_ARMED    (1 << 3)
#define EVT_SOS_ACTIVE   (1 << 4)

// ── Queue: crash events ───────────────────────
QueueHandle_t q_crash_events;

// ── LED helpers ───────────────────────────────
inline void ledSet(bool r, bool g, bool b) {
    digitalWrite(LED_RED_PIN,   r ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
    digitalWrite(LED_BLUE_PIN,  b ? HIGH : LOW);
}

// ── Hardware init ─────────────────────────────
void initPins() {
    pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_RED_PIN,    OUTPUT);
    pinMode(LED_GREEN_PIN,  OUTPUT);
    pinMode(LED_BLUE_PIN,   OUTPUT);
    if (BUZZER_PIN > 0) pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    pinMode(SIM_RST_PIN,    OUTPUT);
    analogSetAttenuation(ADC_11db);   // For 12V divider on ADC
    ledSet(false, false, false);
}

void initI2C() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    if (DEBUG_SERIAL) Serial.println("[INIT] I2C started at 400kHz");
}

void blinkStartup() {
    // Blue blink 3x on boot
    for (int i = 0; i < 3; i++) {
        ledSet(false, false, true);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledSet(false, false, false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

// ── setup() ──────────────────────────────────
void setup() {
    if (DEBUG_SERIAL) {
        Serial.begin(DEBUG_BAUD);
        delay(500);
        Serial.println("\n\n╔══════════════════════════════╗");
        Serial.println("║     SafeBeacon v" FIRMWARE_VERSION "        ║");
        Serial.println("║     Device: " DEVICE_ID "          ║");
        Serial.println("╚══════════════════════════════╝\n");
    }

    initPins();
    initI2C();
    blinkStartup();

    // Initialise shared state
    memset(&g_state, 0, sizeof(DeviceState));
    g_state_mutex  = xSemaphoreCreateMutex();
    g_events       = xEventGroupCreate();
    q_crash_events = xQueueCreate(5, sizeof(CrashEvent));

    configASSERT(g_state_mutex != nullptr);
    configASSERT(g_events      != nullptr);
    configASSERT(q_crash_events != nullptr);

    // ── Spawn FreeRTOS tasks ──
    // CRASH DETECTOR — highest priority, smallest stack
    xTaskCreatePinnedToCore(
        taskCrashDetector, "CrashDet",
        4096, nullptr, PRIORITY_CRASH, &h_crash, 1  // Core 1
    );

    // GPS HANDLER — time-sensitive
    xTaskCreatePinnedToCore(
        taskGpsHandler, "GpsHndlr",
        6144, nullptr, PRIORITY_GPS, &h_gps, 1      // Core 1
    );

    // OBD READER — I/O heavy
    xTaskCreatePinnedToCore(
        taskObdReader, "ObdRdr",
        8192, nullptr, PRIORITY_OBD, &h_obd, 0      // Core 0
    );

    // CONNECTIVITY — MQTT + AT commands
    xTaskCreatePinnedToCore(
        taskConnectivity, "Connect",
        12288, nullptr, PRIORITY_PUBLISH, &h_connect, 0  // Core 0
    );

    // POWER MANAGER
    xTaskCreatePinnedToCore(
        taskPowerManager, "PwrMgr",
        4096, nullptr, PRIORITY_UI, &h_power, 0
    );

    // UI (LED + buzzer feedback)
    xTaskCreatePinnedToCore(
        taskUI, "UI",
        2048, nullptr, PRIORITY_UI, &h_ui, 0
    );

    if (DEBUG_SERIAL) {
        Serial.printf("[INIT] All tasks spawned. Free heap: %u bytes\n",
                      esp_get_free_heap_size());
    }
}

// ── loop() is unused — FreeRTOS takes over ────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
