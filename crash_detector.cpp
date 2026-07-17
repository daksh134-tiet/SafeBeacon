// ─────────────────────────────────────────────
//  SafeBeacon — crash_detector.cpp
//  Three-stage crash detection algorithm
//  using MPU-6050 IMU at 20Hz polling.
//
//  Stage 1: Raw G-force threshold breach
//  Stage 2: Confirmation via speed delta + motion
//  Stage 3: 15-second driver override window
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "config.h"
#include "structs.h"

// External references from main.cpp
extern DeviceState      g_state;
extern SemaphoreHandle_t g_state_mutex;
extern EventGroupHandle_t g_events;
extern QueueHandle_t     q_crash_events;

// ── MPU-6050 Registers ────────────────────────
#define MPU_PWR_MGMT_1    0x6B
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_WHO_AM_I      0x75

// Accelerometer scale: ±8G for crash detection
// Raw LSB/G at ±8G = 4096
#define ACCEL_SCALE_G     4096.0f
// Gyro scale: ±500°/s
#define GYRO_SCALE_DPS    65.5f

// ── Pre-crash ring buffer ─────────────────────
#define PRE_CRASH_SLOTS   60    // 30s at 20Hz with decimation to 2Hz
static PreCrashEntry pre_crash_buf[PRE_CRASH_SLOTS];
static uint8_t  buf_head  = 0;
static uint8_t  buf_count = 0;

// ── State machine ─────────────────────────────
enum class CrashState {
    IDLE,
    TRIGGERED,      // Stage 1 passed, awaiting confirmation
    CONFIRMING,     // Stage 2 — evaluating
    ARMED,          // Stage 3 — countdown, awaiting driver override
    SOS_SENT
};

static CrashState crash_state     = CrashState::IDLE;
static uint32_t   trigger_time_ms = 0;
static ImuSample  peak_sample;

// ── MPU-6050 helpers ──────────────────────────
static bool mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool mpu_read_block(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
    return true;
}

static bool initMPU6050() {
    // Verify device
    uint8_t who;
    mpu_write(MPU_PWR_MGMT_1, 0x00);   // Wake up
    delay(100);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
    who = Wire.read();
    if (who != 0x68 && who != 0x72) {
        Serial.printf("[IMU] WHO_AM_I mismatch: 0x%02X (expected 0x68)\n", who);
        return false;
    }

    mpu_write(MPU_SMPLRT_DIV,   0x13);  // 20Hz sample rate
    mpu_write(MPU_CONFIG,       0x03);  // DLPF 44Hz bandwidth
    mpu_write(MPU_ACCEL_CONFIG, 0x10);  // ±8G full scale
    mpu_write(MPU_GYRO_CONFIG,  0x08);  // ±500°/s

    Serial.println("[IMU] MPU-6050 initialised — ±8G, 20Hz");
    return true;
}

static ImuSample readIMU() {
    uint8_t raw[14];
    ImuSample s = {};
    s.timestamp_ms = millis();

    if (!mpu_read_block(MPU_ACCEL_XOUT_H, raw, 14)) {
        return s;
    }

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10]<< 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12]<< 8) | raw[13]);

    s.ax = ax_raw / ACCEL_SCALE_G;
    s.ay = ay_raw / ACCEL_SCALE_G;
    s.az = az_raw / ACCEL_SCALE_G;
    s.gx = gx_raw / GYRO_SCALE_DPS;
    s.gy = gy_raw / GYRO_SCALE_DPS;
    s.gz = gz_raw / GYRO_SCALE_DPS;

    // Resultant G magnitude
    s.g_total = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);

    // Roll angle (tilt for bike fall detection)
    s.tilt_deg = atan2f(s.ay, s.az) * 180.0f / M_PI;

    return s;
}

// ── Pre-crash buffer ──────────────────────────
static void pushPreCrash(float speed_kmh, float g_total, double lat, double lon) {
    static uint32_t last_push = 0;
    // Decimate to 2Hz for buffer efficiency
    if (millis() - last_push < 500) return;
    last_push = millis();

    pre_crash_buf[buf_head] = {
        .timestamp_ms = millis(),
        .speed_kmh    = speed_kmh,
        .g_total      = g_total,
        .lat          = (float)lat,
        .lon          = (float)lon
    };
    buf_head = (buf_head + 1) % PRE_CRASH_SLOTS;
    if (buf_count < PRE_CRASH_SLOTS) buf_count++;
}

static void copyPreCrashToPacket(SosPacket* pkt) {
    pkt->trace_count = buf_count;
    uint8_t idx = buf_head;
    for (uint8_t i = 0; i < buf_count && i < 60; i++) {
        idx = (idx - 1 + PRE_CRASH_SLOTS) % PRE_CRASH_SLOTS;
        pkt->velocity_trace[buf_count - 1 - i] = pre_crash_buf[idx];
    }
}

// ── Crash severity estimation ─────────────────
static CrashSeverity estimateSeverity(float peak_g, float delta_v) {
    if (peak_g < HARSH_BRAKE_G * 1.5f) return CrashSeverity::HARSH;
    if (peak_g < CRASH_G_THRESHOLD)     return CrashSeverity::HARSH;
    if (delta_v < 20.0f)                return CrashSeverity::MODERATE;
    return CrashSeverity::SEVERE;
}

// ── SOS button ISR state ──────────────────────
static volatile bool sos_button_pressed = false;
static uint32_t      sos_button_time    = 0;

static void IRAM_ATTR sosButtonISR() {
    sos_button_pressed = true;
    sos_button_time    = millis();
}

// ── Stage 3: countdown task (spawned on trigger) ──
static void armCountdownTask(void* pvParams) {
    CrashEvent* event = (CrashEvent*)pvParams;
    uint32_t start    = millis();
    bool     cancelled = false;

    Serial.println("[CRASH] ⚠  Stage 3: SOS armed — 15s override window");
    xEventGroupSetBits(g_events, EVT_SOS_ARMED);

    // LED: rapid red flash; buzzer: beep pattern
    while (millis() - start < CANCEL_WINDOW_MS) {
        // Check for driver cancel: long press SOS button (3s)
        if (sos_button_pressed) {
            uint32_t press_start = sos_button_time;
            while (digitalRead(SOS_BUTTON_PIN) == LOW) {
                if (millis() - press_start > 3000) {
                    cancelled = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            sos_button_pressed = false;
            if (cancelled) break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xEventGroupClearBits(g_events, EVT_SOS_ARMED);

    if (cancelled) {
        Serial.println("[CRASH] ✓ Driver cancelled SOS — logging as harsh event");
        event->cancelled_by_driver = true;
        event->severity = CrashSeverity::HARSH;
    } else {
        Serial.println("[CRASH] 🚨 SOS TRANSMITTING");
        event->sos_sent = true;
        xEventGroupSetBits(g_events, EVT_SOS_ACTIVE);
        // Push to crash event queue for connectivity task
        xQueueSend(q_crash_events, event, 0);
    }

    crash_state = CrashState::SOS_SENT;
    free(event);
    vTaskDelete(nullptr);
}

// ── Main crash detector task ──────────────────
void taskCrashDetector(void* pvParams) {
    Serial.println("[CRASH] Task started");

    if (!initMPU6050()) {
        Serial.println("[CRASH] FATAL: IMU init failed — task halted");
        vTaskDelete(nullptr);
        return;
    }

    // Attach SOS button interrupt
    attachInterrupt(digitalPinToInterrupt(SOS_BUTTON_PIN), sosButtonISR, FALLING);

    ImuSample  current;
    float      speed_at_trigger  = 0.0f;
    uint32_t   trigger_start_ms  = 0;
    float      consecutive_g     = 0.0f;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        current = readIMU();

        if (DEBUG_IMU) {
            Serial.printf("[IMU] ax=%.2f ay=%.2f az=%.2f G=%.2f tilt=%.1f°\n",
                          current.ax, current.ay, current.az,
                          current.g_total, current.tilt_deg);
        }

        // ── Feed pre-crash buffer ──
        float speed = 0.0f;
        double lat = 0.0, lon = 0.0;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            speed = g_state.last_gps.speed_kmh;
            lat   = g_state.last_gps.lat;
            lon   = g_state.last_gps.lon;
            // Update last IMU in shared state
            g_state.last_imu = current;
            xSemaphoreGive(g_state_mutex);
        }
        pushPreCrash(speed, current.g_total, lat, lon);

        // ── Tilt detection (motorcycle fall) ──
        if (fabsf(current.tilt_deg) > TILT_THRESHOLD_DEG && crash_state == CrashState::IDLE) {
            Serial.printf("[CRASH] Tilt detected: %.1f°\n", current.tilt_deg);
            // Treat as moderate crash trigger
            current.g_total = CRASH_G_THRESHOLD;
        }

        // ── Manual SOS button (short press) ──
        if (sos_button_pressed && crash_state == CrashState::IDLE) {
            uint32_t press_start = sos_button_time;
            vTaskDelay(pdMS_TO_TICKS(300));
            if (digitalRead(SOS_BUTTON_PIN) == LOW) {
                // Still held — it's a deliberate press
                Serial.println("[CRASH] Manual SOS button pressed");
                crash_state = CrashState::ARMED;
                CrashEvent* evt = (CrashEvent*)malloc(sizeof(CrashEvent));
                memset(evt, 0, sizeof(CrashEvent));
                evt->severity          = CrashSeverity::MODERATE;
                evt->peak_g            = 0.0f;
                evt->speed_at_impact_kmh = speed;
                strncpy(evt->device_id, DEVICE_ID, 15);
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    evt->location = g_state.last_gps;
                    xSemaphoreGive(g_state_mutex);
                }
                copyPreCrashToPacket(nullptr);   // Just log
                evt->sos_sent = true;
                xQueueSend(q_crash_events, evt, 0);
                free(evt);
            }
            sos_button_pressed = false;
        }

        switch (crash_state) {
            case CrashState::IDLE: {
                // ── Harsh event logging (below crash threshold) ──
                if (current.g_total > HARSH_BRAKE_G && current.g_total < CRASH_G_THRESHOLD) {
                    // Log driving behaviour event — no SOS
                    Serial.printf("[DRIVE] Harsh event: %.2fG\n", current.g_total);
                }

                // ── Stage 1: Raw trigger ──
                if (current.g_total >= CRASH_G_THRESHOLD) {
                    if (trigger_start_ms == 0) {
                        trigger_start_ms = millis();
                        peak_sample = current;
                    }
                    if (current.g_total > peak_sample.g_total) peak_sample = current;

                    if (millis() - trigger_start_ms >= CRASH_DURATION_MS) {
                        Serial.printf("[CRASH] Stage 1 TRIGGERED: %.2fG for %ums\n",
                                      peak_sample.g_total, CRASH_DURATION_MS);
                        crash_state      = CrashState::TRIGGERED;
                        trigger_time_ms  = millis();
                        speed_at_trigger = speed;
                    }
                } else {
                    trigger_start_ms = 0;
                }
                break;
            }

            case CrashState::TRIGGERED:
            case CrashState::CONFIRMING: {
                crash_state = CrashState::CONFIRMING;

                // Stage 2: Confirm via speed delta within 2 seconds
                bool speed_dropped = (speed_at_trigger - speed) > 30.0f;
                bool chaotic_motion = current.g_total > 1.5f;  // Still moving erratically

                if (speed_dropped || chaotic_motion ||
                    (millis() - trigger_time_ms > 2000)) {
                    // Confirmed crash
                    float delta_v = speed_at_trigger - speed;
                    CrashSeverity sev = estimateSeverity(peak_sample.g_total, delta_v);

                    Serial.printf("[CRASH] Stage 2 CONFIRMED: speed_delta=%.1f km/h severity=%d\n",
                                  delta_v, (int)sev);

                    // Build crash event
                    CrashEvent* evt = (CrashEvent*)malloc(sizeof(CrashEvent));
                    memset(evt, 0, sizeof(CrashEvent));
                    evt->severity            = sev;
                    evt->peak_g              = peak_sample.g_total;
                    evt->speed_at_impact_kmh = speed_at_trigger;
                    evt->speed_delta_kmh     = delta_v;
                    evt->peak_sample         = peak_sample;
                    evt->timestamp           = (uint32_t)(millis() / 1000);
                    strncpy(evt->device_id, DEVICE_ID, 15);

                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        evt->location = g_state.last_gps;
                        xSemaphoreGive(g_state_mutex);
                    }

                    crash_state = CrashState::ARMED;
                    trigger_start_ms = 0;

                    // Spawn countdown task for Stage 3
                    xTaskCreate(armCountdownTask, "SosArm", 4096, evt, PRIORITY_CRASH + 1, nullptr);
                }

                // Timeout: if 2 seconds passed with no confirmation, reset
                if (millis() - trigger_time_ms > 2000 && crash_state == CrashState::CONFIRMING) {
                    Serial.println("[CRASH] Stage 2 timeout — reset to IDLE");
                    crash_state      = CrashState::IDLE;
                    trigger_start_ms = 0;
                }
                break;
            }

            case CrashState::ARMED:
                // Countdown task is running — just wait
                break;

            case CrashState::SOS_SENT:
                // Allow reset after 60s
                if (millis() - trigger_time_ms > 60000) {
                    crash_state = CrashState::IDLE;
                    xEventGroupClearBits(g_events, EVT_SOS_ACTIVE);
                    Serial.println("[CRASH] Reset to IDLE after SOS cooldown");
                }
                break;
        }

        // Precise 50ms tick (20Hz)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_POLL_MS));
    }
}
