#pragma once

// ─────────────────────────────────────────────
//  SafeBeacon — config.h
//  Hardware: Honda Brio 2013 + ESP32 + SIM7600EI
// ─────────────────────────────────────────────

// ── Device Identity ──────────────────────────
#define DEVICE_ID           "BRIO-001"
#define FIRMWARE_VERSION    "0.3.1"
#define HW_REVISION         "proto-v1"

// ── MQTT / Cloud ─────────────────────────────
#define MQTT_BROKER         "your-broker.com"    // Replace with actual broker
#define MQTT_PORT           8883
#define MQTT_USER           ""                   // Optional auth
#define MQTT_PASS           ""
#define MQTT_TOPIC_SOS      "safebeacon/sos"
#define MQTT_TOPIC_TELE     "safebeacon/telemetry"
#define MQTT_TOPIC_HEALTH   "safebeacon/health"
#define MQTT_QOS_SOS        1                    // At-least-once for SOS
#define MQTT_QOS_TELE       0                    // Best-effort for telemetry

// ── SIM / Cellular ───────────────────────────
#define APN_NAME            "airtelgprs.com"     // Change to your SIM APN
//  Jio APN:     "jionet"
//  Airtel APN:  "airtelgprs.com"
//  Vi APN:      "portalnmms"
#define EMERGENCY_NUMBER    "+919XXXXXXXXX"       // GUARD call number

// ── Pin Definitions ──────────────────────────
// SIM7600EI — Hardware Serial 2
#define SIM_RX_PIN          16
#define SIM_TX_PIN          17
#define SIM_PWRKEY_PIN      4
#define SIM_RST_PIN         5
#define SIM7600_BAUD        115200

// ELM327 OBD2 — Hardware Serial 1
#define OBD_RX_PIN          26
#define OBD_TX_PIN          27
#define OBD_BAUD            38400

// MPU-6050 IMU — I2C
#define IMU_SDA_PIN         21
#define IMU_SCL_PIN         22
#define IMU_INT_PIN         23                   // Interrupt pin (optional)
#define MPU6050_ADDR        0x68

// Power & Analog
#define VBAT_ADC_PIN        34                   // 12V rail monitor (via divider)
#define VBAT_R1             100000.0f            // 100kΩ top resistor
#define VBAT_R2             10000.0f             // 10kΩ bottom resistor
#define ADC_REF_VOLTAGE     3.3f
#define ADC_RESOLUTION      4095.0f

// User Interface
#define SOS_BUTTON_PIN      0                    // Active LOW, INPUT_PULLUP
#define LED_RED_PIN         25
#define LED_GREEN_PIN       33
#define LED_BLUE_PIN        32
#define BUZZER_PIN          19                   // Optional passive buzzer

// ── Crash Detection Thresholds ───────────────
#define CRASH_G_THRESHOLD   3.5f                 // G-force trigger (tune this!)
#define CRASH_DURATION_MS   80                   // Min duration to confirm
#define HARSH_BRAKE_G       0.8f                 // Log as harsh braking event
#define HARSH_ACCEL_G       0.6f                 // Log as harsh acceleration
#define TILT_THRESHOLD_DEG  60.0f                // Bike fall / rollover
#define CANCEL_WINDOW_MS    15000                // SOS override window (15s)
#define PRE_CRASH_BUFFER_S  30                   // Seconds of pre-crash data kept

// ── OBD2 PIDs (Honda Brio 2013 confirmed) ────
#define PID_ENGINE_RPM      0x0C
#define PID_VEHICLE_SPEED   0x0D
#define PID_COOLANT_TEMP    0x05
#define PID_THROTTLE_POS    0x11
#define PID_FUEL_LEVEL      0x2F
#define PID_CTRL_MODULE_V   0x42
#define PID_INTAKE_TEMP     0x0F
#define PID_INTAKE_MAP      0x0B
#define PID_AMBIENT_TEMP    0x46
#define PID_OIL_TEMP        0x5C
#define PID_MIL_DTC         0x01
#define PID_FUEL_STATUS     0x03
#define OBD_MODE_LIVE       0x01
#define OBD_MODE_DTC        0x03
#define OBD_MODE_CLEAR_DTC  0x04

// ── Sampling Intervals ───────────────────────
#define GPS_POLL_MS         10000                // GPS fix every 10s
#define OBD_POLL_MS         5000                 // OBD read every 5s
#define IMU_POLL_MS         50                   // IMU at 20Hz
#define VBAT_POLL_MS        30000                // Battery voltage every 30s
#define TELE_PUBLISH_MS     15000                // Publish telemetry every 15s
#define HEARTBEAT_MS        60000                // Cloud heartbeat every 60s

// ── Voltage Thresholds ───────────────────────
#define VBAT_WARN_V         12.0f                // Warn: battery getting low
#define VBAT_CRITICAL_V     11.5f                // Critical: imminent failure
#define VBAT_CHARGING_V     13.8f                // Alternator charging detected
#define VBAT_CRANK_V        9.0f                 // Cranking transient (ignore)

// ── Flash / Storage ──────────────────────────
#define FLASH_RING_SIZE     500                  // Max buffered events offline
#define TRIP_MIN_DISTANCE_M 100                  // Minimum trip distance to log

// ── FreeRTOS Task Priorities ─────────────────
#define PRIORITY_CRASH      5                    // Highest — safety critical
#define PRIORITY_GPS        4
#define PRIORITY_OBD        3
#define PRIORITY_PUBLISH    2
#define PRIORITY_UI         1                    // Lowest — LED/buzzer

// ── Debug ─────────────────────────────────────
#define DEBUG_SERIAL        true
#define DEBUG_BAUD          115200
#define DEBUG_OBD           false                // Verbose OBD AT commands
#define DEBUG_IMU           false                // Raw IMU values
