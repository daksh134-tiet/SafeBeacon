#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────
//  SafeBeacon — structs.h
//  Shared data structures across firmware tasks
// ─────────────────────────────────────────────

// ── GPS Fix ──────────────────────────────────
struct GpsFix {
    double   lat;
    double   lon;
    float    speed_kmh;
    float    altitude_m;
    float    accuracy_m;
    uint8_t  satellites;
    uint32_t timestamp;          // Unix epoch
    bool     valid;
};

// ── IMU Sample ───────────────────────────────
struct ImuSample {
    float    ax;                 // Acceleration X (G)
    float    ay;                 // Acceleration Y (G)
    float    az;                 // Acceleration Z (G)
    float    gx;                 // Gyro X (deg/s)
    float    gy;                 // Gyro Y (deg/s)
    float    gz;                 // Gyro Z (deg/s)
    float    g_total;            // |a| resultant magnitude
    float    tilt_deg;           // Roll angle
    uint32_t timestamp_ms;
};

// ── OBD Telemetry Snapshot ───────────────────
struct ObdSnapshot {
    uint16_t rpm;                // Engine RPM
    uint8_t  speed_kmh;          // Vehicle speed
    int8_t   coolant_temp_c;     // Coolant temperature
    int8_t   oil_temp_c;         // Oil temperature
    int8_t   intake_temp_c;      // Intake air temperature
    uint8_t  throttle_pct;       // Throttle position %
    uint8_t  fuel_level_pct;     // Fuel level %
    uint8_t  intake_map_kpa;     // Intake manifold pressure
    float    ctrl_module_v;      // ECU supply voltage
    bool     mil_on;             // Check engine light
    uint8_t  dtc_count;          // Active fault code count
    uint32_t timestamp;
    bool     valid;
};

// ── Vehicle Battery State ────────────────────
struct BatteryState {
    float    voltage_v;
    bool     charging;           // Alternator active
    bool     warn;               // Below warning threshold
    bool     critical;           // Below critical threshold
    uint32_t timestamp;
};

// ── Crash Event ──────────────────────────────
enum class CrashSeverity : uint8_t {
    NONE      = 0,
    HARSH     = 1,              // Hard brake / accel — logged only
    MODERATE  = 2,              // Impact, driver unharmed likely
    SEVERE    = 3               // High-G, SOS dispatched
};

struct CrashEvent {
    CrashSeverity severity;
    float         peak_g;
    float         speed_at_impact_kmh;
    float         speed_delta_kmh;      // Speed lost during crash
    GpsFix        location;
    ImuSample     peak_sample;
    uint32_t      timestamp;
    bool          sos_sent;
    bool          cancelled_by_driver;
    char          device_id[16];
};

// ── Pre-crash Ring Buffer Entry ───────────────
struct PreCrashEntry {
    uint32_t timestamp_ms;
    float    speed_kmh;
    float    g_total;
    float    lat;
    float    lon;
};

// ── SOS Packet (transmitted on crash) ────────
struct SosPacket {
    char          device_id[16];
    uint32_t      timestamp;
    double        lat;
    double        lon;
    float         speed_at_impact;
    float         peak_g;
    CrashSeverity severity;
    float         battery_v;
    uint8_t       dtc_count;
    bool          mil_on;
    // Pre-crash velocity trace (last 30 seconds)
    uint8_t       trace_count;
    PreCrashEntry velocity_trace[60];   // 30s @ 2Hz
};

// ── Trip Record ──────────────────────────────
struct TripRecord {
    uint32_t start_time;
    uint32_t end_time;
    double   start_lat;
    double   start_lon;
    double   end_lat;
    double   end_lon;
    float    distance_km;
    float    max_speed_kmh;
    float    avg_speed_kmh;
    uint16_t max_rpm;
    uint16_t harsh_brake_count;
    uint16_t harsh_accel_count;
    uint16_t overspeed_seconds;
    uint16_t idle_seconds;
    float    safety_score;          // 0–100
    char     device_id[16];
};

// ── Device Health Heartbeat ──────────────────
struct HealthPacket {
    char     device_id[16];
    char     fw_version[16];
    uint32_t uptime_s;
    float    battery_v;
    int8_t   rssi_dbm;              // Cellular signal strength
    uint8_t  gps_satellites;
    bool     obd_connected;
    bool     imu_ok;
    uint32_t timestamp;
};

// ── Shared State (FreeRTOS mutex protected) ──
struct DeviceState {
    GpsFix       last_gps;
    ObdSnapshot  last_obd;
    BatteryState battery;
    ImuSample    last_imu;
    TripRecord   current_trip;
    bool         sos_armed;
    bool         trip_active;
    bool         obd_available;
    uint32_t     last_publish_ms;
};
