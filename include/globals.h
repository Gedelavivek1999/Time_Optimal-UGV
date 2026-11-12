#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ============================================================
//  SAFETY SWITCH
//  false = whole pipeline runs, Serial shows commands, but the
//          wheels never turn (use this first, wheels lifted).
// ============================================================
#define MOTORS_ENABLED   true
#define DEBUG_TELEMETRY  true   // one Serial line every TELEMETRY_EVERY samples

// ---------------- Hardware pins (verified by your tests) ----------------
const int PIN_TRIG = 13;
const int PIN_ECHO = 25;   // ECHO is 5V -> use a divider / level shifter!

const int PIN_ENA = 32;
const int PIN_IN1 = 26;
const int PIN_IN2 = 27;
const int PIN_ENB = 21;
const int PIN_IN3 = 22;
const int PIN_IN4 = 23;

// ---------------- Sensor ----------------
const uint32_t SENSOR_PERIOD_MS = 50;   // 20 Hz ping rate
const uint32_t SENSOR_WAIT_MS   = 30;   // wait for echo (4 m = ~23.5 ms)
const float    MIN_RANGE_CM     = 2.0f;
const float    MAX_RANGE_CM     = 400.0f;

// ---------------- Estimator (alpha-beta filter) ----------------
const float ALPHA            = 0.5f;    // distance gain   (higher = trust sensor more)
const float BETA             = 0.2f;    // velocity gain
const float JUMP_REJECT_CM   = 60.0f;   // innovation larger than this = suspicious
const int   REJECT_REINIT    = 3;       // consecutive suspicious samples -> re-init filter
const int   LOST_RESET_COUNT = 5;       // invalid samples in a row -> forget the track

// ---------------- Time-optimal control ----------------
const float SAFE_DISTANCE_CM  = 20.0f;  // desired gap to target
const float MAX_ACCEL_CM_S2   = 50.0f;  // TUNE: sets where the switch happens
// Pure bang-bang: command is always +100 (full forward) or -100 (full reverse)
const float CMD_MAX           = 100.0f;

// ---------------- Motor driver ----------------
const uint32_t MOTOR_FAILSAFE_MS   = 150;  // no command for this long -> stop
const uint32_t REVERSAL_COAST_MS   = 30;   // pause when flipping direction
const float    CMD_MIN_ACTIVE      = 5.0f; // |cmd| below this = stop
const uint32_t REVERSE_MAX_MS      = 1500; // no rear sensor: limit continuous reverse

// ---------------- Math task ----------------
const uint32_t MATH_RX_TIMEOUT_MS = 200;   // sensor silent this long -> stop
const int      TELEMETRY_EVERY    = 4;     // print every 4th sample (200 ms)

// ---------------- Data passed sensor -> math ----------------
struct TargetData {
    float    distance_cm;   // raw HC-SR04 distance (only meaningful if valid)
    uint32_t timestamp_ms;  // millis() when the sample was taken
    bool     valid;         // false = no usable echo this cycle
};

// ---------------- Queues ----------------
extern QueueHandle_t targetQueue;   // TargetData, length 1 (latest value)
extern QueueHandle_t motorQueue;    // float -100..+100, length 1 (latest value)

// ---------------- Task prototypes ----------------
void Task_Sensor(void *pvParameters);
void Task_Math(void *pvParameters);
void Task_Motor(void *pvParameters);

#endif
