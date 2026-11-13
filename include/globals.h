#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


#define MOTORS_ENABLED   true


const int PIN_TRIG = 13;
const int PIN_ECHO = 25;   // ECHO is 5V -> use a divider / level shifter!

const int PIN_ENA = 32;
const int PIN_IN1 = 26;
const int PIN_IN2 = 27;
const int PIN_ENB = 21;
const int PIN_IN3 = 22;
const int PIN_IN4 = 23;

const uint32_t SENSOR_PERIOD_MS = 50;   // 20 Hz ping rate
const uint32_t SENSOR_WAIT_MS   = 30;   // wait for echo (4 m = ~23.5 ms)
const float    MIN_RANGE_CM     = 2.0f;
const float    MAX_RANGE_CM     = 400.0f;

const float ALPHA            = 0.5f;    // distance gain   (higher = trust sensor more)
const float BETA             = 0.2f;    // velocity gain
const float JUMP_REJECT_CM   = 60.0f;   // innovation larger than this = suspicious
const int   REJECT_REINIT    = 3;       // consecutive suspicious samples -> re-init filter
const int   LOST_RESET_COUNT = 5;       // invalid samples in a row -> forget the track

const float SAFE_DISTANCE_CM  = 20.0f;  // desired gap to target
const float MAX_ACCEL_CM_S2   = 50.0f;  // TUNE: sets where the switch happens
// Pure bang-bang: command is always +100 (full forward) or -100 (full reverse)
const float CMD_MAX           = 100.0f;

const uint32_t MOTOR_FAILSAFE_MS   = 150;  // no command for this long -> stop
const uint32_t REVERSAL_COAST_MS   = 30;   // pause when flipping direction
const float    CMD_MIN_ACTIVE      = 5.0f; // |cmd| below this = stop
const uint32_t REVERSE_MAX_MS      = 1500; // no rear sensor: limit continuous reverse


const uint32_t MATH_RX_TIMEOUT_MS = 200;   // sensor silent this long -> stop

// Dual output: the SAME CSV line goes to
//   1) hardware UART2 (GPIO17 TX / GPIO16 RX)  -> real UART signal
//   2) Bluetooth SPP ("UGV-Pursuit")           -> shows as a COM port on the PC
#define ENABLE_UART_OUT   false
#define ENABLE_BT_OUT     false
#define ENABLE_USB_OUT    true    // data on the USB Serial Monitor
const long TELEM_BAUD   = 115200;
const int  UART2_RX_PIN = 16;
const int  UART2_TX_PIN = 17;
#define BT_DEVICE_NAME    "UGV-Pursuit"
const int  TELEM_BUFFER_LINES = 400;   // ~20 s of history kept while no PC is connected
const int  TELEM_LINE_MAX     = 64;

// UART loopback self-test: wire GPIO17 (TX) to GPIO16 (RX) with ONE jumper.
// The task reads back what it just sent and reports OK/BAD counts on USB Serial.
// Leave false when the jumper is not fitted.
#define UART_LOOPBACK_TEST  false

struct TargetData {
    float    distance_cm;   // raw HC-SR04 distance (only meaningful if valid)
    uint32_t timestamp_ms;  // millis() when the sample was taken
    bool     valid;         // false = no usable echo this cycle
};

struct TelemetryPacket {
    uint32_t t_ms;
    float    raw_cm;     // -1 if no valid echo
    float    dist_cm;    // filtered distance
    float    vel_cm_s;   // filtered velocity
    float    S;          // switching surface
    float    cmd;        // -100 / 0 / +100
    bool     valid;
};


extern QueueHandle_t targetQueue;   // TargetData, length 1 (latest value)
extern QueueHandle_t motorQueue;    // float -100..+100, length 1 (latest value)
extern QueueHandle_t telemetryQueue; // TelemetryPacket, length 1 (latest value)


void Task_Sensor(void *pvParameters);
void Task_Math(void *pvParameters);
void Task_Motor(void *pvParameters);
void Task_Uart(void *pvParameters);

#endif
