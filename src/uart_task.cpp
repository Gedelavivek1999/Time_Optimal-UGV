#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "globals.h"

#if ENABLE_BT_OUT
  #include <BluetoothSerial.h>
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error "Bluetooth is not enabled for this board/core (need a classic ESP32)"
  #endif
  static BluetoothSerial SerialBT;
#endif

static const char CSV_HEADER[] = "t_ms,raw_cm,dist_cm,vel_cm_s,S,cmd,valid\n";

// ---- history buffer: keeps the latest lines while no PC is connected ----
static char    ring[TELEM_BUFFER_LINES][TELEM_LINE_MAX];
static uint8_t ring_len[TELEM_BUFFER_LINES];
static int     ring_head  = 0;   // next slot to write
static int     ring_count = 0;

static void ring_push(const char *line, int n) {
    memcpy(ring[ring_head], line, n);
    ring_len[ring_head] = (uint8_t)n;
    ring_head = (ring_head + 1) % TELEM_BUFFER_LINES;
    if (ring_count < TELEM_BUFFER_LINES) ring_count++;
}

#if ENABLE_BT_OUT
static void ring_flush_to_bt() {
    int idx = (ring_head - ring_count + TELEM_BUFFER_LINES) % TELEM_BUFFER_LINES;
    for (int i = 0; i < ring_count; i++) {
        SerialBT.write((const uint8_t *)ring[idx], ring_len[idx]);
        idx = (idx + 1) % TELEM_BUFFER_LINES;
        if ((i & 7) == 7) vTaskDelay(1);   // let the Bluetooth stack drain
    }
    ring_count = 0;
}
#endif


void Task_Uart(void *pvParameters) {
    (void)pvParameters;
    TelemetryPacket p;
    char line[TELEM_LINE_MAX];

#if ENABLE_USB_OUT
    Serial.print(CSV_HEADER);
#endif

#if ENABLE_UART_OUT
    Serial2.begin(TELEM_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    Serial2.print(CSV_HEADER);
  #if UART_LOOPBACK_TEST
    Serial2.flush();
    vTaskDelay(pdMS_TO_TICKS(20));
    while (Serial2.available()) Serial2.read();   // discard echo of the header
  #endif
#endif
#if UART_LOOPBACK_TEST
    unsigned long lb_ok = 0, lb_bad = 0;
#endif
#if ENABLE_BT_OUT
    if (SerialBT.begin(BT_DEVICE_NAME)) {
        Serial.printf("Bluetooth ready: pair with \"%s\"\n", BT_DEVICE_NAME);
    } else {
        Serial.println("Bluetooth start FAILED");
    }
    bool was_connected = false;
#endif

    while (true) {
        if (xQueueReceive(telemetryQueue, &p, portMAX_DELAY) != pdPASS) continue;

        int n = snprintf(line, sizeof(line), "%lu,%.1f,%.1f,%.1f,%.1f,%.0f,%d\n",
                         (unsigned long)p.t_ms, p.raw_cm, p.dist_cm,
                         p.vel_cm_s, p.S, p.cmd, p.valid ? 1 : 0);
        if (n <= 0) continue;
        if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1;

#if ENABLE_UART_OUT
        Serial2.write((const uint8_t *)line, n);          // hardware UART, always on
#endif
#if ENABLE_USB_OUT
        Serial.write((const uint8_t *)line, n);          // USB Serial Monitor
#endif
#if ENABLE_BT_OUT
        bool connected = SerialBT.hasClient();
        if (connected) {
            if (!was_connected) {                          // PC just connected
                SerialBT.print(CSV_HEADER);
                ring_flush_to_bt();                        // send what happened before
            }
            SerialBT.write((const uint8_t *)line, n);      // live data
        } else {
            ring_push(line, n);                            // keep for later
        }
        was_connected = connected;
#else
        (void)ring_push;
#endif

#if UART_LOOPBACK_TEST
        // Read back the line we just sent on GPIO17 via GPIO16 (jumper required)
        vTaskDelay(pdMS_TO_TICKS(8));                 // 45 bytes @115200 = ~4 ms
        char rx[TELEM_LINE_MAX];
        int  rn = 0;
        while (Serial2.available() && rn < (int)sizeof(rx)) rx[rn++] = (char)Serial2.read();
        while (Serial2.available()) Serial2.read();   // drop any excess
        if (rn == n && memcmp(rx, line, n) == 0) lb_ok++; else lb_bad++;
        if (((lb_ok + lb_bad) % 20) == 0) {           // once per second
            if (lb_ok == 0) {
                Serial.println("[UART loopback] nothing received - fit jumper GPIO17 -> GPIO16");
            } else {
                Serial.printf("[UART loopback] RX OK=%lu BAD=%lu\n", lb_ok, lb_bad);
            }
        }
#endif
    }
}