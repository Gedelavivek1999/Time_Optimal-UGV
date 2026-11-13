#include <Arduino.h>
#include "globals.h"

QueueHandle_t targetQueue = NULL;
QueueHandle_t motorQueue  = NULL;
QueueHandle_t telemetryQueue = NULL;

static void halt_with_message(const char *msg) {
    // Unrecoverable setup error: report forever, never start the robot.
    while (true) {
        Serial.println(msg);
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("=== UGV time-optimal pursuit: starting ===");
    Serial.printf("MOTORS_ENABLED = %d\n", (int)MOTORS_ENABLED);

    // Length-1 queues used with xQueueOverwrite = "latest value" mailboxes
    targetQueue = xQueueCreate(1, sizeof(TargetData));
    motorQueue  = xQueueCreate(1, sizeof(float));
    telemetryQueue = xQueueCreate(1, sizeof(TelemetryPacket));
    if (targetQueue == NULL || motorQueue == NULL || telemetryQueue == NULL) {
        halt_with_message("FATAL: queue creation failed");
    }

    // Motor first, so its pins are driven to a safe (stopped) state immediately.
    // Priorities: Motor 5 > Sensor 4 > Math 3 > Uart 2. All on core 1.
    BaseType_t ok = pdPASS;
    ok &= xTaskCreatePinnedToCore(Task_Motor,  "Motor",  4096, NULL, 5, NULL, 1);
    ok &= xTaskCreatePinnedToCore(Task_Math,   "Math",   4096, NULL, 3, NULL, 1);
    ok &= xTaskCreatePinnedToCore(Task_Sensor, "Sensor", 4096, NULL, 4, NULL, 1);
    ok &= // Telemetry task on core 0 (with the Bluetooth stack) so core 1 stays clean for control
    ok &= xTaskCreatePinnedToCore(Task_Uart,   "Uart",   6144, NULL, 2, NULL, 0);
    if (ok != pdPASS) {
        halt_with_message("FATAL: task creation failed");
    }
}

void loop() {
    // All work is done by FreeRTOS tasks.
    vTaskDelete(NULL);
}
