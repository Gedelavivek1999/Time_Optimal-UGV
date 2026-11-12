#include <Arduino.h>
#include <math.h>
#include "globals.h"

enum Dir { DIR_STOP = 0, DIR_FWD = 1, DIR_REV = -1 };

static void motor_stop() {
    digitalWrite(PIN_ENA, LOW);  digitalWrite(PIN_ENB, LOW);
    digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, LOW);
}

// Direction logic exactly as validated in your motor test, full speed.
static void motor_drive(Dir dir) {
    if (dir == DIR_FWD) {
        digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
    } else {
        digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    }
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
}

void Task_Motor(void *pvParameters) {
    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    motor_stop();

    float    cmd           = 0.0f;
    Dir      cur_dir       = DIR_STOP;
    uint32_t reverse_start = 0;
    bool     reverse_block = false;

    while (true) {
        // Block until Math sends a command. Timeout = failsafe stop.
        if (xQueueReceive(motorQueue, &cmd, pdMS_TO_TICKS(MOTOR_FAILSAFE_MS)) != pdPASS) {
            cmd = 0.0f;
        }

        Dir want = DIR_STOP;
        if (fabsf(cmd) >= CMD_MIN_ACTIVE) want = (cmd > 0.0f) ? DIR_FWD : DIR_REV;

#if !MOTORS_ENABLED
        want = DIR_STOP;   // dry run
#endif

        // Reverse limit (no rear sensor)
        if (want == DIR_REV) {
            if (reverse_start == 0) reverse_start = millis();
            if ((millis() - reverse_start) > REVERSE_MAX_MS) reverse_block = true;
            if (reverse_block) want = DIR_STOP;
        } else {
            reverse_start = 0;
            if (cmd >= 0.0f) reverse_block = false;
        }

        if (want == DIR_STOP) {
            if (cur_dir != DIR_STOP) motor_stop(); 
            cur_dir = DIR_STOP;
        } else if (want != cur_dir) {
            // Direction change: brief coast to protect the H-bridge and gears
            if (cur_dir != DIR_STOP) {
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(REVERSAL_COAST_MS));
            }
            motor_drive(want);
            cur_dir = want;
        }
    }
}
