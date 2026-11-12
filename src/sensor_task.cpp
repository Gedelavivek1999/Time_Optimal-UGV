#include <Arduino.h>
#include "driver/gpio.h"
#include "globals.h"

// ---------------- ISR shared data ----------------
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_width_us = 0;
static volatile bool     echo_armed    = false;  // rising edge seen, waiting for falling
static volatile bool     echo_done     = false;  // full pulse captured

static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR echo_isr() {
    if (gpio_get_level((gpio_num_t)PIN_ECHO)) {
        echo_start_us = micros();
        echo_armed = true;
    } else if (echo_armed) {
        echo_width_us = micros() - echo_start_us;
        echo_armed = false;
        echo_done = true;
    }
}

void Task_Sensor(void *pvParameters) {
    pinMode(PIN_TRIG, OUTPUT);
    digitalWrite(PIN_TRIG, LOW);
    pinMode(PIN_ECHO, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echo_isr, CHANGE);

    TargetData data;
    data.distance_cm  = 0.0f;
    data.timestamp_ms = 0;
    data.valid        = false;

    const TickType_t period = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    const TickType_t wait   = pdMS_TO_TICKS(SENSOR_WAIT_MS);
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        // Discard anything left over (e.g. the late falling edge of a
        // "no object" 38 ms echo) BEFORE triggering a new measurement.
        portENTER_CRITICAL(&sensorMux);
        echo_done  = false;
        echo_armed = false;
        // 12 us trigger pulse, protected from preemption
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);
        portEXIT_CRITICAL(&sensorMux);

        // Sleep; the ISR measures the echo in the background.
        vTaskDelay(wait);

        bool     done;
        uint32_t width;
        portENTER_CRITICAL(&sensorMux);
        done  = echo_done;
        width = echo_width_us;
        echo_done = false;
        portEXIT_CRITICAL(&sensorMux);

        data.timestamp_ms = millis();
        data.valid = false;
        if (done) {
            float dist = (float)width * 0.0343f / 2.0f;
            if (dist >= MIN_RANGE_CM && dist <= MAX_RANGE_CM) {
                data.distance_cm = dist;
                data.valid = true;
            }
        }

        xQueueOverwrite(targetQueue, &data);
        vTaskDelayUntil(&lastWake, period);
    }
}
