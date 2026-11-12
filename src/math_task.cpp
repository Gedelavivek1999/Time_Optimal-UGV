#include <Arduino.h>
#include <math.h>
#include "globals.h"

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Pure bang-bang time-optimal switching law.
//   e_p = d - d_safe,  e_v = d_dot
//   S   = e_p + e_v*|e_v| / (2*A_max)
//   S > 0 -> FULL forward   (+100)
//   S <= 0 -> FULL reverse  (-100)  (brake)
static float compute_command(float d, float v, float &S_out) {
    float e_p = d - SAFE_DISTANCE_CM;
    float S = e_p + (v * fabsf(v)) / (2.0f * MAX_ACCEL_CM_S2);
    S_out = S;
    return (S > 0.0f) ? CMD_MAX : -CMD_MAX;
}

void Task_Math(void *pvParameters) {
    TargetData s;
    bool     tracking   = false;
    float    x          = 0.0f;   // filtered distance (cm)
    float    v          = 0.0f;   // filtered velocity (cm/s)
    uint32_t last_ts    = 0;
    int      miss       = 0;
    int      rejects    = 0;
    uint32_t sample_no  = 0;
    float    cmd        = 0.0f;
    float    S          = 0.0f;

    while (true) {
        if (xQueueReceive(targetQueue, &s, pdMS_TO_TICKS(MATH_RX_TIMEOUT_MS)) != pdPASS) {
            // Sensor task silent -> stop
            tracking = false;
            cmd = 0.0f;
            xQueueOverwrite(motorQueue, &cmd);
            Serial.println("[math] no sensor data -> STOP");
            continue;
        }

        if (!s.valid) {
            // No usable echo: stop immediately, forget track after a few misses
            miss++;
            cmd = 0.0f;
            S = 0.0f;
            if (miss >= LOST_RESET_COUNT) tracking = false;
        } else {
            miss = 0;
            float z = s.distance_cm;

            if (!tracking) {
                x = z;
                v = 0.0f;
                last_ts = s.timestamp_ms;
                rejects = 0;
                tracking = true;
                cmd = compute_command(x, v, S);   // act immediately on identification
            } else {
                float dt = (float)(s.timestamp_ms - last_ts) * 0.001f;
                last_ts = s.timestamp_ms;
                dt = clampf(dt, 0.02f, 0.5f);

                float x_pred = x + v * dt;
                float r = z - x_pred;   // innovation

                if (fabsf(r) > JUMP_REJECT_CM) {
                    rejects++;
                    if (rejects >= REJECT_REINIT) {   // it's real: re-initialise
                        x = z;
                        v = 0.0f;
                        rejects = 0;
                    } else {
                        x = x_pred;                    // ignore the outlier
                    }
                } else {
                    rejects = 0;
                    x = x_pred + ALPHA * r;
                    v = v + (BETA * r) / dt;
                }
                cmd = compute_command(x, v, S);
            }
        }

        xQueueOverwrite(motorQueue, &cmd);

#if DEBUG_TELEMETRY
        if ((++sample_no % TELEMETRY_EVERY) == 0) {
            Serial.printf("[math] raw=%.1f valid=%d  d=%.1f v=%.1f  S=%.1f  cmd=%.0f\n",
                          s.valid ? s.distance_cm : -1.0f, (int)s.valid,
                          x, v, S, cmd);
        }
#endif
    }
}
