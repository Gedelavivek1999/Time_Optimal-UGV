# UGV Time-Optimal Pursuit System

An ESP32-based Unmanned Ground Vehicle (UGV) project that implements a time-optimal "bang-bang" tracking controller. Built on FreeRTOS, the system concurrently handles hardware interrupts, signal filtering, motor control, and live telemetry without blocking the main control loop.

## Key Features
 **Advanced Control Logic:** Uses a pure bang-bang time-optimal switching law based on a sliding surface to aggressively pursue targets or brake.
 **Real-Time Filtering:** Implements an Alpha-Beta filter with jump-rejection to smooth noisy ultrasonic sensor data and estimate target velocity.
 **FreeRTOS Architecture:** Separates operations into dedicated, prioritized tasks (Motor, Math, Sensor, UART) pinned to specific CPU cores.
 **Live Telemetry:** Streams real-time CSV data (distance, velocity, control commands) over Bluetooth, USB, or Hardware UART for PC analysis.
