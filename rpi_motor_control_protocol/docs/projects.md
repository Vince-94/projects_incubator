# Projects

Terminology:
- ISR: Interrupt Service Routine


## Project 1 - RTOS fundamentals + safe I/O (FOUNDATION)

A minimal FreeRTOS demo for the RP2350 that shows how to build tasks, safely handle hardware interrupts, and pass messages between tasks.

### Goals
- Learn FreeRTOS basics: tasks, priorities, delays, notifications, and queues
- Safely interact with hardware (LEDs, button) without blocking or busy-waiting
- Centralize logging to keep ISRs and tasks lightweight
- Provide a simple CLI over USB serial for interaction

### Implementing
- Two periodic tasks: blink two LEDs at different rates and different priorities.
- Button ISR + handler task: ISR signals a task via vTaskNotifyGiveFromISR, which then logs the event.
- Logger task: consumes messages from a FreeRTOS queue and prints them over USB serial.
- CLI task: reads characters from USB serial and posts them as log messages.


## Project 2 - Motor basics + encoder + PID loop (CONTROL)

Implement a FreeRTOS-based motor-control demo. It reads pulses from a single-channel encoder (or Hall sensor), computes RPM at a fixed control rate, runs a PID controller, and drives the motor via PWM. It’s written for learning and bench testing — emphasis on safe ISR design, deterministic control timing, and clear separation between ISR work and task work.

### Goals
- teach and demonstrate correct embedded/RTOS patterns for real-time control
  - minimal ISRs, defer heavy work to tasks
  - safe inter-task communications (task notifications, queues)
  - deterministic periodic control loop (sample → compute → actuate)
- implement a basic closed-loop motor controller
  - measure rotational speed (RPM) from encoder pulses
  - compute control action with PID (P / I / D)
  - convert PID output to PWM duty and apply it safely
- include safety and observability features so you can test on-bench
  - low startup duty, duty clamping, emergency disable
  - centralized logging via USB serial

### Implementing
- Encoder pulse capture (ISR)
- Atomic read-and-clear of encoder counts
- Periodic PID control task
- PWM setup + duty application
- Enable/disable button with ISR + task notification
- Centralized logger task + queue
- Safety measures
- Minimal, test-friendly defaults
- Clean separation of responsibilities


## Project 3 - DSHOT implementation (DSHOT PROJECT — focused)

### 3a - DSHOT150

Implement a valid DSHOT packets (checksum, telemetry bit).

#### Goal
- Generating the DSHOT waveform at DSHOT150 (lowest speed) by bit-banging with short, precise delays.
- Integrating that with FreeRTOS: a dedicated dshot_task that continuously sends frames, and a simple queue-based API to update throttle from other tasks (CLI, PID, etc.).
- Keeping the implementation simple and explicit so you can inspect/measure waveforms with a logic analyzer and learn where jitter arises.

#### Implementing
- Packet builder for the 16-bit DSHOT packet (11 bits throttle, 1 telemetry, 4 checksum).
- Bit-bang waveform generator for DSHOT150 with per-bit high/low timing derived from the baud (rounded to µs).
- FreeRTOS integration
  - dshot_task: repeatedly sends the most recent packet at a fixed frame rate (configurable).
  - xQueue for throttle updates: other tasks call dshot_set_throttle_percent() to update throttle atomically.
  - Use of taskENTER_CRITICAL()/taskEXIT_CRITICAL() while transmitting a frame to avoid interrupt-induced jitter (short critical section ≈ 110 µs).





## Project 3 - DSHOT implementation (DSHOT PROJECT — focused)

Implement a robust DSHOT ESC driver for the RP2040 (Pico-2 / RP2350) under FreeRTOS, starting from a simple, learnable implementation and iterating to a production-grade driver that supports high speeds and telemetry


### Goals
1. Build correct 16-bit DSHOT packet and send it bit-banged at DSHOT150.
   1. RTOS: single driver task or CLI triggers send; keep ISR minimal (none for send).
   2. Learn packet structure, measure waveform, verify CRC & bit encoding.
2. Offload waveform timing to PIO state machine. CPU just writes 16-bit packets (or preconverted pulse pattern) to PIO FIFO.
   1. RTOS: DSHOT driver task queues throttle updates → PIO send; driver uses a semaphore or DMA completion notification.
   2. Learning deterministic timing without blocking scheduler.
3. Use DMA to feed PIO or PWM hardware so a full frame (all bits / motors) is streamed without CPU intervention.
   1. RTOS: driver handles concurrent requests, DMA IRQ notifies driver task, scalable to many channels
4. Implement bidirectional timing: set line to input after send, capture ESC telemetry bits in correct window.
   1. Add arming, fail-safe, heartbeat watchdog, logging, and calibration


### Implementing

1. Bit-banged DSHOT150: a driver or CLI task that builds packets and calls a blocking send routine. Logging task for outputs. No interrupts required for sending. Timing depends on CPU + disabling interrupts; works only at low rates (DSHOT150 and maybe DSHOT300 on fast core if you disable interrupts), not robust for production.
   1. `build_dshot_packet(uint16_t throttle, bool telemetry)` — returns 16-bit packet (11-bit throttle, 1 telemetry, 4-bit checksum).
   2. `dshot_send_bitbanged(pin, packet, bitrate)` — toggles GPIO to encode MSB→LSB with timing approximated for given bitrate (DSHOT150).
   3. A FreeRTOS driver task that reads throttle commands from a queue and calls `dshot_send_bitbanged()`.
2. PIO-based DSHOT (robust unidirectional) (target: DSHOT300/600): in this implementation the critical section is moved from the CPU to the hw state machine, using PIO (programmable I/O coprocessor) that execute instructions at deterministic time.
3. PIO + DMA / multi-ESC DSHOT (high performance) (target: DSHOT600/1200 / multiple motors)
4. Telemetry & robust safety features (advanced, bidirectional)



| **Strategy**                            | **DSHOT Range Supported** | **Precision / Jitter**  | **CPU Load** | **Scalability (ESCs)** | **RTOS Compatibility**      | **Implementation Complexity** | **Hardware Requirements**        | **Notes / Typical Use**                               |
| --------------------------------------- | ------------------------- | ----------------------- | ------------ | ---------------------- | --------------------------- | ----------------------------- | -------------------------------- | ----------------------------------------------------- |
| **Bit-banged (GPIO toggling)**        | DSHOT150/300              | 🟡 Low (µs jitter)       | 🔴 High       | 1–2                    | ✅ Works, but preempts tasks | 🟢 Simple (pure software)      | None                             | Easy for demos, not real-time safe                    |
| **Timer / PWM ISR-based**             | DSHOT150/300              | Medium                  | Medium       | 2–4                    | ⚠️ Needs careful ISR timing  | 🟡 Moderate                    | Hardware timers                  | Can run in RTOS but risk of jitter if other tasks run |
| **PIO-only**                          | DSHOT150/600              | High                    | Low          | 4–6                    | ✅ Excellent (DMA-free)      | 🟢 Moderate                    | 1 PIO SM per ESC                 | Stable timing, minimal CPU usage                      |
| **PIO + DMA**                         | DSHOT150/1200             | Very High (ns accuracy) | 🟢 Very Low   | 6–8                    | ✅ Fully RTOS-safe           | 🔵 Complex (PIO + DMA sync)    | 1 PIO SM + 1 DMA channel per ESC | **Best balance** — high perf and low CPU cost         |



