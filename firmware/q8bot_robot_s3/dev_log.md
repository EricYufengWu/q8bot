# Dev Log

## 2026-03-27
### PowerManager module (`powerManager.h/.cpp`)
- New `PowerManager` class wrapping LATCH_PIN / SENSE_PIN / LED_PIN power latch logic
- **Power-on**: `begin()` blocks in `setup()` before RTOS tasks start; fades LED in over 3 s while SENSE_PIN held HIGH, then latches via LATCH_PIN HIGH. Returns false (no latch) if button released early.
- **Power-off**: `startMonitorTask()` spawns FreeRTOS task (priority 2, 2 kB stack). Detects 3 s hold on SENSE_PIN, fades LED out, pulls LATCH_PIN LOW. Aborts cleanly if released early.
- LED shared with `robotStateTask` — no mutex; power sequences intentionally stomp normal patterns.
- `millis()`-based timing (not tick-based) to avoid cumulative `vTaskDelay` drift over the 3 s window.
