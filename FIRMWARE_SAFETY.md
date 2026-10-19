# SignalSnagger firmware safety invariants

## Interrupt-shared state

`volatile` prevents compiler caching; it does not make a multi-byte AVR access
atomic. The firmware uses these ownership rules:

| ISR | Shared state | Foreground rule |
| --- | --- | --- |
| `RTC_CNT_vect` | `g_powerdown_seconds` | Use `atomicReadU16` and `atomicWriteU16`. |
| `ADC0_RESRDY_vect` | `g_beep`, Goertzel sample state | Use `atomicWriteU16` for foreground beep updates. Goertzel owns its double-buffer handoff. |
| `TCB0_INT_vect` | button events, `g_rotary_count`, `g_rssi_countdown`, headphone state | Exchange multi-byte events atomically; byte-sized flags may be read directly. |
| `PORTF_PORT_vect` | `g_rotary_edges` | Consumed only by `TCB0_INT_vect`; same-level AVR ISRs do not nest. |
| `TCB1_INT_vect` | utility millisecond counter | Access only through the atomic helper in `tcb.cpp`. |
| `TCB2_INT_vect` | I2C timeout ticks | The ticks are byte-sized and therefore atomic on AVR. |
| `TCB3_INT_vect` | LED timeout and blink state | Access the 32-bit timeout atomically. `leds::blink` disables TCB3 while replacing 16-bit periods and counters. |

Foreground-only receiver frequencies and configuration values must not be
declared `volatile`. If a future ISR begins accessing one, add an explicit
atomic accessor or transfer event rather than relying on `volatile`.

## Bounded hardware waits

Hardware polling must have a finite timeout and propagate failure to the
caller. `waitForStatusClear` is the shared foreground helper for byte-sized
peripheral status registers. Its timeout callback depends on timer interrupts,
so it must not be used with global interrupts disabled or from an ISR.

## Receiver clock boundary

All receiver entry points use `receiverFrequencyIsSupported`; 3.500000 MHz and
4.000000 MHz are valid inclusive limits. The experimentally established Si5351
quadrature function and register sequence are a protected hardware boundary.

## Hardware smoke-test menu

`RELEASE_WORKFLOW.md` defines which checks are required according to release
maturity and change risk. The complete functional smoke test is:

1. No 32 kHz or Si5351 startup error is displayed.
2. Startup and signal LEDs behave as in the released 0.38 firmware.
3. Tuning, gain, audio, and quadrature reception remain normal.
4. While editing a memory frequency, each upward and downward encoder step
   changes the received frequency before the menu is exited.
5. Removing headphones shows the full 60-second countdown and then powers down.
6. Reconnecting headphones starts the receiver normally.

For pre-1.0 development firmware, always test boot/version and the behavior
directly affected by the change; document every omitted item as not performed.
For versions `1.0.0` and later, complete every applicable item unless an
explicit waiver is recorded. Follow `RELEASE_WORKFLOW.md` for memory backup,
programming, and independent flash/configuration readback requirements.
