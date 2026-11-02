# SignalSnagger firmware safety invariants

## Interrupt-shared state

`volatile` prevents compiler caching; it does not make a multi-byte AVR access
atomic. The firmware uses these ownership rules:

| ISR | Shared state | Foreground rule |
| --- | --- | --- |
| `RTC_CNT_vect` | `g_powerdown_seconds` | Use `atomicReadU16` and `atomicWriteU16`. |
| `ADC0_RESRDY_vect` | direct DAC output, pending 12 kHz sample, dropped-frame/backlog counters, ADC heartbeat | Publish the pending sample before setting `g_audio_frame_ready`. Saturate the dropped-frame counter. After 64 consecutive drops, center the DAC and stop the ADC event/interrupt source before returning. |
| `TCA1_CMP0_vect` | `g_beep`, audio-cue state, audio-control slots, Goertzel sample state, DSP heartbeat | Copy the pending sample before clearing `g_audio_frame_ready`. Clear consecutive backlog only when consuming a frame. Write the inactive control slot completely before publishing its byte-sized index. Use `atomicWriteU16` for non-owner beep updates. Goertzel owns its double-buffer handoff. |
| `TCB0_INT_vect` | button events, `g_rotary_count`, `g_rssi_countdown`, headphone state, switch-service heartbeat | Exchange multi-byte events atomically; byte-sized flags may be read directly. Publish the heartbeat only after completing the ISR body. |
| `PORTF_PORT_vect` | `g_rotary_edges` | Consumed only by `TCB0_INT_vect`; same-level AVR ISRs do not nest. |
| `TCB1_INT_vect` | utility millisecond counter | Access only through the atomic helper in `tcb.cpp`. |
| `TCB2_INT_vect` | I2C timeout ticks | The ticks are byte-sized and therefore atomic on AVR. |
| `TCB3_INT_vect` | LED timeout and blink state | Access the 32-bit timeout atomically. `leds::blink` disables TCB3 while replacing 16-bit periods and counters. |

Foreground-only receiver frequencies and configuration values must not be
declared `volatile`. If a future ISR begins accessing one, add an explicit
atomic accessor or transfer event rather than relying on `volatile`.

## Runtime lockup containment

The hardware watchdog uses a two-second period. Foreground code may execute
`wdr` only after the 300 Hz switch service has advanced and, whenever audio is
expected, both ADC capture and 12 kHz DSP have advanced since the previous
feed. A foreground stall, UI/switch stall, ADC stall, or DSP stall therefore
leaves the watchdog unfed.

Sustained audio backlog has a faster circuit breaker. Sixty-four consecutive
unconsumed DSP frames center the DAC and stop the level-1 ADC source. The
resulting missing audio heartbeat deliberately causes a complete watchdog
reset rather than attempting partial recovery from interrupt context.

The lockout marker and dropped-frame count use a validated `.noinit` record so
they survive a watchdog reset without EEPROM wear. About diagnostics show
`D<drops> W<watchdog-reset>L<audio-lockout>`. The hardware reset flag must
accompany a valid marker pair before the lockout evidence is trusted.

Disable the watchdog only during startup initialization and intentional
standby. Re-arm it immediately after normal audio flow starts or resumes.

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
