# SignalSnagger AVR Build Environment

## Reference build

The deployed 0.38 firmware was built by Microchip Studio with these inputs:

- AVR-GCC 7.3.0 from Microchip AVR 8-bit GNU Toolchain 3.7.0.518;
- Atmel `AVR-Dx_DFP` 1.9.103;
- the source list in `Software/AVR128DA48/SignalSnagger/SignalSnagger.cppproj`;
- `-O0 -g2 -DDEBUG`, function/data sections, and section garbage collection.

The portable wrapper retains `-g2 -DDEBUG` but uses `-O2` for routine debugging
and hardware testing. Debug symbols remain available, while the 48 kHz capture
and 12 kHz DSP interrupt paths run with representative real-time performance.
Some source-level stepping and local-variable inspection are less exact than at
`-Og`; use breakpoints and debugger-visible counters for timing investigations.
Optimization or debug-setting changes remain separately reviewed firmware
changes, not incidental build-system cleanup.

Set `AVR_TOOLCHAIN_ROOT` to the extracted compiler directory and
`AVR_DFP_ROOT` to the extracted DFP version directory. The repository does not
silently download or replace toolchains. Verify the environment with:

```sh
just doctor
```

## Routine commands

```sh
just --list
just test
just avr-build
just check
just avr-repeatability
```

Products remain under the ignored `build/` directory. The portable wrapper
does not modify Microchip Studio's tracked `Debug` directory or its generated
makefiles.

The build gate reserves capacity for stack and future recovery work by limiting
linked flash use to 120,000 bytes and initialized plus zero-initialized static
RAM to 12,288 bytes. The linker still enforces the device's absolute limits.

## Release qualification

`just release-preflight` combines repository checks, a secret scan, and two
byte-for-byte identical builds. It does not replace live receiver testing. A
release still requires tuning, quadrature, gain, display, LED, headphone-power,
and control verification on representative hardware as described in
[RELEASE_WORKFLOW.md](RELEASE_WORKFLOW.md).
