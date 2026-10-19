# SignalSnagger AVR Build Environment

## Reference build

SignalSnagger uses a portable macOS build wrapper for its AVR128DA48 firmware.
The reference inputs are:

- AVR-GCC 7.3.0 from Microchip AVR 8-bit GNU Toolchain 3.7.0.518;
- Atmel `AVR-Dx_DFP` 1.9.103;
- the source list in `Software/AVR128DA48/SignalSnagger/SignalSnagger.cppproj`;
- warning-as-error compilation and section garbage collection.

Set `AVR_TOOLCHAIN_ROOT` to the extracted compiler directory and
`AVR_DFP_ROOT` to the extracted DFP version directory. The repository does not
silently download or replace toolchains.

Verify the environment before building:

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

Build products remain under the ignored `build/` directory. The portable
wrapper does not modify Microchip Studio's generated `Debug` directory or its
vendor-managed makefiles.

## Release qualification

`just release-preflight` combines repository checks, a secret scan, and two
byte-for-byte identical AVR builds. It does not replace receiver hardware
testing. A release still requires live verification of tuning, quadrature
behavior, gain, display, and controls on representative hardware as described
in [RELEASE_WORKFLOW.md](RELEASE_WORKFLOW.md).
