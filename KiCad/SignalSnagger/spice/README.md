# SignalSnagger receiver SPICE models

This directory contains the Stage 1 nominal models assigned to the KiCad
schematic from the receiver RF input through the headphone amplifier.  The
models target KiCad 9's ngspice backend and use `${KIPRJMOD}`-relative paths so
the project remains portable.

## Model status

| Schematic parts | Model | Basis | Stage 1 limitation |
| --- | --- | --- | --- |
| Q101-Q105 | `SS_SMMBT2222ALT1G` | 2N2222A-class Gummel-Poon model and onsemi device limits | Not yet correlated to a measured SMMBT2222ALT1G lot at 3.5-3.7 MHz |
| U104, U105 | `SS_DG9422E` | Vishay published on-resistance and nominal capacitance | Charge injection and control feedthrough omitted |
| RV101, RV302, RV901 | `SS_POT_10K`, `SS_POT_100K` | Schematic resistance values | Linear, nominal 50% wiper positions; sweep `W` for actual settings |
| T101 | `SS_PWB_4_CLC` | Coilcraft published inductance, DCR, 1:4 impedance ratio, and bandwidth | Coupling and capacitance are nominal; S-parameter fit is Stage 2 |
| U102 | `SS_PI5C3253` | Diodes Incorporated truth table, on-resistance, and capacitance | Behavioral switches omit nonlinear distortion and charge injection |
| U103 | `SS_REF4132B25` | TI 2.5 V part selected by the BOM/MPN and the circuit's mid-supply bias requirement | Nominal DC behavior only; startup/noise/load regulation are simplified |
| U302, U901 | `SS_TS912_DUAL` | ST TS912 limits (1 MHz GBP, rail-to-rail output, 65 ohm macro-model output resistance) | Behavioral surrogate; slew rate, noise, and output-current limiting are not complete |
| U201 | `SS_AVR128DA48_PASS_44K1` | Requested Stage 1 processor placeholder | Samples I/PD2 at 44.1 kHz, quantizes to 10 bits, and drives DAC0/PD6; Q/DSP/decimation are deferred |
| U202 | `SS_SI5351_QSD` | Receiver quadrature-clock requirement | Ideal 3.6 MHz quadrature clocks; programmed frequency and clock impairments are parameters for later sweeps |
| U304, U902 | `SS_LM386_1` | TI LM386-1 gain and bandwidth limits | Nominal gain-20 audio model; output power, clipping recovery, noise, and distortion need correlation |
| D301 | `SS_MSS1P4_M3` | Vishay ratings and typical Schottky behavior | Nominal detector/limiter model pending measured curve fit |

These are analysis aids, not substitutes for hardware measurements.  The
behavioral models are deliberately prefixed `SS_` to prevent them from being
mistaken for vendor-supplied macro-models.

Run `just spice-check` from the repository root to export the KiCad SPICE
netlist and verify that every Stage 1 receiver-path reference resolves to the
intended repository-local model. On the current macOS host, the check narrowly
waives KiCad's known Fontconfig configuration warnings during non-rendering
netlist export; all other exporter diagnostics fail the check.

## Reference sources

- onsemi, *MMBT2222LT1, SMMBT2222ALT1 General Purpose Transistors*:
  <https://www.onsemi.com/pdf/datasheet/mmbt2222lt1-d.pdf>
- Vishay, *DG9421/DG9422 Precision Low-Voltage CMOS Analog Switches*:
  <https://www.vishay.com/docs/70679/dg9421.pdf>
- Coilcraft, *PWB-4-CLC Wideband RF Transformer*:
  <https://www.coilcraft.com/en-us/products/transformers/wideband-rf-transformers/smt/pwb/pwb-4-c/>
- Diodes Incorporated, *PI5C3253 Dual 4:1 Mux/Demux Bus Switch*:
  <https://www.diodes.com/datasheet/download/PI5C3253.pdf>
- Texas Instruments, *REF4132 12-ppm/C Low-Noise Low-Power Precision Voltage Reference*:
  <https://www.ti.com/product/REF4132>
- STMicroelectronics, *TS912 rail-to-rail CMOS dual operational amplifier*:
  <https://www.st.com/resource/en/datasheet/ts912b.pdf>
- Texas Instruments, *LM386 Low Voltage Audio Power Amplifier*:
  <https://www.ti.com/lit/ds/symlink/lm386.pdf>
- Silicon Labs, *Si5351-B I2C-Programmable Any-Frequency CMOS Clock Generator*:
  <https://www.silabs.com/documents/public/data-sheets/Si5351-B.pdf>
- Microchip, *AVR128DA28/32/48/64 Data Sheet*:
  <https://www.microchip.com/en-us/product/AVR128DA48>

ST and TI publish fuller PSpice models for the TS912 and REF4132.  They are not
vendored here because Stage 1 must remain redistributable and ngspice-portable;
Stage 2 includes compatibility and license review before replacing the
surrogates.
