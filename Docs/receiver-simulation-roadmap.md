# Receiver simulation roadmap

The goal is a reproducible model of the SignalSnagger receiver from the RF
antenna port to the headphone output.  Simulation results must distinguish
published component behavior, engineering assumptions, and hardware-correlated
measurements.

## Stage 1 - nominal end-to-end path

- Assign ngspice-compatible models to the active RF, quadrature detector,
  baseband, processor-passthrough, and audio-output parts.
- Use a 44.1 kHz, 10-bit sampled I-channel passthrough from the AVR ADC input to
  its DAC output.  Leave I/Q combination, decimation, and DSP filtering out.
- Preserve published package pin mappings in every assigned model.
- Resolve the voltage-reference identity as REF4132B25 (2.5 V), matching the
  BOM/MPN and the circuit's mid-supply bias function.
- Keep antenna excitation and component/model uncertainty explicit rather than
  presenting nominal models as measured receiver performance.
- Verify that every selected receiver-path part resolves to a model in an
  exported KiCad netlist; unrelated connector/mechanical placeholders are
  counted but do not block Stage 1 model assignment.

## Stage 2 - correlation and receiver-performance model

- Replace the nominal antenna port with measured loop/sense-antenna impedance,
  coupling, loss, and orientation-dependent signal sources.
- Fit T101 to Coilcraft S-parameters across the receiver band and include PCB
  interconnect parasitics.
- Validate vendor TS912 and REF4132 PSpice macro-models for ngspice compatibility
  and redistribution terms; use them when they improve verified accuracy.
- Correlate gain, bandwidth, noise figure, compression, intermodulation, switch
  feedthrough, and audio clipping against representative hardware measurements.
- Model component tolerances and temperature with corner and Monte Carlo runs.
- Add both I and Q ADC channels, exact sample timing, quantization, DAC behavior,
  decimation, filtering, demodulation, AGC, and other firmware DSP.
- Establish automated reference simulations and acceptance tolerances for
  sensitivity, selectivity, dynamic range, and audio output.
- Add a receiver-only simulation harness (or simulation variants) that excludes
  unrelated digital, connector, and mechanical placeholders from runnable
  end-to-end analyses.

Stage 2 is complete only when simulation-to-hardware differences are recorded
for named test conditions; a converged simulator run alone is not validation.
