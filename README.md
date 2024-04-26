# SignalSnagger
Status: SignalSnagger is still in development. Product design and BOM contents are preliminary and incomplete.

SignalSnagger is an 80-meter band radio orienteering (ARDF) direction-finding receiver designed to operate in the Amateur Radio Service 3.5 MHz to 3.7 MHz frequency range. The receiver is designed to perform well in all common radio orienteering event formats: classic, sprint, and foxoring. Using a precision temperature-compensated signal generator, it is stable and accurate. A tuned loop with a "sense antenna," based on Dale Hunt's WB6BYU proven design, provides reliable signal-direction readings over a wide range of signal levels. This is an affordable, modern, competition-grade receiver design.

The mechanical design is optimized for simplicity, ruggedness, and ergonomic comfort. Only the front knob (attached to a rotary encoder) extends beyond the walls of the chassis box. Aside from wires running to the battery, there is no point-to-point wiring - all connections are handled by PCB traces and parts mounted to the circuit board. The headphone jack is on the bottom of the receiver, preventing water intrusion. The square antenna geometry makes for a stronger loop that can be readily fabricated using 3D printing. Switches for sense and configuration are recessed and immune to water penetration. If something breaks, all parts are readily available from online parts vendors, and schematic and BOM are provided, facilitating customer troubleshooting and repair.

The electronics design is optimized to avoid part obsolescence and to provide high performance. SignalSlinger uses a Software Defined Radio approach, utilizing a quadrature sampling detector to provide high dynamic range and to maximize software-upgradable features. A 25 MHz processor capable of sampling the baseband signal can filter and process the audio to achieve reliable signal strength indication and signal detection using DSP algorithms to achieve unparalleled SNR performance to detect weak signals deep in the noise.

The SignalSlinger project is 100% Open Source. All software and hardware design documents can be downloaded from this GitHub repository.

BOM: https://docs.google.com/spreadsheets/d/1Z3T1SeX6hfT8fjVtJgXu2S2YWHPFD_kywdSzAihOS8g/edit?usp=sharing

Also, check out SignalSnagger's sibling transmitter project: <a href="https://github.com/OpenARDF/SignalSlinger">SignalSlinger</a>.
