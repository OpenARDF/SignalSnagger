set shell := ["bash", "-uc"]

# Fast, platform-neutral regression tests.
test:
    ./scripts/test-host.sh

# Reproducible firmware build using the repository's pinned AVR toolchain inputs.
avr-build:
    ./scripts/avr-build.sh

# Blocking local gate for firmware changes.
check: test avr-build
