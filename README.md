# Pendel

Cart-pole (inverted pendulum) rig: a stepper-driven cart on a rail, a pendulum on a quadrature encoder, and a
Raspberry Pi 4 on a custom carrier PCB running a 500 Hz C++ controller (`pendctl`). The controller runs either on
the Pi (local mode) or on a PC over UDP with a constant delay (remote mode).

## Documentation

- [Theory and technical reference](docs/01_theory_and_technical_reference.md): the rig, the physics and model,
  PID and cascade control, latency, hardware and pinout, software architecture, the remote protocol.
- [Setup and operation guide](docs/02_setup_and_operation_guide.md): installing git and Python, getting the code,
  reaching the Pi, running local and remote mode safely, changing settings, troubleshooting, cheat sheet.

Also: `CONTROL_BRIEF.md` (the project brief), `CLAUDE.md` (rules, conventions and current status), and the dated
design notes and measurements in `docs/`.
