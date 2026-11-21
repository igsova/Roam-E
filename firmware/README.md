# Wheel controller firmware

An Arduino Nano (ATmega328P, 16 MHz) runs the wheel velocity loop. The
Raspberry Pi sends wheel targets in encoder ticks per second over USB serial;
the Nano closes the loop on the encoders and streams telemetry back. The ROS
side of the link is `encoder_odometry_node` in
[`src/roam_e/roam_e/encoder_odometry_node.py`](../src/roam_e/roam_e/encoder_odometry_node.py).

| Path | Contents |
|---|---|
| [`roam_e_motor_controller/roam_e_motor_controller.ino`](roam_e_motor_controller/roam_e_motor_controller.ino) | Sketch: serial protocol, PI + feed-forward loop, watchdog, telemetry |
| [`roam_e_motor_controller/breakaway_assist.h`](roam_e_motor_controller/breakaway_assist.h) | Duty floor that ramps up until a stalled wheel starts moving |
| [`roam_e_motor_controller/bounded_startup.h`](roam_e_motor_controller/bounded_startup.h) | Opt-in paired straight-line start with travel and time budgets |
| [`build_avr.py`](build_avr.py) | Compiles the sketch with the Ubuntu AVR toolchain (no upload) |
| [`tests/`](tests/) | Host-side tests: the sketch compiled against a mocked Arduino API |

## Control loop

- **Velocity loop at 50 Hz.** Per wheel: PI on the filtered encoder speed plus
  a feed-forward term `Kv · target + static PWM`. The static PWM is set per
  wheel and per direction, because the two gearmotors need different duty to
  start moving. The integrator is clamped (anti-windup) and the output is
  limited to 230/255.
- **Breakaway assist.** Small in-place turns from Nav2 ask for less duty than
  it takes to overcome static friction. If a commanded wheel shows no encoder
  progress, a duty floor ramps up from 100 until it moves, for at most 500 ms
  per attempt.
- **Watchdog.** Motion commands must be refreshed within 350 ms, or both
  bridges are switched off and `EVENT,WATCHDOG_STOP` is sent. Malformed
  commands and overlong lines also stop the motors.
- **Telemetry at 20 Hz.**

## Wiring

| Signal | Nano pin |
|---|---|
| Left DRV8871 IN1 / IN2 | D10 / D5 |
| Right DRV8871 IN1 / IN2 | D9 / D6 |
| Left encoder A / B | D2 (interrupt) / D11 |
| Right encoder A / B | D3 (interrupt) / D12 |

The encoders are decoded on both edges of channel A: 748 ticks per wheel
revolution. Both wheels count forward motion as positive, and the right bridge
output is inverted in software to match the wiring.

## Serial protocol

115200 baud, ASCII lines ending in `\n`. On boot the Nano prints
`READY,ROAM_E_MOTOR_CONTROLLER,2`.

| Command | Reply | Effect |
|---|---|---|
| `VEL,<left>,<right>` | `OK,VEL,…` | Wheel targets in ticks/s (±2000) |
| `STOP` | `OK,STOP` | Stop both wheels |
| `PWM,<left>,<right>` | `OK,PWM,…` | Open-loop duty (±230), for bench tests |
| `PID,<kp>,<ki>,<kv>,<static>` | `OK,PID,…` | Loop gains in thousandths and one static PWM for all wheels, while stopped |
| `FF,<lf>,<lr>,<rf>,<rr>` | `OK,FF,…` | Static PWM per wheel and direction, while stopped |
| `START,0` / `START,1` | `OK,START,…` | Normal mode / opt-in bounded startup |
| `ENC`, `ZERO`, `INFO`, `CAPS`, `PING` | | Encoder counts, reset counts, limits, capabilities, liveness |

Errors are reported as `ERR,<reason>`: `RANGE`, `BAD_COMMAND`,
`LINE_TOO_LONG`, `MOTORS_ACTIVE` or `CONTROL_FAULT`.

Telemetry line:

```text
T,<ms>,<left_ticks>,<right_ticks>,<left_pwm>,<right_pwm>,<left_target>,<right_target>,<left_speed>,<right_speed>
```

## Build, flash and test

The host tests compile the sketch against [`tests/Arduino.h`](tests/Arduino.h),
a small mock of the Arduino API with a controllable clock, pins and encoder
counters. They then run each scenario: watchdog, stall and wrong-direction
detection, anti-windup, PWM ceiling, breakaway assist, and so on. They need a
C++17 compiler.

```bash
python3 -m unittest firmware/tests/test_firmware.py
```

To compile for the Nano on Ubuntu:

```bash
sudo apt install gcc-avr binutils-avr avr-libc arduino-core-avr
python3 firmware/build_avr.py --toolchain-root / --output-dir build/firmware
```

The firmware uses about 14 KB of flash and 475 bytes of RAM. Flash it with the
Arduino IDE, `arduino-cli upload`, or `avrdude` with the `.hex` from
`build/firmware/`. CI runs both the tests and the build on every push.
