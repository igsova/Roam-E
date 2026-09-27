# R17 power controls — mechanical provision, not a released wiring design

The assembly now provides positions for two purchased Hanibos KCD1-style
two-terminal latching ON/OFF rockers. Their smooth simplified Blender shapes
are references only; do not print the switches, terminals, screws, or nuts.

## Intended controls

- **BATTERY:** mounted on the removable drawer. Intended to control the battery
  supply, not an external USB-C supply.
- **WHEELS:** mounted above the right wheel on P16B, not on the drawer. Intended
  to isolate the motor-driver power branch while leaving the compute branch
  supplied. A software stop is not equivalent to electrical isolation.

These labels describe intended functions, not completed or tested wiring.
No firmware, electrical connections, or power settings were changed.

## The purchased switch rating still needs verification

User-provided listing: Hanibos, ON/OFF, 10 A / 125 V and 6 A / 250 V; the
photographed body explicitly marks these ratings **VAC**. No DC rating, exact
part-number datasheet, battery voltage, peak/stall current, or capacitive inrush
rating has been established. The title's “2 pole” is not evidence of DPST:
the pictured two terminals represent a single switched path.

**Do not infer 10 A DC capability from the AC label or connect either rocker
directly into the battery/motor supply on that basis.** DC arcing and inductive
loads require the appropriate switching rating. See the manufacturer's
[switch-load guidance](https://components.omron.com/us-en/products/basic-knowledge/switches/applications).

Before wiring, verify the exact switch's DC rating against maximum battery
voltage and the actual load/inrush. An alternative is a suitably engineered,
DC-rated power switching stage controlled by the rocker, but that needs its own
design, ratings, protection, and space review. A motor-driver sleep/enable input
alone does not meet a request for physical wheel-input isolation. Do not treat
these general-purpose controls as a safety-rated emergency stop.

Use appropriately rated source protection, insulated terminals and strain
relief. The model reserves space but does not select a connector or wire size.
The drawer requires a disconnect and service loop; avoid live exposed contacts,
pinched cables and a harness that can reach either tire. Disconnect the battery
and all external supplies before assembly or wiring work.

## Raspberry Pi 5 automatic boot

The Pi 5 normally starts when a valid supply is applied. In the official
[bootloader documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#WAIT_FOR_POWER_BUTTON),
`WAIT_FOR_POWER_BUTTON` defaults to `0`. Setting it to `1` together with
`POWER_OFF_ON_HALT=1` instead makes it wait for the power button after supply
power has been removed. No additional external latching switch across the
Pi's onboard power button is needed or appropriate for automatic boot.

To inspect a particular Pi without changing its configuration, run
`rpi-eeprom-config` **on that Pi** and review `WAIT_FOR_POWER_BUTTON`. This task
did not connect to the Pi or change its EEPROM. Auto-start after a real power
cycle is different from waking a halted Pi that still has supply power.

Use a graceful OS shutdown and wait for it to finish before cutting battery
power. Do not parallel the buck converter's 5 V output with another USB-C 5 V
supply unless the power path is specifically designed to isolate/select
sources. Prefer one validated Pi input supply at a time. If USB-C or another
path is still supplying the robot, BATTERY OFF is not necessarily robot OFF.

Sources checked 2026-09-25. Mechanical fit does not certify electrical safety.
