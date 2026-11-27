# Pool Heater Controller

An ESP32-S3 add-on that takes over thermostat control of a heat pump pool heater (Blue Torrent SHAS-32H-UC).

The heater's own water probe used to sit in a dry pocket in the heat exchanger, so the outdoor temperature affected when the heater turned on and off as much as the pool water did. In this project the probe is moved to the outside of the pool's steel wall, behind the insulation, and read by an ESP32-S3-Zero. The ESP32 decides when to heat and gives the heater controller one of two fixed resistances in place of the probe:

| ESP32 wants | Relay on | Heater controller sees | Heater display |
|---|---|---|---|
| heat | K2 (GP5) | COLD resistor, 9.0 kΩ | about 50 °F, calls for heat |
| no heat | K1 (GP6) | HOT resistor, 3.83 kΩ | about 89 °F, stays off |
| board dead | neither | open sensor | P1 fault, stops (fail-safe) |

The heater's own setpoint is fixed at 70 °F, midway between the two. The real control happens on the ESP32's web page.

## What's here

| Folder | Contents |
|---|---|
| `firmware/PoolHeater/` | Arduino sketch (`PoolHeater.ino`) and `secrets.example.h` |
| `kicad/` | KiCad 9 schematic and project |
| `docs/` | Build guide (`pool_heater_build.html`, open in a browser) and 1:1 perfboard drawings (print at 100 %) |

## Hardware

- Waveshare ESP32-S3-Zero on female headers
- 2 × Panasonic AQV252G PhotoMOS relays in sockets, each switching a fixed resistor across the heater's probe input
- Probe divider: 6.6 kΩ to 3.3 V, into GP1 through 1 kΩ with 100 nF to ground
- 18 × 24 hole perfboard, 5.08 mm screw terminals, CAT-5 runs to the probe and the heater
- Powered by its own 5 V USB-C supply

Full parts list, schematic, layout, wiring lists, calibration record and installation notes are in `docs/pool_heater_build.html`.

## Firmware

Features: web page at `http://poolheater.local` (setpoint, Neg Differential, confirm time, minimum off time, schedule, Auto / Force off / Force on), about 14 months of 5-minute temperature history in flash with a zoomable graph, calibration, Wi-Fi updates, watchdog, and Wi-Fi safeguards.

Control rules: the heater stops after every reading has been at or above the setpoint for the confirm time, and restarts after every reading has been at or below setpoint minus Neg Differential for the confirm time.

### First-time setup

1. Arduino IDE with the **esp32 by Espressif Systems** board package.
2. In `firmware/PoolHeater/`, copy `secrets.example.h` to `secrets.h` and fill in your Wi-Fi name, password and update password. `secrets.h` is in `.gitignore` and never uploaded.
3. Tools settings:
   - Board: **ESP32S3 Dev Module** (not S2, not "Octal")
   - USB CDC On Boot: **Enabled**
   - Flash Size: 4MB, PSRAM: QSPI PSRAM
   - Partition Scheme: **Default 4MB with spiffs** (holds the temperature history)
   - Erase All Flash Before Sketch Upload: **Disabled** (enabling it wipes settings and history)
4. Upload by USB the first time. After that, use Tools → Port → **poolheater** to upload over Wi-Fi.

Keep only one `.ino` file in the sketch folder: the Arduino IDE combines every `.ino` file in a folder, so stray copies cause "redefinition" errors.

## Probe calibration

Two-point calibration: 13,820 Ω at 32.0 °F (ice bath) and 2,890 Ω at 96.3 °F, giving B = 3696 and 4,444 Ω at 77 °F. These are built into the firmware as the default curve. After mounting, an installed offset (about +3 °F) is set on the web page with Calibrate → Match reading.
