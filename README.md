# wtBLEep

Firmware for the **wtBLEep** node — a dual-band Wi-Fi + sub-GHz sensor that broadcasts into
[Flying Flocks](https://github.com/real-CAK3D/FlyingFlocks). It speaks the public Biscuit
BLE/GATT protocol (so the app connects to it like any fleet board) and mirrors the same line
protocol over USB-CDC serial, so the app can drive it over BLE **or** a USB cable.

## Exact hardware

| Part | What |
| --- | --- |
| **Board** | **Seeed Studio XIAO ESP32-C5** (dual-band 2.4 / 5 GHz Wi-Fi) |
| **Sub-GHz radio** | **CC1101** on an **E07-M1101D** module (315 / 433 / 868 / 915 MHz) |
| **Advertised BLE name** | `wtBLEep` |
| **Status LED** | none — the firmware drives no LED, so a dark board is normal; it's alive if `wtBLEep` shows up in the app's scan |

> Flash this firmware **only** to a XIAO ESP32-C5 wired as below. It is not for the S3 / other
> Biscuit boards — they run their own firmware.

## What it does (one board, three jobs per sweep)

1. **Dual-band Wi-Fi scan** → `DATA:AP` (feeds the 5 GHz APs the 2.4 GHz-only boards can't see)
2. **Sub-GHz RSSI sweep** → `DATA:RF` (315 / 433 / 868 / 915 MHz band activity for the map)
3. **Sub-GHz raw decode** → `DATA:TPMS` / `DATA:WEATHER` / `DATA:KEYFOB` (tyre sensors, 433 weather
   stations, keyfobs / remotes) — needs the GDO0 data wire below

Compiled with `ALLOW_TX`, it also supports a gated OOK replay (`CMD:tx:<freq>,<bits>`) — **your own
devices only**; the app confirms ownership first.

## Wiring — CC1101 4-wire SPI + one data wire, all on the front XIAO C5 pads

| CC1101 / E07 | XIAO C5 pin | GPIO |
| --- | --- | --- |
| CSN | D0 | GPIO1 |
| SCK | D3 | GPIO7 |
| MOSI | D10 | GPIO10 |
| MISO | D8 | GPIO8 |
| GDO0 (E07 pin 3) | D2 | GPIO25 |
| VCC | 3V3 | — |
| GND | GND | — |

`GDO0` carries the raw demodulated data and is **required for TPMS / keyfob decode** (jobs 1 and 2
work without it; job 3 stays quiet). MOSI and GDO0 are on the front D10 / D2 pads because the C5's
GPIO2 / GPIO4 are underside JTAG pads that can't be soldered.

## BLE / GATT protocol

```
service  4fafc201-1fb5-459e-8fcc-c5c9c331914b   (advertised; name "wtBLEep")
CMD   w  beb5483e-36e1-4688-b7f5-ea07361b26a8   central -> device   "CMD:scanall:" etc.
RESP  n  beb5483e-36e1-4688-b7f5-ea07361b26a9   device -> central   "DATA:...\n"
STAT  r  beb5483e-36e1-4688-b7f5-ea07361b26aa   JSON health
```

## Build / flash

Arduino core: **esp32 by Espressif** (XIAO_ESP32C5 board). Needs the **ELECHOUSE CC1101** library.

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C5 .
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32C5 -p COM11 .
```

Replace `COM11` with the board's serial port.
