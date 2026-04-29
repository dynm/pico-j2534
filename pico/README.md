# RP2350 Pico J2534 Firmware

This firmware exposes the RP2350 as a WinUSB VCI and bridges USB bulk packets to an SPI CAN controller such as MCP2515.

## Hardware

Default SPI wiring is in `include/board_config.h`:

| Signal | GPIO |
| --- | --- |
| INT | 8 |
| CS | 9 |
| SCK | 10 |
| MOSI | 11 |
| MISO | 12 |
| LED | 25 |

Set `PICOJ_MCP2515_OSC_HZ` to match the crystal on the CAN board. The presets currently support 8 MHz and 16 MHz MCP2515 modules at 125 kbit/s, 250 kbit/s, and 500 kbit/s.

Status LED behavior:

- Slow blink: waiting for USB host.
- Fast blink: CAN controller/configuration error.
- Solid on: USB is connected and CAN is ready.
- Short flicker: USB or CAN traffic.

## Build

Install the Raspberry Pi Pico SDK and set `PICO_SDK_PATH`:

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S pico -B build/pico -DPICO_BOARD=pico2
cmake --build build/pico
```

Flash `build/pico/pico_j2534.uf2` to the RP2350 board.

GitHub Actions also builds the UF2 with the Pico SDK on Ubuntu. The workflow at `.github/workflows/build-pico-firmware.yml` runs for `v*` tags or manual dispatch, uploads `pico_j2534.uf2`, and attaches `pico_j2534-pico2-uf2.zip` to tag-backed GitHub Releases.

## USB

The firmware enumerates as vendor-specific VID `1209`, PID `2534` and publishes a Microsoft OS 2.0 descriptor with compatible ID `WINUSB`. Windows 10/11 should bind the inbox WinUSB driver automatically and create DeviceInterfaceGUID `{A9F78E2A-39A0-4A36-A6DF-6D80C96F54E1}` for the DLL.
