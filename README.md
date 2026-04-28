# pico_j2534

> Important: WIP, don't use until this line is removed.

This repository contains a starter J2534 VCI split into the requested folders:

- `windows_vci`: Windows J2534 PassThru DLL source for DTS Monaco.
- `pico`: RP2350 firmware using WinUSB bulk endpoints and an SPI CAN controller.

The transport is WinUSB because HID polling is too slow for this use case. The firmware publishes Microsoft OS 2.0 descriptors so Windows can bind the inbox WinUSB driver, and the DLL is still required because DTS Monaco loads J2534 devices through the Windows PassThru registry and calls the exported `PassThru*` functions.

The default CAN controller target is MCP2515 over SPI. If your SPI CAN controller is a different chip, keep the USB packet protocol and replace the low-level driver in `pico/src/mcp2515.c`.
