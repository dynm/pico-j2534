# RP2350 Pico J2534 Windows VCI

This folder contains a Windows J2534-1 PassThru DLL for DTS Monaco and other J2534 clients.

The DLL talks to the RP2350 board with WinUSB bulk endpoints. The firmware exposes a composite USB device with an unused CDC interface plus Microsoft OS 2.0 descriptors so Windows 10/11 can bind the inbox WinUSB driver automatically after first plug-in, without a custom INF. The DLL supports one physical CAN channel and exposes `CAN` and `ISO15765`.

## Build

Use a Visual Studio Developer PowerShell:

```powershell
cmake -S windows_vci -B build/windows_vci -A Win32
cmake --build build/windows_vci --config Release
```

DTS 9 uses the 32-bit J2534/PassThru path through Softing's D-PDU bridge. Build and register the `Win32` DLL for DTS 9 even on 64-bit Windows. Only use the `x64` build for a confirmed 64-bit J2534 client.

You can also build the DLL with GitHub Actions. The workflow at `.github/workflows/build-windows-vci.yml` builds both `Win32` and `x64` on `windows-latest` for `v*` tags or manual runs. It uploads matching `pico_j2534.dll` and `vin_smoke_test.exe` artifacts, and on tag builds attaches `pico_j2534-win32.zip` and `pico_j2534-x64.zip` to the GitHub Release.

## VIN Smoke Test

The `vin_smoke_test` tool dynamically loads `pico_j2534.dll` and can run in two modes:

```powershell
.\vin_smoke_test.exe --check-exports .\pico_j2534.dll
.\vin_smoke_test.exe .\pico_j2534.dll
```

`--check-exports` only verifies that the DLL loads and exposes the required J2534 entry points, so it runs in GitHub Actions without hardware. Without that flag, the tool opens the VCI, connects ISO15765 at 500 kbit/s, sends OBD-II service `09 02`, and prints the VIN response. Use the Win32 tool with the Win32 DLL and the x64 tool with the x64 DLL.

## Register for DTS Monaco

1. Build or download the `Win32` `pico_j2534.dll`.
2. Copy `register_pico_j2534.reg.in` to `register_pico_j2534.reg`.
3. Replace `@DLL_PATH@` with the absolute path to the 32-bit DLL, escaping backslashes, for example `C:\\VCI\\pico_j2534.dll`.
4. Import the file from an elevated prompt:

```powershell
reg import windows_vci\register_pico_j2534.reg
```

The template writes both native and `WOW6432Node` J2534 registry keys. For DTS 9, both entries should point to the same 32-bit DLL so the Softing D-PDU PassThru bridge cannot accidentally load an x64 DLL.

## Current Scope

The DLL is a practical starter VCI:

- WinUSB interface discovery through the firmware's Microsoft OS 2.0 descriptor.
- J2534 exports with undecorated names through `pico_j2534.def`.
- WinUSB interface discovery by DeviceInterfaceGUID `{A9F78E2A-39A0-4A36-A6DF-6D80C96F54E1}`.
- Classic CAN transmit/receive.
- J2534 `LOOPBACK` support for CAN transmit confirmations (`TX_MSG_TYPE` readback).
- Firmware-side ISO-TP segmentation, flow control, and reassembly for normal addressing via `isotp-c`.
- J2534 pass/block filters and ISO-TP flow-control filters for normal addressing.
- CAN and ISO15765 only. Legacy protocols, fake voltage readings, and programming-voltage no-ops are intentionally not exposed.

Classic CAN is intentionally kept as a first-class path because J2534 clients can open the `CAN` protocol directly, and it remains useful for bus checks, filters, and tools that do not use ISO15765. The DLL no longer contains its own ISO-TP transport state machine; ISO15765 payloads are moved between the DLL and firmware as USB chunks, while the Pico firmware handles CAN-frame timing with `isotp-c`.
