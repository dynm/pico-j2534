# RP2350 Pico J2534 Windows VCI

This folder contains a Windows J2534-1 PassThru DLL for DTS Monaco and other J2534 clients.

The DLL talks to the RP2350 board with WinUSB bulk endpoints. The firmware exposes Microsoft OS 2.0 descriptors so Windows 10/11 can bind the inbox WinUSB driver automatically after first plug-in. The DLL supports one CAN channel and exposes `CAN` and `ISO15765`.

## Build

Use a Visual Studio Developer PowerShell:

```powershell
cmake -S windows_vci -B build/windows_vci -A Win32
cmake --build build/windows_vci --config Release
```

Many DTS Monaco installations are 32-bit, so build `Win32` first. If your Monaco install is 64-bit, build with `-A x64` and register the 64-bit DLL path.

You can also build the DLL with GitHub Actions. The workflow at `.github/workflows/build-windows-vci.yml` builds both `Win32` and `x64` on `windows-latest` and uploads matching `pico_j2534.dll` and `vin_smoke_test.exe` artifacts. On `v*` tag builds, it also attaches `pico_j2534-win32.zip` and `pico_j2534-x64.zip` to the GitHub Release.

## VIN Smoke Test

The `vin_smoke_test` tool dynamically loads `pico_j2534.dll` and can run in two modes:

```powershell
.\vin_smoke_test.exe --check-exports .\pico_j2534.dll
.\vin_smoke_test.exe .\pico_j2534.dll
```

`--check-exports` only verifies that the DLL loads and exposes the required J2534 entry points, so it runs in GitHub Actions without hardware. Without that flag, the tool opens the VCI, connects ISO15765 at 500 kbit/s, sends OBD-II service `09 02`, and prints the VIN response. Use the Win32 tool with the Win32 DLL and the x64 tool with the x64 DLL.

## Register for DTS Monaco

1. Build `pico_j2534.dll`.
2. Copy `register_pico_j2534.reg.in` to `register_pico_j2534.reg`.
3. Replace `@DLL_PATH@` with the absolute path to the built DLL, escaping backslashes, for example `C:\\VCI\\pico_j2534.dll`.
4. Import the file from an elevated prompt:

```powershell
reg import windows_vci\register_pico_j2534.reg
```

The template writes both native and `WOW6432Node` J2534 registry keys because Monaco is often a 32-bit process.

## Current Scope

The DLL is a practical starter VCI:

- WinUSB interface discovery through the firmware's Microsoft OS 2.0 descriptor.
- J2534 exports with undecorated names through `pico_j2534.def`.
- WinUSB interface discovery by DeviceInterfaceGUID `{A9F78E2A-39A0-4A36-A6DF-6D80C96F54E1}`.
- Classic CAN transmit/receive.
- Basic ISO-TP segmentation and reassembly for normal addressing.
- Stubbed acceptance for pass and flow-control filters so common J2534 clients can continue initialization.

ISO-TP flow-control timing is intentionally simple in this first version. For heavy flashing workloads, add full FC/BS/STmin handling in `src/isotp.cpp` and `src/j2534.cpp`.
