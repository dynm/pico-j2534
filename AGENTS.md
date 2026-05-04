# Repository Instructions

## Build Environment

- On macOS, build the Windows VCI with MinGW, not the host AppleClang Windows-incompatible path.
- Prefer the 32-bit MinGW target for DTS Monaco unless a task explicitly asks for x64.
- When locating the Pico SDK, check `$HOME/.pico-sdk` first before falling back to `PICO_SDK_PATH`, vendored SDKs under `build/`, or system locations.
- Keep generated build trees under `build/` and release artifacts under `dist/`.

## Build Outputs

- After a successful build, package the Windows VCI output as a zip in `dist/`.
- The `dist/` directory must also contain the Pico firmware UF2 from the same build.
- Do not leave release artifacts only in `build/`; copy or package them into `dist/` before considering the build complete.

## Firmware Deployment

- The Pico firmware USB protocol includes `PICOJ_CMD_BOOTLOADER`, which asks the device to reset into the UF2 bootloader.
- After building firmware, try to detect whether the Pico J2534 device is currently connected.
- If the device is present, send the USB bootloader command, wait for the Pico bootloader volume to appear, then flash the newly built UF2.
- If no device is present, leave the UF2 in `dist/` and report that flashing was skipped because hardware was not detected.

## macOS Build Flow

Use this order when implementing or updating build automation:

1. Find the Pico SDK, checking `$HOME/.pico-sdk` first.
2. Configure and build the Pico firmware.
3. Copy the firmware UF2 into `dist/`.
4. Configure and build the Windows VCI with a MinGW toolchain on macOS.
5. Zip the Windows VCI artifacts into `dist/`.
6. Probe for the connected device.
7. If present, reset it into bootloader mode and flash the UF2.

## Notes

- `macos_tools/uds_vin_test` already has a `--bootloader` path for sending the firmware bootloader command.
- Keep DTS Monaco compatibility centered on the Win32 J2534 DLL unless there is a clear reason to produce or test x64-only artifacts.
