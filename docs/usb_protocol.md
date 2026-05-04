# Pico J2534 USB Protocol

Each WinUSB bulk transfer carries one 64-byte `picoj_packet_t`. Full-speed USB bulk endpoints still use 64-byte packets, but they avoid HID's polling/report overhead and let WinUSB queue transfers more efficiently.

```c
typedef struct picoj_packet {
    uint16_t magic;   // 0x4A50
    uint8_t seq;      // Host request sequence; firmware echoes it in replies.
    uint8_t cmd;      // picoj_cmd_t
    uint8_t len;      // Payload bytes.
    uint8_t payload[59];
} picoj_packet_t;
```

Commands:

- `HELLO`: host probes firmware version and channel count.
- `SET_BITRATE`: host sends `picoj_bitrate_t`; firmware configures the SPI CAN controller.
- `CAN_TX`: host sends `picoj_can_frame_t`; firmware queues the CAN frame into an MCP2515 TX buffer and returns `STATUS`.
- `CAN_RX`: firmware sends unsolicited received CAN frames with sequence `0`.
- `STATUS`: firmware response with `picoj_status_t`.
- `CLEAR_RX`: host asks firmware to drop queued CAN receive frames and return `STATUS`.
- `ISOTP_CONFIG`: host sends `picoj_isotp_config_t` to configure request, response, and flow-control CAN IDs for firmware-side ISO-TP.
- `ISOTP_TX`: host sends one `picoj_isotp_chunk_t` payload chunk; firmware assembles chunks and starts `isotp-c` transmission when the full payload has arrived.
- `ISOTP_RX`: firmware sends unsolicited `picoj_isotp_chunk_t` chunks with sequence `0` after `isotp-c` reassembles a complete ISO-TP payload.
- `BOOTLOADER`: host asks firmware to return `STATUS` and reboot into the Pico USB UF2 bootloader.

`picoj_isotp_chunk_t` carries up to 49 payload bytes per USB packet. `total_len` is limited to 4095 bytes to match classic ISO-TP normal addressing. `CAN_TX` and `CAN_RX` remain available for raw J2534 `CAN` traffic; ISO15765 traffic uses the ISO-TP commands so CAN consecutive-frame timing stays on the Pico instead of the Windows DLL scheduler.

The firmware exposes Microsoft OS 2.0 descriptors with DeviceInterfaceGUID `{A9F78E2A-39A0-4A36-A6DF-6D80C96F54E1}`. The Windows DLL discovers that interface with SetupAPI and uses WinUSB bulk reads/writes.
