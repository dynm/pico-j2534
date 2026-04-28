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
- `CAN_TX`: host sends `picoj_can_frame_t`; firmware transmits the CAN frame.
- `CAN_RX`: firmware sends unsolicited received CAN frames with sequence `0`.
- `STATUS`: firmware response with `picoj_status_t`.

The firmware exposes Microsoft OS 2.0 descriptors with DeviceInterfaceGUID `{A9F78E2A-39A0-4A36-A6DF-6D80C96F54E1}`. The Windows DLL discovers that interface with SetupAPI and uses WinUSB bulk reads/writes.
