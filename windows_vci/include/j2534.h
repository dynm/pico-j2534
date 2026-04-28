#pragma once

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STATUS_NOERROR 0x00
#define ERR_NOT_SUPPORTED 0x01
#define ERR_INVALID_CHANNEL_ID 0x02
#define ERR_INVALID_PROTOCOL_ID 0x03
#define ERR_NULL_PARAMETER 0x04
#define ERR_INVALID_IOCTL_VALUE 0x05
#define ERR_INVALID_FLAGS 0x06
#define ERR_FAILED 0x07
#define ERR_DEVICE_NOT_CONNECTED 0x08
#define ERR_TIMEOUT 0x09
#define ERR_INVALID_MSG 0x0A
#define ERR_INVALID_TIME_INTERVAL 0x0B
#define ERR_EXCEEDED_LIMIT 0x0C
#define ERR_INVALID_MSG_ID 0x0D
#define ERR_DEVICE_IN_USE 0x0E
#define ERR_INVALID_IOCTL_ID 0x0F
#define ERR_BUFFER_EMPTY 0x10
#define ERR_BUFFER_FULL 0x11
#define ERR_BUFFER_OVERFLOW 0x12
#define ERR_PIN_INVALID 0x13
#define ERR_CHANNEL_IN_USE 0x14
#define ERR_MSG_PROTOCOL_ID 0x15
#define ERR_INVALID_FILTER_ID 0x16
#define ERR_NO_FLOW_CONTROL 0x17
#define ERR_NOT_UNIQUE 0x18
#define ERR_INVALID_BAUDRATE 0x19
#define ERR_INVALID_DEVICE_ID 0x1A

#define CAN 0x00000005
#define ISO15765 0x00000006

#define CAN_29BIT_ID 0x00000100
#define ISO15765_FRAME_PAD 0x00000040

#define TX_MSG_TYPE 0x00000001
#define START_OF_MESSAGE 0x00000002
#define ISO15765_ADDR_TYPE 0x00000080

#define PASS_FILTER 0x00000001
#define BLOCK_FILTER 0x00000002
#define FLOW_CONTROL_FILTER 0x00000003

#define CLEAR_TX_BUFFER 0x00000007
#define CLEAR_RX_BUFFER 0x00000008
#define CLEAR_PERIODIC_MSGS 0x00000009
#define CLEAR_MSG_FILTERS 0x0000000A

typedef struct {
    unsigned long ProtocolID;
    unsigned long RxStatus;
    unsigned long TxFlags;
    unsigned long Timestamp;
    unsigned long DataSize;
    unsigned long ExtraDataIndex;
    unsigned char Data[4128];
} PASSTHRU_MSG;

typedef struct {
    unsigned long Parameter;
    unsigned long Value;
} SCONFIG;

typedef struct {
    unsigned long NumOfParams;
    SCONFIG* ConfigPtr;
} SCONFIG_LIST;

long WINAPI PassThruOpen(void* pName, unsigned long* pDeviceID);
long WINAPI PassThruClose(unsigned long DeviceID);
long WINAPI PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags, unsigned long Baudrate, unsigned long* pChannelID);
long WINAPI PassThruDisconnect(unsigned long ChannelID);
long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout);
long WINAPI PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout);
long WINAPI PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pMsgID, unsigned long TimeInterval);
long WINAPI PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID);
long WINAPI PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG* pMaskMsg, PASSTHRU_MSG* pPatternMsg, PASSTHRU_MSG* pFlowControlMsg, unsigned long* pFilterID);
long WINAPI PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID);
long WINAPI PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage);
long WINAPI PassThruReadVersion(unsigned long DeviceID, char* pFirmwareVersion, char* pDllVersion, char* pApiVersion);
long WINAPI PassThruGetLastError(char* pErrorDescription);
long WINAPI PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void* pInput, void* pOutput);

#ifdef __cplusplus
}
#endif
