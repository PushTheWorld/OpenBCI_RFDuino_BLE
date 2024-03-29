/**
* Name: OpenBCI_Radio_Definitions.h
* Date: 3/15/2016
* Purpose: This is the header file for the OpenBCI radios definitions.
*
* Author: Push The World LLC (AJ Keller)
*   Much credit must also go to Joel Murphy who with Conor Russomanno and Leif
*     Percifield created the original OpenBCI_32bit_Device.ino and
*     OpenBCI_32bit_Host.ino files in the Summer of 2014. Much of this code base
*     is inspired directly from their work.
*/

#ifndef __OpenBCI_RFDuino_Definitions__
#define __OpenBCI_RFDuino_Definitions__

// These are helpful maximums to reference nad use in the code
#define RFDUINO_BLE_MAX_PACKET_SIZE_BYTES 20

#define OPENBCI_MAX_DATA_BYTES_IN_PACKET 31
#define OPENBCI_MAX_PACKET_SIZE_BYTES 32
#define OPENBCI_MAX_PACKET_SIZE_STREAM_BYTES 33

#define OPENBCI_TIMEOUT_PACKET_NRML_uS 500 // The time to wait before determining a multipart packet is ready to be send
#define OPENBCI_TIMEOUT_PACKET_STREAM_uS 88 // Slightly longer than it takes to send a serial byte at 115200
#define OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600 950 // Slightly longer than it takes to send a serial byte at 9600
#define OPENBCI_TIMEOUT_PACKET_POLL_MS 48 // Poll time out length for sending null packet from device to host
#define OPENBCI_TIMEOUT_COMMS_MS 270 // Comms failure time out length. Used only by Host.

#define BYTES_PER_SAMPLE 6
#define BYTES_PER_CHANNEL 3
#define POSITION_ACCEL_BYTE 19
#define BYTES_PER_ACCEL 6
#define NUM_BLE_PACKETS 10
#define BYTES_PER_BLE_PACKET 20
#define BYTES_PER_TINY_BUF 99
// Stream byte stuff
#define OPENBCI_STREAM_BYTE_START 0xA0
#define OPENBCI_STREAM_BYTE_STOP 0xC0
// 4101000000000000000000000000000000000000000000000000000000000000C04102000000000000000000000000000000000000000000000000000000000000C04103000000000000000000000000000000000000000000000000000000000000C0
// Max buffer lengths
#define OPENBCI_BUFFER_LENGTH_MULTI 100 // 16 * 33

// Number of buffers
#define OPENBCI_NUMBER_RADIO_BUFFERS 1
#define OPENBCI_NUMBER_SERIAL_BUFFERS 2

// These are the three different possible configuration modes for this library
#define OPENBCI_MODE_DEVICE 0
#define OPENBCI_MODE_HOST 1
#define OPENBCI_MODE_PASS_THRU 2

// Pins used by the Device
#define OPENBCI_PIN_DEVICE_PCG 5
// Pins used by the Host
#define OPENBCI_PIN_HOST_LED 2
#define OPENBCI_PIN_HOST_TIME 3
#define OPENBCI_PIN_HOST_RESET 6

// roles for the RFduinoGZLL
#define RFDUINOGZLL_ROLE_HOST HOST
#define RFDUINOGZLL_ROLE_DEVICE DEVICE0

// Channel limits
#define RFDUINOGZLL_CHANNEL_LIMIT_LOWER 0
#define RFDUINOGZLL_CHANNEL_LIMIT_UPPER 25

// flash memory address for RFdunioGZLL
#define RFDUINOGZLL_FLASH_MEM_ADDR 251

// Max number of packets on the TX buffer
#define RFDUINOGZLL_MAX_PACKETS_ON_TX_BUFFER 2

// Used to determine what to send after a proccess out bound buffer
#define ACTION_RADIO_SEND_NONE 0x00
#define ACTION_RADIO_SEND_NORMAL 0x01
#define ACTION_RADIO_SEND_SINGLE_CHAR 0x02

// Used to determine the result of processing a packet
#define OPENBCI_PROCESS_RADIO_FAIL_SWITCH_LAST      0x00
#define OPENBCI_PROCESS_RADIO_FAIL_SWITCH_NOT_LAST  0x01
#define OPENBCI_PROCESS_RADIO_FAIL_MISSED_LAST      0x02
#define OPENBCI_PROCESS_RADIO_FAIL_MISSED_NOT_LAST  0x03
#define OPENBCI_PROCESS_RADIO_PASS_LAST_MULTI       0x04
#define OPENBCI_PROCESS_RADIO_PASS_LAST_SINGLE      0x05
#define OPENBCI_PROCESS_RADIO_PASS_NOT_LAST_FIRST   0x06
#define OPENBCI_PROCESS_RADIO_PASS_NOT_LAST_MIDDLE  0x07
#define OPENBCI_PROCESS_RADIO_PASS_SWITCH_LAST      0x08
#define OPENBCI_PROCESS_RADIO_PASS_SWITCH_NOT_LAST  0x09

// Stream packet EOTs
#define OPENBCI_STREAM_PACKET_HEAD 0x41
#define OPENBCI_STREAM_PACKET_TAIL 0xC0

// Commands
#define OPENBCI_HOST_PRIVATE_CMD_KEY            0xF0
#define OPENBCI_HOST_CMD_CHANNEL_GET            0x00
#define OPENBCI_HOST_CMD_CHANNEL_SET            0x01
#define OPENBCI_HOST_CMD_CHANNEL_SET_OVERIDE    0x02
#define OPENBCI_HOST_CMD_POLL_TIME_GET          0x03
#define OPENBCI_HOST_CMD_POLL_TIME_SET          0x04
#define OPENBCI_HOST_CMD_BAUD_DEFAULT           0x05
#define OPENBCI_HOST_CMD_BAUD_FAST              0x06
#define OPENBCI_HOST_CMD_SYS_UP                 0x07
#define OPENBCI_HOST_CMD_TIME_PIN_HIGH          0x08
#define OPENBCI_HOST_CMD_TIME_PIN_LOW           0x09
#define OPENBCI_HOST_CMD_BAUD_HYPER             0x0A
#define OPENBCI_HOST_CMD_BAUD_BLE               0x0B

// Raw data packet types/codes
#define OPENBCI_PACKET_TYPE_RAW_AUX      = 3; // 0011
#define OPENBCI_PACKET_TYPE_STANDARD     = 0; // 0000
#define OPENBCI_PACKET_TYPE_TIME_SYNCED  = 1; // 0001
#define OPENBCI_PACKET_TYPE_USER_DEFINED = 2; // 0010

// Possible baud rates
#define OPENBCI_BAUD_RATE_DEFAULT 115200
#define OPENBCI_BAUD_RATE_BLE 9600

// Private Radio Places
#define OPENBCI_HOST_PRIVATE_POS_KEY 1
#define OPENBCI_HOST_PRIVATE_POS_CODE 2
#define OPENBCI_HOST_PRIVATE_POS_PAYLOAD 3

#endif
