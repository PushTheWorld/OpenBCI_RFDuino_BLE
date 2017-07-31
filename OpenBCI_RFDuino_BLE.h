/**
* Name: OpenBCI_Radio.h
* Date: 3/15/2016
* Purpose: This is the header file for the OpenBCI radios. Let us define two
*   over arching paradigms: Host and Device, where:
*     Host is connected to PC via USB VCP (FTDI).
*     Device is connectedd to uC (PIC32MX250F128B with UDB32-MX2-DIP).
*
* Author: Push The World LLC (AJ Keller)
*   Much credit must also go to Joel Murphy who with Conor Russomanno and Leif
*     Percifield created the original OpenBCI_32bit_Device.ino and
*     OpenBCI_32bit_Host.ino files in the Summer of 2014. Much of this code base
*     is inspired directly from their work.
*/


#ifndef __OpenBCI_Radios__
#define __OpenBCI_Radios__

#include <Arduino.h>
#include <RFduinoBLE.h>

// needed for enum and callback support
// #include "libRFduinoGZLL.h"
#include "OpenBCI_RFDuino_BLE_Definitions.h"

class OpenBCI_RFDuino_BLE_Class {

public:
    // ENUMS
    typedef enum STREAM_STATE {
        STREAM_STATE_INIT,
        STREAM_STATE_STORING,
        STREAM_STATE_TAIL,
        STREAM_STATE_READY
    };
    typedef enum STORE_STATE {
        STORE_STATE_INIT,
        STORE_STATE_STORING_FIRST,
        STORE_STATE_STORING_SECOND,
        STORE_STATE_STORING_THRID,
        STORE_STATE_READY
    };

    typedef enum BLE_PACKET_STATE {
        BLE_PACKET_STATE_INIT,
        BLE_PACKET_STATE_STORING_FIRST,
        BLE_PACKET_STATE_STORING_SECOND,
        BLE_PACKET_STATE_STORING_THRID,
        BLE_PACKET_STATE_GOT_ALL_PACKETS,
        BLE_PACKET_STATE_READY
    };

    // STRUCTS
    typedef struct {
        char      data[BYTES_PER_BLE_PACKET];
        uint8_t   positionRead;
        uint8_t   positionWrite;
    } PacketBuffer;

    typedef struct {
        boolean         overflowed;
        uint8_t         numberOfPacketsToSend;
        uint8_t         numberOfPacketsSent;
        PacketBuffer    packetBuffer[OPENBCI_NUMBER_SERIAL_BUFFERS];
    } Buffer;

    typedef struct {
        uint8_t         typeByte;
        char            data[OPENBCI_MAX_PACKET_SIZE_BYTES];
        uint8_t         bytesIn;
        boolean         flushing;
        STREAM_STATE    state;
    } StreamPacketBuffer;

    typedef struct {
        boolean flushing;
        boolean gotAllPackets;
        char    data[OPENBCI_BUFFER_LENGTH_MULTI];
        int     positionWrite;
        uint8_t previousPacketNumber;
    } BufferRadio;

    typedef struct {
      uint8_t data[BYTES_PER_BLE_PACKET];
      uint8_t bytesIn;
      BLE_PACKET_STATE state;
    } BLEPacket;

// SHARED
    OpenBCI_RFDuino_BLE_Class();
    void        begin();
    void        begin(uint32_t);
    void        beginDebug(uint32_t);
    void        blePacketReset();
    void        blePacketReset(BLEPacket *);
    void        bufferAddTimeSyncSentAck(void);
    void        bufferCleanChar(char *, int);
    void        bufferCleanCompleteBuffer(Buffer *, int);
    void        bufferCleanCompletePacketBuffer(PacketBuffer *, int);
    void        bufferCleanPacketBuffer(PacketBuffer *,int);
    void        bufferCleanBuffer(Buffer *, int);
    boolean     bufferRadioAddData(BufferRadio *, char *, int, boolean);
    void        bufferRadioClean(BufferRadio *);
    boolean     bufferRadioHasData(BufferRadio *);
    void        bufferRadioFlush(BufferRadio *);
    void        bufferRadioFlushBuffers(void);
    boolean     bufferRadioLoadingMultiPacket(BufferRadio *buf);
    byte        bufferRadioProcessPacket(char *data, int len);
    void        bufferRadioProcessSingle(BufferRadio *buf);
    boolean     bufferRadioReadyForNewPage(BufferRadio *buf);
    void        bufferRadioReset(BufferRadio *);
    boolean     bufferRadioSwitchToOtherBuffer(void);
    void        bufferResetStreamPacketBuffer(void);
    boolean     bufferSerialAddChar(char);
    boolean     bufferSerialHasData(void);
    void        bufferSerialProcessCommsFailure(void);
    void        bufferSerialReset(uint8_t);
    boolean     bufferSerialTimeout(void);
    void        bufferStreamAddChar(BLEPacket *, char);
    boolean     bufferStreamAddData(char *);
    void        bufferStreamFlushBuffers(void);
    boolean     bufferStreamReadyForNewPacket(StreamPacketBuffer *);
    boolean     bufferStreamReadyToSendToHost(StreamPacketBuffer *buf);
    void        bufferStreamReset(void);
    void        bufferStreamReset(StreamPacketBuffer *);
    boolean     bufferStreamSendToHost(StreamPacketBuffer *buf);
    void        bufferStreamStoreData(StreamPacketBuffer *, char *);
    boolean     bufferStreamTimeout(void);
    boolean     byteIdGetIsStream(uint8_t);
    int         byteIdGetPacketNumber(uint8_t);
    byte        byteIdGetStreamPacketType(uint8_t);
    char        byteIdMake(boolean, uint8_t, char *, uint8_t);
    byte        byteIdMakeStreamPacketType(uint8_t);
    boolean     commsFailureTimeout(void);
    void        configure(uint32_t);
    void        configureDevice(void);
    boolean     didPCSendDataToHost(void);
    boolean     didPicSendDeviceSerialData(void);
    boolean     flashNonVolatileMemory(void);
    uint32_t    getSecreteKey(void);
    uint32_t    getPollTime(void);
    boolean     hasStreamPacket(void);
    boolean     hostPacketToSend(void);
    boolean     isATailByte(uint8_t);
    void        ledFeedBackForPassThru(void);
    // void        moveStreamPacketToTempBuffer(volatile char *data);
    boolean     needToSetSecreteKey(void);
    boolean     needToSetPollTime(void);
    byte        outputGetStopByteFromByteId(char);
    void        pollHost(void);
    boolean     pollNow(void);
    boolean     packetToSend(void);
    boolean     packetsInSerialBuffer(void);
    void        pollRefresh(void);
    void        pushRadioBuffer(void);
    void        printBaudRateChangeTo(int);
    void        printSecreteKey(char);
    void        printSecreteKeyVerify(void);
    void        printCommsTimeout(void);
    void        printEOT(void);
    void        printFailure(void);
    void        printMessageToDriver(uint8_t);
    void        printPollTime(char);
    void        printSuccess(void);
    void        printValidatedCommsTimeout(void);
    void        processCommsFailureSinglePacket(void);
    boolean     processDeviceRadioCharData(char *, int);
    byte        processOutboundBuffer(PacketBuffer *);
    byte        processOutboundBufferCharDouble(char *);
    byte        processOutboundBufferCharTriple(char *);
    boolean     processOutboundBufferForTimeSync(void);
    boolean     processRadioCharDevice(char);
    void        resetPic32(void);
    boolean     revertToDefaultPollTime(void);
    void        revertToPreviousSecreteKey(void);
    boolean     sendPacketToConnectedDevice(void);
    void        sendPollMessageToHost(void);
    void        sendRadioMessageToHost(byte);
    void        sendStreamPackets(void);
    boolean     serialWriteTimeOut(void);
    void        setByteIdForPacketBuffer(int);
    boolean     setSecreteKey(uint32_t);
    boolean     setPollTime(uint32_t);
    void        writeBufferToSerial(char *,int);

    //////////////////////
    // SHARED VARIABLES //
    //////////////////////
    // CUSTOMS
    BLEPacket blePackets[NUM_BLE_PACKETS];
    StreamPacketBuffer spBuffer;
    BufferRadio bufferRadio[OPENBCI_NUMBER_RADIO_BUFFERS];
    uint8_t currentRadioBufferNum;
    BufferRadio *currentRadioBuffer;
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile boolean connectedDevice;
    Buffer bufferSerial;
    PacketBuffer *currentPacketBufferSerial;

    // BOOLEANS
    boolean debugMode;
    // CHARS
    char singleCharMsg[1];
    char singlePayLoad[1];

    volatile boolean sendingMultiPacket;
    volatile unsigned long timeOfLastPoll;

    volatile boolean isWaitingForNewSecreteKeyConfirmation;
    volatile boolean isWaitingForNewPollTimeConfirmation;
    volatile boolean sendSerialAck;
    volatile boolean printMessageToDriverFlag;
    volatile boolean systemUp;
    volatile boolean packetInTXRadioBuffer;

    STREAM_STATE curStreamState;

    uint8_t radioMode;
    volatile uint8_t msgToPrint;
    volatile uint8_t ackCounter;

    unsigned long lastTimeHostHeardFromDevice;
    volatile unsigned long lastTimeSerialRead;

    uint32_t secreteKey;
};

// Very important, major key to success #christmas
extern OpenBCI_RFDuino_BLE_Class radioBLE;

#endif // OPENBCI_RADIO_H
