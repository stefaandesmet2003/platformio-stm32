/**************************************************************************
 *  Copyright (C) 2008 - 2010 by Simon Qian                               *
 *  SimonQian@SimonQian.com                                               *
 *                                                                        *
 *  Project:    Versaloon                                                 *
 *  File:       SWIM.h                                                    *
 *  Author:     SimonQian                                                 *
 *  Versaion:   See changelog                                             *
 *  Purpose:    SWIM interface header file                                *
 *  License:    See license                                               *
 *------------------------------------------------------------------------*
 *  Change Log:                                                           *
 *      YYYY-MM-DD:     What(by Who)                                      *
 *      2008-11-07:     created(by SimonQian)                             *
 **************************************************************************/

#define VSFERR_NOT_READY              1
#define VSFERR_NONE                   0
#define VSFERR_NOT_SUPPORT            -1
#define VSFERR_NOT_AVAILABLE          -3
#define VSFERR_NOT_ACCESSABLE         -4
#define VSFERR_NOT_ENOUGH_RESOURCES   -5
#define VSFERR_FAIL                   -6
#define VSFERR_INVALID_PARAMETER      -7
#define VSFERR_INVALID_RANGE          -8
#define VSFERR_INVALID_PTR            -9
#define VSFERR_IO                     -10
#define VSFERR_BUG                    -11
#define VSFERR_UNKNOWN                -100

#define SWIM_BUFFERSIZE   1024 // for the async operation
// swim status codes
#define SWIM_OK           0x00
#define SWIM_BUSY         0x01
#define SWIM_NO_RESPONSE  0x04 // Target did not respond. SWIM not active?
#define SWIM_BAD_STATE    0x05 // ??


#define STATE_READY             0
#define STATE_SET_READ_ADDRESS  1
#define STATE_SET_WRITE_ADDRESS 2
#define STATE_READ_DATA         3
#define STATE_WRITE_DATA        4
#define STATE_ERROR             5 // no response or nack received, report SWIM_NO_RESPONSE to readStatus

typedef struct {
  uint8_t state;
  uint16_t totalBytes;    // total number of bytes to read/write in current request
  uint16_t curBytes;      // actual number of bytes read/written
  uint16_t otfEndBytes;   // idx where the current transaction stops (max 255 bytes per on-the-fly transaction)
  uint8_t *buffer;
  uint32_t address;
} swimStatusAsync_t;

// in async operation, the swim commands are executed asynchronously and the interface function returns immediately (stlink emulation)
// call swim_update() to update the swim state machine
int swim_init(bool isAsync); // F4-00
int swim_exit(void); // F4-01
int swim_setHighSpeed(bool highSpeed); // F4-03
int swim_doEntrySequence(void); // F4-04
int swim_srst(void); // F4-05
int swim_commsReset(void); // F4-06
int swim_assertReset(void); // F4-07 - pull SWIM_RST low
int swim_deassertReset(void); // F4-08 - release SWIM_RST, will be pulled high
int swim_readStatus(swimStatusAsync_t *status); // F4-09
int swim_wotf(uint32_t addr, uint16_t len, uint8_t *data); // F4 - 0A
int swim_rotf(uint32_t addr, uint16_t len, uint8_t *data); // F4 - 0B
// these functions exist only in the usb frontend
//int swim_readBuffer(uint8_t *data); // F4-0C
//int swim_getBufferSize(uint16_t *bufferSize); // F4-0D

void swim_update(void); // async operation
