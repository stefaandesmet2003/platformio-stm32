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

#define VSFERR_NOT_READY				1
#define VSFERR_NONE						0
#define VSFERR_NOT_SUPPORT				-1
#define VSFERR_NOT_AVAILABLE			-3
#define VSFERR_NOT_ACCESSABLE			-4
#define VSFERR_NOT_ENOUGH_RESOURCES		-5
#define VSFERR_FAIL						-6
#define VSFERR_INVALID_PARAMETER		-7
#define VSFERR_INVALID_RANGE			-8
#define VSFERR_INVALID_PTR				-9
#define VSFERR_IO						-10
#define VSFERR_BUG						-11
#define VSFERR_UNKNOWN					-100

int swim_init(void); // F4-00
int swim_exit(void); // F4-01
int swim_setHighSpeed(bool highSpeed); // F4-03
int swim_doEntrySequence(void); // F4-04
int swim_srst(void); // F4-05
int swim_commsReset(void); // F4-06
int swim_assertReset(void); // F4-07 - pull SWIM_RST low
int swim_deassertReset(void); // F4-08 - release SWIM_RST, will be pulled high
int swim_readStatus(uint8_t *status); // F4-09
int swim_wotf(uint32_t addr, uint16_t len, uint8_t *data); // F4 - 0A
int swim_rotf(uint32_t addr, uint16_t len, uint8_t *data); // F4 - 0B
int swim_readBuffer(uint8_t *data); // F4-0C
int swim_getBufferSize(uint16_t *bufferSize); // F4-0D
