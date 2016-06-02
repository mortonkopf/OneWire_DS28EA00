/**********************************************************************
* File: DS28EA00.h
* Programmer: Justin Jordan
* Updated: 03JAN14
*
* Description: header file for DS28EA00 1-wire temperature sensor
*
**********************************************************************/

#ifndef DS28EA00_h
#define DS28EA00_h

#define OW_BUS_PIN 2

//Control Function Commands
#define WRITE_SCRATCHPAD 0x4E
#define READ_SCRATCHPAD  0xBE
#define COPY_SCRATCHPAD  0x48
#define CONVERT_TEMP     0x44
#define READ_PWR_MODE    0xB4
#define RECALL_EEPROM    0xB8
#define PIO_ACCESS_READ  0xF5
#define PIO_ACCESS_WRITE 0xA5
#define CHAIN            0x99

//1-Wire ROM Function Commands
#define READ_ROM                0x33
#define MATCH_ROM               0x55
#define SEARCH_ROM              0xF0
#define CONDITIONAL_SEARCH_ROM  0xEC
#define CONDITIONAL_READ_ROM    0x0F
#define SKIP_ROM                0xCC
#define OVRDRV_SKIP_ROM         0x3C
#define OVRDRV_MATCH_ROM        0x69

//Chain States
#define CHAIN_OFF  0x3C
#define CHAIN_ON   0x5A
#define CHAIN_DONE 0x96

#define VALID_SEQUENCE 0xAA

//Temperature Alarm Values
#define LO_ALARM 0
#define HI_ALARM 0

//Configuration register values
#define RES_9_BIT   0x1F
#define RES_10_BIT  0x3F
#define RES_11_BIT  0x5F
#define RES_12_BIT  0x7F

#define NEGATIVE 1
#define POSITIVE 0

//PIO Assignements
#define BOTH_ON   0xFC 
#define PIOA_ON   0xFE //will shut off B
#define PIOB_ON   0xFD //will shut off A
#define BOTH_OFF  0xFF

typedef struct{
  unsigned char rom_code[8];
  unsigned char pwr_mode;
  int           raw_temp;
  unsigned char temp_hi_alm;
  unsigned char temp_lo_alm;
  unsigned char config_register;
  float         temperature;
  unsigned char pio_state;
} ds28ea00_t;
  

#endif


/**********************************************************************
* Copyright (C) 2013 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/
