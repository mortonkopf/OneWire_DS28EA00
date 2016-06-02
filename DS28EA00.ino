/**********************************************************************
* File: DS28EA00
* Programmer: Maxim Integrated Systems Apps
* Started: 22JAN14
* Updated: 23JAN14
*
* Description: Demonstration of using the DS28EA00 1-wire devices.
*              Program supports up to 10 sensors on the bus.
*              Sensors can be added/removed from the bus while the 
*              program is running to test CHAIN feature of the DS28EA00
*
* This project requires the OneWire library which can be found at the
* following link - http://www.pjrc.com/teensy/td_libs_OneWire.html
*
* To make this library visible to the Arduino IDE follow the 
* instructions at the following link - http://arduino.cc/en/Guide/Libraries
*
* This project was developed using the following hardware:
* 1. Arduino UNO - http://arduino.cc/en/Main/ArduinoBoardUno
* 2. Osepp LCD Shield - http://osepp.com/products/shield-arduino-compatible/16x2-lcd-display-keypad-shield/
* 3. DS28EA00EVKIT - http://www.maximintegrated.com/datasheet/index.mvp/id/5489
*
* One of the EVKIT boards was modified to provide breadboard access to 
* the 1-wire data pin on the board.  The rest of the boards were 
* connected with the provided cables.
*
* The following is how the dip-switches on each DS28EA00EVKIT board 
* were configured:

* Board 1 - This board must be connected directly to the Arduino.
*   switch 1 - closed LEDA
*   switch 2 - closed LEDB
*   switch 3 - open   A1
*   switch 4 - open   A2
*   switch 5 - open   B EXT
*   switch 6 - closed B GND
*   switch 7 - closed Vcc
*
* Board 2 and 3 - The position of these boards is irrelevant.
*                 Changing the positions of these boards on the fly
*                 demonstrates the CHAIN feature of the device
*   switch 1 - closed LEDA
*   switch 2 - closed LEDB
*   switch 3 - open   A1
*   switch 4 - open   A2
*   switch 5 - closed B EXT
*   switch 6 - open   B GND
*   switch 7 - closed Vcc
* 
* The Osepp LCD Shield is not required for this project to work.
* Simply comment out the LCD relevant code (5 lines in setup, 4 lines 
* in the main loop) and make sure the function print_bus_data is 
* uncommented in the main loop.
**********************************************************************/

#include <LiquidCrystal.h>
#include <OneWire.h>
#include "DS28EA00.h"


/**********************************************************************
*Following is specific to the Osepp LCD Shield
**********************************************************************/

//select the pins used on the LCD Shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int lcd_key     = 0;
int adc_key_in  = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
/*********************************************************************/
 
//Set up ow_bus.  Pin can be changed in header file
OneWire ow_bus(OW_BUS_PIN);

//This is an array of structs
ds28ea00_t ds28ea00_array[10];

unsigned char num_ds28ea00_on_bus = 0;
unsigned char old_num_devices = 0;
unsigned char selected_ds28ea00 = 0;
unsigned char idx = 0;
unsigned char idy = 0;
unsigned char data;

void setup() 
{
  lcd.begin(16, 2);        // start the library
  lcd.setCursor(0,0);
  lcd.print("DS28EA00# ="); 
  lcd.setCursor(0, 2);
  lcd.print("Temp = ");
  Serial.begin(9600);
  delay(500);
  ds28ea00_detect_num_devices(ds28ea00_array);
  old_num_devices = num_ds28ea00_on_bus;
  ds28ea00_config_bus();
}

void loop() 
{
  //See if DS28EA00 sensors have been added/removed from bus
  ds28ea00_detect_num_devices(ds28ea00_array);
  
  //If they have, configure them
  if(old_num_devices != num_ds28ea00_on_bus)
  {
    ds28ea00_config_bus();
  }
  old_num_devices = num_ds28ea00_on_bus;
  
  //Test for user input
  if(read_LCD_buttons() == btnSELECT)
  {
    selected_ds28ea00++;
    
    if(selected_ds28ea00 >= num_ds28ea00_on_bus)
    {
      selected_ds28ea00 = 0;
    }
  }
  
  //Display Current Device
  lcd.setCursor(12, 0);
  lcd.print(selected_ds28ea00 + 1);
  
  //Convert the temperature on selected device
  ds28ea00_convert_temperature(selected_ds28ea00);
  
  //Display temperature
  lcd.setCursor(7, 1);
  lcd.print(ds28ea00_array[selected_ds28ea00].temperature);
  
  // comment the following if print_bus_data() is used
  delay(100);
  
  //The following prints the members of each devices struct to a rolling terminal
  //The array of structs updates dynamically when devices are added removed.
  
  print_bus_data();
}


/**********************************************************************
*Function Defenitions - The arduino ide creates function prototypes 
*                       when the sketch is compiled, therefore none are 
*                       included in the header file as would be done in
*                       a normal c or cpp project.  Please refer to the 
*                       following link for more details about the 
*                       arduino build process.
*                       http://www.arduino.cc/en/Hacking/BuildProcess
**********************************************************************/


/**********************************************************************
* Function: ds28ea00_detect_num_devices
* Parameters: ds28ea00_t *device_array - Pointer to an array of type 
*                                        ds28ea00_t
* Returns: none
*
* Description: Detects devices on the bus
**********************************************************************/
void ds28ea00_detect_num_devices(ds28ea00_t *device_array)
{
  int state = 0;
  
  do
  {
    state = ds28ea00_sequence_discoverey(device_array);
    if(state == -1)
    {
      Serial.println("Error!");
    }
  }
  while(state == -1);
  
  return;
}
/********** END ds28ea00_detect_num_devices****************************/


/**********************************************************************
* Function: ds28ea00_config_bus
* Parameters: none
* Returns: none
*
* Description: Populates structs of devices with desired configuration
               and writes to devices on bus
**********************************************************************/
void ds28ea00_config_bus(void)
{
  for(idx = 0; idx < num_ds28ea00_on_bus; idx++)
  {
    ds28ea00_get_pwr_mode(idx);
    
    //see header file for #defines of the following
    ds28ea00_array[idx].temp_lo_alm = LO_ALARM;
    ds28ea00_array[idx].temp_hi_alm = HI_ALARM;
    ds28ea00_array[idx].config_register = RES_12_BIT;
    
    ds28ea00_write_scratchpad(idx);
    ds28ea00_copy_scratchpad(idx);
    
    ds28ea00_convert_temperature(idx);
  }
  
  return;
}
/********** END ds28ea00_config_bus************************************/


/**********************************************************************
* Function: ds28ea00_sequence_discoverey
* Parameters: ds28ea00_t *device_array - Pointer to an array of type 
*                                        ds28ea00_t
* Returns: none
*
* Description: Detects devices on the bus and populates the 64-bit rom
*              codes of the device_array
**********************************************************************/
int ds28ea00_sequence_discoverey(ds28ea00_t *device_array)
{
  unsigned char test_end_of_bus;
  
  idx = 0;
  idy = 0;
  
  ow_bus.reset();
  ow_bus.skip();
  ow_bus.write(CHAIN);
  ow_bus.write(CHAIN_ON);
  ow_bus.write(~CHAIN_ON);
  
  data = ow_bus.read();
  if(data != VALID_SEQUENCE)
  {
    return(-1);
  }
  
  do
  {
    ow_bus.reset();
    ow_bus.write(CONDITIONAL_READ_ROM);
    
    test_end_of_bus = 0xFF;
    for(idy = 0; idy < 8; idy++)
    {
      data = ow_bus.read();
      test_end_of_bus &= data;
      device_array[idx].rom_code[idy] = data;
    }
    
    if(test_end_of_bus == 0xFF)
    {
      break;
    }
    
    idx++;
    num_ds28ea00_on_bus = idx;
    ow_bus.reset();
    ow_bus.write(PIO_ACCESS_WRITE);
    ow_bus.write(CHAIN);
    ow_bus.write(CHAIN_DONE);
    ow_bus.write(~CHAIN_DONE);
  
    data = ow_bus.read();
    if(data != VALID_SEQUENCE)
    {
      return(-1);
    }
  }
  while(test_end_of_bus != 0xFF);
  
  ow_bus.reset();
  ow_bus.skip();
  ow_bus.write(CHAIN);
  ow_bus.write(CHAIN_OFF);
  ow_bus.write(~CHAIN_OFF);
  
  data = ow_bus.read();
  if(data != VALID_SEQUENCE)
  {
    return(-1);
  }
  return(0);
}
/********** END ds28ea00_sequence_discoverey***************************/


/**********************************************************************
* Function: ds28ea00_get_pwr_mode
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Initiates a READ_PWR_MODE command with the selected 
*              ds28ea00 device and updates the power mode member of 
*              the device's struct.
**********************************************************************/
void ds28ea00_get_pwr_mode(unsigned char device)
{
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(READ_PWR_MODE);
  
  ds28ea00_array[device].pwr_mode = ow_bus.read();

  return;
}
/********** END ds28ea00_get_pwr_mode**********************************/


/**********************************************************************
* Function: ds28ea00_set_resolution
* Parameters: unsigned char device - ds28ea00 struct offset in array
*             unsigned char val - byte representing resolution of 
*                                 device
* Returns: none
*
* Description: writes 'val' to configuration register of 'device'
**********************************************************************/
void ds28ea00_set_resolution(unsigned char device, unsigned char val)
{
  ds28ea00_array[device].config_register = val;
  ds28ea00_write_scratchpad(device);
  ds28ea00_copy_scratchpad(device);
  
  return;
}
/********** END ds28ea00_set_resolution********************************/


/**********************************************************************
* Function: ds28ea00_set_hi_alarm
* Parameters: unsigned char device - ds28ea00 struct offset in array
*             unsigned char val - byte representing hi alarm
*
* Returns: none
*
* Description: writes 'val' to hi alarm register of 'device'
**********************************************************************/
void ds28ea00_set_hi_alarm(unsigned char device, unsigned char val)
{
  ds28ea00_array[device].temp_hi_alm = val;
  ds28ea00_write_scratchpad(device);
  ds28ea00_copy_scratchpad(device);
  
  return;
}
/********** END ds28ea00_set_hi_alarm*********************************/


/**********************************************************************
* Function: ds28ea00_set_lo_alarm
* Parameters: unsigned char device - ds28ea00 struct offset in array
*             unsigned char val - byte representing lo alarm
*
* Returns: none
*
* Description: writes 'val' to lo alarm register of 'device'
**********************************************************************/
void ds28ea00_set_lo_alarm(unsigned char device, unsigned char val)
{
  ds28ea00_array[device].temp_lo_alm = val;
  ds28ea00_write_scratchpad(device);
  ds28ea00_copy_scratchpad(device);
  
  return;
}
/********** END ds28ea00_set_lo_alarm*********************************/


/**********************************************************************
* Function: ds28ea00_read_scratchpad
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Reads the selected device's scratchpad
**********************************************************************/
void ds28ea00_read_scratchpad(unsigned char device)
{
  int temp_lo_byte;
  int temp_hi_byte;
  
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(READ_SCRATCHPAD);
  temp_lo_byte = ow_bus.read();
  temp_hi_byte = ow_bus.read();
  ds28ea00_array[device].temp_hi_alm = ow_bus.read();
  ds28ea00_array[device].temp_lo_alm = ow_bus.read();
  ds28ea00_array[device].config_register = ow_bus.read();
  //last three bytes of scratchpad reserved so we don't care
  ow_bus.reset();
  
  //build temp into single signed int
  ds28ea00_array[device].raw_temp = ((temp_hi_byte << 8) | temp_lo_byte);
  
  return;
}
/********** END ds28ea00_read_scratchpad*******************************/


/**********************************************************************
* Function: ds28ea00_write_scratchpad
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Writes the desired temperature alarms and configuration 
               register to the selected device's scratchpad
**********************************************************************/
void ds28ea00_write_scratchpad(unsigned char device)
{
  
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(WRITE_SCRATCHPAD);
  ow_bus.write(ds28ea00_array[device].temp_hi_alm);
  ow_bus.write(ds28ea00_array[device].temp_lo_alm);
  ow_bus.write(ds28ea00_array[device].config_register);
  ow_bus.reset();
  
  return;
}
/********** END ds28ea00_write_scratchpad******************************/


/**********************************************************************
* Function: ds28ea00_copy_scratchpad
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Copies scratchpad of selected device to its EEPROM
**********************************************************************/
void ds28ea00_copy_scratchpad(unsigned char device)
{
  
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(COPY_SCRATCHPAD);
  
  do
  {
    data = ow_bus.read();
  }
  while(data != 0xFF);
  
  return;
}
/********** END ds28ea00_copy_scratchpad*******************************/


/**********************************************************************
* Function: ds28ea00_convert_temperature
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Initiates a temperature conversion on selected device
**********************************************************************/
void ds28ea00_convert_temperature(unsigned char device)
{
  int temp_lo_byte;
  int temp_hi_byte;
  
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(CONVERT_TEMP);
  
  do
  {
    data = ow_bus.read();
  }
  while(data = 0);
  
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(READ_SCRATCHPAD);
  temp_lo_byte = ow_bus.read();
  temp_hi_byte = ow_bus.read();
  //we just want temp bytes so reset
  ow_bus.reset();
  
  //build temp into single signed int
  ds28ea00_array[device].raw_temp = ((temp_hi_byte << 8) | temp_lo_byte);
  
  ds28ea00_format_temperature(device);
  
  return;
}
/********** END ds28ea00_convert_temperature***************************/


/**********************************************************************
* Function: ds28ea00_format_temperature
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Formats the raw temperature conversion 
**********************************************************************/
void ds28ea00_format_temperature(unsigned char device)
{
  float f_temp = 0;
  int r_temp = ds28ea00_array[device].raw_temp;
  unsigned char sign = POSITIVE;
 
  if(r_temp & 0x8000)
  {
    sign = NEGATIVE;
    r_temp = ~r_temp + 1;
  }
  
  switch(ds28ea00_array[device].config_register)
  {
    case RES_9_BIT:
      r_temp = ((r_temp >> 3) & 0x1FF);
      f_temp = (r_temp >> 1);
      
      if(r_temp & 0x001)
      {
        f_temp += 0.5;
      }
      break;
      
    case RES_10_BIT: 
      r_temp = ((r_temp >> 2) & 0x3FF);
      f_temp = (r_temp >> 2);
      
      for(idy = 0; idy < 2; idy++)
      {
        if(r_temp & (1 << idy))
        {
          f_temp += (1.0/(1 << (2 - idy)));
        }
      }
      break;
      
    case RES_11_BIT:  
      r_temp = ((r_temp >> 1) & 0x7FF);
      f_temp = (r_temp >> 3);
      
      for(idy = 0; idy < 3; idy++)
      {
        if(r_temp & (1 << idy))
        {
          f_temp += (1.0/(1 << (3 - idy)));
        }
      }
      break;
      
    default :
      f_temp = (r_temp >> 4);
      
      for(idy = 0; idy < 4; idy++)
      {
        if(r_temp & (1 << idy))
        {
          f_temp += (1.0/(1 << (4 - idy)));
        }
      }
  }
  
  if(sign == NEGATIVE)
  {
    ds28ea00_array[device].temperature = (-1*f_temp);
  }
  else
  {
    ds28ea00_array[device].temperature = f_temp;
  }
  
 
  return;
}
/********** END ds28ea00_format_temperature***************************/


/**********************************************************************
* Function: ds28ea00_pio_read
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Reads the PIO status register of the selected device
**********************************************************************/
void ds28ea00_pio_read(unsigned char device)
{
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(PIO_ACCESS_READ);
  
  ds28ea00_array[device].pio_state = ow_bus.read();
  
  ow_bus.reset();
  
  return;
  
}
/********** END ds28ea00_pio_read**************************************/


/**********************************************************************
* Function: ds28ea00_pio_write
* Parameters: unsigned char device - ds28ea00 struct offset in array
* Returns: none
*
* Description: Writes to the PIO assignment register of the selected 
*              device
**********************************************************************/
int ds28ea00_pio_write(unsigned char device, unsigned char val)
{
  ow_bus.reset();
  ow_bus.select(ds28ea00_array[device].rom_code);
  ow_bus.write(PIO_ACCESS_WRITE);
  ow_bus.write(val);
  ow_bus.write(~val);
  
  data = ow_bus.read();
  if(data != VALID_SEQUENCE)
  {
    return(-1);
  }
 
  ds28ea00_array[device].pio_state = ow_bus.read();
  
  ow_bus.reset();
  
  return(0);  
}
/********** END ds28ea00_pio_write*************************************/


/**********************************************************************
* Function: print_bus_data
* Parameters: none
* Returns: none
*
* Description: Prints how many devices are on the bus and the members
*              of their structs.
**********************************************************************/
void print_bus_data(void)
{
  Serial.print("Number of DS28EA00 on bus = ");
  Serial.println(num_ds28ea00_on_bus);
  Serial.println("");
  
  for(idx = 0; idx < num_ds28ea00_on_bus; idx++)
  {
    Serial.print("DS28EA00 #");
    Serial.println(idx + 1);
    Serial.print("64-bit ROM code = ");
    
    for(idy = 0; idy < 8; idy++)
    {
      data = ds28ea00_array[idx].rom_code[idy];
      if(data > 0x0F)
      {
        Serial.print(data , HEX);
      }
      else
      {
        Serial.print(0);
        Serial.print(data , HEX);
      }
    }
    Serial.println("");
    
    Serial.print("Power Mode = ");
    Serial.println(ds28ea00_array[idx].pwr_mode, HEX);
    
    Serial.print("Raw Temperature = ");
    Serial.println(ds28ea00_array[idx].raw_temp, HEX);
    
    Serial.print("High Alarm = ");
    Serial.println(ds28ea00_array[idx].temp_hi_alm, HEX);
    
    Serial.print("Low Alarm = ");
    Serial.println(ds28ea00_array[idx].temp_lo_alm, HEX);
    
    Serial.print("Configuration Register = ");
    Serial.println(ds28ea00_array[idx].config_register, HEX);
    
    Serial.print("Temperature = ");
    Serial.println(ds28ea00_array[idx].temperature, HEX);

      Serial.print("Pin State = ");
    Serial.println(ds28ea00_array[idx].pio_state, HEX);
    
    Serial.println("");
  }
  
  delay(2500);
  
  return;
}
/********** END print_bus_data*****************************************/


/**********************************************************************
*Following is specific to the Osepp LCD Shield
**********************************************************************/

/**********************************************************************
* Function: read_LCD_buttons
* Parameters: none
* Returns: A int representing button pressed
*
* Description: Taken from Osepp LCD Shield example.  The shield uses a 
*              voltage divider network with different taps to represent
*              each button based on the analog voltage read in on A0.
**********************************************************************/
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);  
 
 //btnNone read first for speed since most likely case
 if (adc_key_in > 1000) return btnNONE; 
 if (adc_key_in < 50)   return btnRIGHT; 
 if (adc_key_in < 195)  return btnUP;
 if (adc_key_in < 380)  return btnDOWN;
 if (adc_key_in < 555)  return btnLEFT;
 if (adc_key_in < 790)  return btnSELECT;  
 
 return btnNONE;  // when all others fail, return this...
}
/********** END ow_receive_conformation********************************/


/**********************************************************************
* Function: 
* Parameters: 
* Returns: 
*
* Description: 
**********************************************************************/


/********** END function**********************************************/

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
