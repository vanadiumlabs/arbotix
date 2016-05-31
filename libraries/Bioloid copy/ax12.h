/** @file 
    @brief DYNAMIXEL Library Header
   */ 
/*
  ax12.h - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>

#ifndef ax12_h
#define ax12_h

/** Maximum number of servos that can be controlled via this library**/
#define AX12_MAX_SERVOS             30

/** TX/RX packet buffer size**/
#define AX12_BUFFER_SIZE            32

/** Maximum number of registers for report**/
#define MAX_REGISTERS 80

/** Maximum position value for AX seros**/
#define AX_MAX_POSITION_VALUE 1023

/** Maximum position value for MX seros**/
#define MX_MAX_POSITION_VALUE 4095

/* Legacy Configuration */
#if defined(ARBOTIX)
  // no config needed
#elif defined(SERVO_STIK) || defined(ARBOTIX2)
  #define AX_RX_SWITCHED
  #define SET_RX_WR (PORTC |= 0x40)
  #define SET_AX_WR (PORTC |= 0x80)
  #define SET_RX_RD (PORTC = (PORTC & 0xBF) | 0x80)
  #define SET_AX_RD (PORTC = (PORTC & 0x7F) | 0x40)
  #define INIT_AX_RX DDRC |= 0xC0; PORTC |= 0xC0
#elif defined(ARBOTIX_1280)
  #define AX_RX_SWITCHED
  #define SET_RX_WR (PORTG |= 0x08)
  #define SET_AX_WR (PORTG |= 0x10)
  #define SET_RX_RD (PORTG = (PORTG&0xE7) | 0x10 )
  #define SET_AX_RD (PORTG = (PORTG&0xE7) | 0x08 )
  #define INIT_AX_RX DDRG |= 0x18; PORTG |= 0x18
#endif

/* End Legacy Configuration */

/* EEPROM REGISTER ADDRESSES */
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

/* MX Specific EEPROM registers */
#define MX_MULTI_TURN_OFFSET_L 20
#define MX_MULTI_TURN_OFFSET_H 21
#define MX_RESOLUTION_DIVIDER 22

/* RAM REGISTER ADDRESSES */
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

/* MX Specific RAM registers */
#define MX_P 28
#define MX_I 27
#define MX_D 26
#define MX_CURRENT_L 68
#define MX_CURRENT_H 69
#define MX_TORQUE_CONTROL_MODE 70
#define MX_GOAL_TORQUE_L 71
#define MX_GOAL_TORQUE_H 72
#define MX_GOAL_ACCELERATION 73



/* Status Return Levels */
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2


/* Instruction Set */
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

/* Error Levels */
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64

/* AX-S1, deprecated */
#define AX_LEFT_IR_DATA             26
#define AX_CENTER_IR_DATA           27
#define AX_RIGHT_IR_DATA            28
#define AX_LEFT_LUMINOSITY          29
#define AX_CENTER_LUMINOSITY        30
#define AX_RIGHT_LUMINOSITY         31
#define AX_OBSTACLE_DETECTION       32
#define AX_BUZZER_INDEX             40





/** Model Numbers **/
#define AX_12_MODEL_NUMBER 12
#define AX_18_MODEL_NUMBER 18
#define AX_12W_MODEL_NUMBER 44
#define MX_12W_MODEL_NUMBER 104
#define MX_28_MODEL_NUMBER 29
#define MX_64_MODEL_NUMBER 54
#define MX_106_MODEL_NUMBER 64

/** Broadcast address to send command to all servos **/
#define DXL_BROADCAST 254




/************************//**
 * @brief Macro to initialize servo library
 *
 * @param baud speed to start the DYNAMIXEL chain at
 *
 * @return none
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: start library at 1mbps (1000000 bps)
 * 
 * dxlInit(1000000);
 ******************************************/
void dxlInit(long baud);

 /************************//**
 * @brief helper functions to switch direction of comms
 *
 * @param id target servo id 
 * @todo (is id this actually needed?)
 * @return none
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * setTX();
 ******************************************/
void dxlSetTXall();     // for sync write

void dxlSetTX(int id);
 /************************//**
 * @brief helper functions to switch direction of comms
 *
 * @param id target servo id 
 * @todo (is id this actually needed?)
 * @return none
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * setRX();
 ******************************************/
void dxlSetRX(int id);


 /************************//**
 * @brief Sends a character out the serial port.
 *
 * @param data data to send
 * @return none
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * ax12write();
 ******************************************/

void dxlWrite(unsigned char data);
 /************************//**
 * @brief Sends a character out the serial port, and puts it in the tx_buffer
 *
 * @param data data to send
 * @return none
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * ax12writeB();
 ******************************************/
void dxlWriteB(unsigned char data);


/************************//**
 * @brief function to read DYNAMIXEL packet
 *
 * @param id the ID number for target servo
 * @param delayTime time (in milliseconds) that the servo should wait before returning data
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 return delay time on servo # 1 to 100ms
 *
 *  dxlSetReturnDelayTime(1,100);
 *******************************************/
int dxlReadPacket(int length);


/************************//**
 * @brief Function to get data from specified register on single servo
 *
 * @param id the ID number for target servo
 * @param regstart starting/lower register to start reading from
 * @param length how many registers to read. Usually 1 or 2
 *
 * @return error byte
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present position (register 36/37)
 *
 *  int position = dxlGetRegister(id, 36, 2);
 *******************************************/  
int dxlGetRegister(int id, int regstart, int length);

/************************//**
 * @brief Function to set a single register on the servo
 *
 * @param id the ID number for target servo
 * @param register the target register to set the data in
 * @param data  data for the register, values 0-255 (anything larger will be truncated)
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1's register #30 (goal position) to 512
 *
 *  dxlSetRegister(1,25, 1);
 *******************************************/
void dxlSetRegister(int id, int regstart, int data);

/************************//**
 * @brief Function to write a single value to 2 registers
 *
 * @param id the ID number for target servo
 * @param regstart starting/lower register. Data will be written accross regstart and regstart+1
 * @param data  data for the registers, values 0-65535
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * This function will do the necessary  conversions to conevert the 'data' value into 2 single bytes , and then write the bytes to the corresponding registers.
 * This function is usually used to write data for values that span 2 registers like goal position, goal speed, etc
 *
 * EXAMPLE: set servo # 1's register #30 (goal position) to 512
 *
 *  dxlSetRegister2(1,30, 512);
 *******************************************/
void dxlSetRegister2(int id, int regstart, int data);


/************************//**
 * @brief Function to get last error byte
 *
 * @param id the ID number for target servo
 *
 * @return error byte
 *
 * Compatible Servos: All AX/MX Servos
 *
 * By defualt, a DYNAMIXEL servo will return data after a read or write operation. 
 * Included in this data is an 'error' byte. Each bit of the byte indicates a different 
 * error. If all the bits are '0', the whole byte is 'zero' and there is no error.
 * Otherwise an error is present.
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error (ERR_INSTRUCTION)
 *
 * Bit 5 -  Overload Error (ERR_OVERLOAD)
 *
 * Bit 4 -  CheckSum Error (ERR_CHECKSUM)
 *
 * Bit 3 -  Range Error (ERR_RANGE)
 *
 * Bit 2 -  OverHeating Error (ERR_OVERHEATING)
 *
 * Bit 1 -  Angle Limit Error (ERR_ANGLE_LIMIT)
 *
 * Bit 0 -  Input Voltage Error (ERR_VOLTAGE)
 *
 *  EXAMPLE: get servo # 1's error byte
 *
 *  int position = dxlGetPosition(id);
 *  
 *  int error = ax12GetLastError() 
 *******************************************/  
int dxlGetLastError();

void axRegisterDump();
void mxRegisterDump();




float dxlGetSystemVoltage(int numberOfServos);
float dxlGetSystemVoltage(int numberOfServos, int servoList[]);
void dxlLedTest(int numberOfServos, int ledTime);
void dxlLedTest(int numberOfServos, int servoList[], int ledTime);



int dxlScanServos(int numberOfServos, int servoList[], int returnList[]);
int dxlScanServos(int numberOfServos, int returnList[]);
// int dxlScanServos(int servoList[]);
// int dxlScanServos(int numberOfServos);


void dxlServoReport();
void dxlServoReport(int numberOfServos);
void dxlServoReport(int numberOfServos, int servoList[]);

void dxlVoltageReport(int numberOfServos);
void dxlVoltageReport(int numberOfServos, int servoList[]);
void dxlRegisterReport(int servoNumber);

void dxlRegisterReportMultiple(int numberOfServos);
void dxlRegisterReportMultiple(int numberOfServos, int servoList[]);

void mxSetJointMode(int servoId);
void axSetJointMode(int servoId);
void dxlSetJointMode(int servoId, int CWAngleLimit, int CCWAngleLimit);
void dxlSetWheelMode(int servoId);
void mxSetMultiTurnMode(int servoId);



void mxBulkRead();

void dxlSycWrite();

void dxlReset(int id);

void dxlPing(int id);

void dxlAction(int id);

void dxlRegWrite2(int id, int regstart, int data);

void dxlRegWrite(int id, int regstart, int data);




//actual functions
// #define dxlSetJointMode()
// #define dxlSetWheeltMode()
// #define dxlSetMultiTurnMode()
// #define dxlGettMode()



extern unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
extern unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
extern unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];
#if defined(AX_RX_SWITCHED)
// Need to stow type of servo (which bus it's on)
extern unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif




/************************//**
 * @brief Macro to turn a single servo's LED on
 *
 * @param id the ID number for target servo
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE:  turn servo #1's LED on
 *
 *  dxlLEDOn(1);
 *******************************************/
#define dxlLEDOn(id) (dxlSetRegister(id, AX_LED, 1))

/************************//**
 * @brief Macro to turn a single servo's LED off
 *
 * @param id the ID number for target servo
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE:  turn servo #1's LED off
 *
 *  dxlLEDOff(1);
 *******************************************/
#define dxlLEDOff(id) (dxlSetRegister(id, AX_LED, 0))

/************************//**
 * @brief Macro to lock a single servo's EEPROM
 *
 * @param id the ID number for target servo
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: lock servo #1's EEPROM
 *
 *  dxlLockEEPROM(1);
 *******************************************/
#define dxlLockEEPROM(id) (dxlSetRegister(id, AX_LOCK, 1))



/************************//**
 * @brief Macro to turn the torque on for a single servo
 *
 * @param id the ID number for target servo
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1's torque to on
 *
 *  dxlTorqueOn(1);
 *******************************************/
#define dxlTorqueOn(id) (dxlSetRegister(id, AX_TORQUE_ENABLE, 1))


/************************//**
 * @brief Macro to turn the torque off for a single servo
 *
 * @param id the ID number for target servo
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1's torque to off
 *
 *  dxlTorqueOff(1);
 *******************************************/
#define dxlTorqueOff(id) (dxlSetRegister(id, AX_TORQUE_ENABLE, 0))


/************************//**
 * @brief Macro to set torqqe on for ALL servos
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 * 
 * This function uses the broadcast address to turn torque on to every connected servo
 *
 * EXAMPLE: turn on torque to all servos
 *
 *  dxlTorqueOnAll();
 *******************************************/
#define dxlTorqueOnAll() (dxlSetRegister(254, AX_TORQUE_ENABLE, 1))


/************************//**
 * @brief Macro to set torque off for ALL servos
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 * 
 * This function uses the broadcast address to turn torque off to every connected servo
 *
 * EXAMPLE: turn off torque to all servos
 *
 *  dxlTorqueOnAll();
 *******************************************/
#define dxlTorqueOffAll() (dxlSetRegister(254, AX_TORQUE_ENABLE, 0))



//     /$$$$$$              /$$           /$$$$$$$$ /$$$$$$$$ /$$$$$$$  /$$$$$$$   /$$$$$$  /$$      /$$       /$$$$$$$                      /$$             /$$                               /$$      /$$                                                  
//    /$$__  $$            | $$          | $$_____/| $$_____/| $$__  $$| $$__  $$ /$$__  $$| $$$    /$$$      | $$__  $$                    |__/            | $$                              | $$$    /$$$                                                  
//   | $$  \__/  /$$$$$$  /$$$$$$        | $$      | $$      | $$  \ $$| $$  \ $$| $$  \ $$| $$$$  /$$$$      | $$  \ $$  /$$$$$$   /$$$$$$  /$$  /$$$$$$$ /$$$$$$    /$$$$$$   /$$$$$$       | $$$$  /$$$$  /$$$$$$   /$$$$$$$  /$$$$$$   /$$$$$$   /$$$$$$$
//   |  $$$$$$  /$$__  $$|_  $$_/        | $$$$$   | $$$$$   | $$$$$$$/| $$$$$$$/| $$  | $$| $$ $$/$$ $$      | $$$$$$$/ /$$__  $$ /$$__  $$| $$ /$$_____/|_  $$_/   /$$__  $$ /$$__  $$      | $$ $$/$$ $$ |____  $$ /$$_____/ /$$__  $$ /$$__  $$ /$$_____/
//    \____  $$| $$$$$$$$  | $$          | $$__/   | $$__/   | $$____/ | $$__  $$| $$  | $$| $$  $$$| $$      | $$__  $$| $$$$$$$$| $$  \ $$| $$|  $$$$$$   | $$    | $$$$$$$$| $$  \__/      | $$  $$$| $$  /$$$$$$$| $$      | $$  \__/| $$  \ $$|  $$$$$$ 
//    /$$  \ $$| $$_____/  | $$ /$$      | $$      | $$      | $$      | $$  \ $$| $$  | $$| $$\  $ | $$      | $$  \ $$| $$_____/| $$  | $$| $$ \____  $$  | $$ /$$| $$_____/| $$            | $$\  $ | $$ /$$__  $$| $$      | $$      | $$  | $$ \____  $$
//   |  $$$$$$/|  $$$$$$$  |  $$$$/      | $$$$$$$$| $$$$$$$$| $$      | $$  | $$|  $$$$$$/| $$ \/  | $$      | $$  | $$|  $$$$$$$|  $$$$$$$| $$ /$$$$$$$/  |  $$$$/|  $$$$$$$| $$            | $$ \/  | $$|  $$$$$$$|  $$$$$$$| $$      |  $$$$$$/ /$$$$$$$/
//    \______/  \_______/   \___/        |________/|________/|__/      |__/  |__/ \______/ |__/     |__/      |__/  |__/ \_______/ \____  $$|__/|_______/    \___/   \_______/|__/            |__/     |__/ \_______/ \_______/|__/       \______/ |_______/ 
//                                                                                                                                 /$$  \ $$                                                                                                                 
//                                                                                                                                |  $$$$$$/                                                                                                                 
//                                                                                                                                 \______/                                                                                                                  


//i think this one needs specia
#define dxlSetId(oldId, newId) (dxlSetRegister(oldId, AX_ID, newId))
//special
#define dxlSetBaud(id, baud) (dxlSetRegister(id, AX_BAUD_RATE, baud))
//#define dxlSetBaud(id, baudVal) (dxlSetRegister(id, AX_GOAL_POSITION_L, pos))




/************************//**
 * @brief Macro to set the return delay time.
 *
 * @param id the ID number for target servo
 * @param delayTime time (in milliseconds) that the servo should wait before returning data
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 return delay time on servo # 1 to 100ms
 *
 *  dxlSetReturnDelayTime(1,100);
 *******************************************/
#define dxlSetReturnDelayTime(id, delayTime) (dxlSetRegister(id, AX_RETURN_DELAY_TIME, delayTime))


/************************//**
 * @brief Macro to set the Clockwise Angle Limit for the servo
 *
 * @param id the ID number for target servo
 * @param cwLimit clockwise angle limit (in native servo position)
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1  cw angle limit to 0
 *
 *  dxlSetCWAngleLimit(1,0);
 *******************************************/
#define dxlSetCWAngleLimit(id, cwLimit) (dxlSetRegister2(id, AX_CW_ANGLE_LIMIT_L, cwLimit))


/************************//**
 * @brief Macro to set the Counter Clockwise Angle Limit for the servo
 *
 * @param id the ID number for target servo
 * @param ccwLimit counter clockwise angle limit (in native servo position)
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1  cw angle limit to 1023
 *
 *  dxlSetCCWAngleLimit(1,1023);
 *******************************************/
#define dxlSetCCWAngleLimit(id, ccwLimit) (dxlSetRegister2(id, AX_CCW_ANGLE_LIMIT_L, ccwLimit))


/************************//**
 * @brief Macro to set the temperature limit for the servo
 *
 * @param id the ID number for target servo
 * @param temperature temperature limit in degrees celcius before servo goes into alarm mode
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1  temp limit to 50 degrees Celcius
 *
 *  dxlSetTempLimit(1,50);
 *******************************************/
#define dxlSetTempLimit(id, temperature) (dxlSetRegister(id, AX_LIMIT_TEMPERATURE, temperature))





/************************//**
 * @brief Macro to set the low voltage limit for the servo
 *
 * @param id the ID number for target servo
 * @param lowVoltage minimum voltage for servo before going into alarm mode. Value is (real voltage() * 10
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * @detailed 
 *
 *  EXAMPLE: set servo # 1 low voltage limit to 9.9 volts
 *
 *  dxlSetLowVoltage(1,99);
 *******************************************/
#define dxlSetLowVoltage(id, lowVoltage) (dxlSetRegister(id, AX_DOWN_LIMIT_VOLTAGE, lowVoltage))


/************************//**
 * @brief Macro to set the high voltage limit for the servo
 *
 * @param id the ID number for target servo
 * @param highVoltage maximum voltage for servo before going into alarm mode. Value is (real voltage() * 10
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 high voltage limit to 12.8 volts
 *
 *  dxlSetLowVoltage(1,128);
 *******************************************/
#define dxlSetHighVoltage(id, highVoltage) (dxlSetRegister(id, AX_UP_LIMIT_VOLTAGE, highVoltage))


/************************//**
 * @brief Macro to set startup torque limit limit for the servo
 *
 * @param id the ID number for target servo
 * @param torque startup torque limit. Value 0-1023, where each unit is about .1% of the max torque (i.e. 512 = 50%, 1023 = 100%)
 *
 * @return none  
 *
 * @detailed This function sets the EEPROM max torque. Every time the servo is reset, this value will get loaded into the RAM max torque. If the RAM torque is overwritten, that will take precedece
 *
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 torque limit to 74.9%
 *
 *  dxlSetStartupMaxTorque(1,767);
 *******************************************/
#define dxlSetStartupMaxTorque(id, torque) (dxlSetRegister2(id, AX_MAX_TORQUE_L, torque))


/************************//**
 * @brief Macro to set status return level for the servo
 *
 * @param id the ID number for target servo
 * @param statusReturnData status return level for the servo.
 *
 * 2 = return packet for all commands (AX_RETURN_ALL)
 *
 * 1 = returns for READ commands only. (AX_RETURN_READ)
 * 
 * 0 = no return packets except for PING (AX_RETURN_NONE)
 *
 * @return none  
 *
 * @detailed broadcast instructions (254) will never recieve return packets, regardless of what the status return level is set to
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 to only send return packets on PING
 *
 *  dxlSetStatusReturnLevel(1,0);
 *
 *******************************************/
#define dxlSetStatusReturnLevel(id, statusReturnData) (dxlSetRegister(id, AX_RETURN_LEVEL, statusReturnData))

/************************//**
 * @brief Macro to set which errors will trigger the alarm LED
 *
 * @param id the ID number for target servo
 * @param alarmData eacb bit of this value is mapped to a different error. If the error occurs and the bit is set to '1', then the LED will trigger when the error triggers 
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error
 *
 * Bit 5 -  Overload Error
 *
 * Bit 4 -  CheckSum Error
 *
 * Bit 3 -  Range Error
 *
 * Bit 2 -  OverHeating Error
 *
 * Bit 1 -  Angle Limit Error
 *
 * Bit 0 -  Input Voltage Error
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo LED alarm for all errors (0-6)
 *
 *  dxlSetAlarmLED(1,127);
 *******************************************/
#define dxlSetAlarmLED(id, alarmData) (dxlSetRegister(id, AX_ALARM_LED, alarmData))

/************************//**
 * @brief Macro to set which errors will trigger the alarm shutdown
 *
 * @param id the ID number for target servo
 * @param shutdownData each bit of this value is mapped to a different error. If the error occurs and the bit is set to '1', then the LED will trigger when the error triggers 
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error
 *
 * Bit 5 -  Overload Error
 *
 * Bit 4 -  CheckSum Error
 *
 * Bit 3 -  Range Error
 *
 * Bit 2 -  OverHeating Error
 *
 * Bit 1 -  Angle Limit Error
 *
 * Bit 0 -  Input Voltage Error
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set shutdown for overload and overheating only (2, 5)
 *
 *  dxlSetAlarmShutdown(1,36);
 *******************************************/
#define dxlSetAlarmShutdown(id, shutdownData) (dxlSetRegister(id, AX_ALARM_SHUTDOWN, shutdownData))

/************************//**
 * @brief Macro to set the multi-turn offset for multi turn mode
 *
 * @param id the ID number for target servo
 * @param offset offset from the actual present position. Can range from -24576 to 24576
 *
 * @return none  
 *
 * Compatible Servos: MX Servos Only
 *
 * EXAMPLE: set servo # 1 high voltage limit to 12.8 volts
 *
 *  mxSetMultiTurnOffset(1,128);
 *******************************************/
#define mxSetMultiTurnOffset(id, offset) (dxlSetRegister2(id, MX_MULTI_TURN_OFFSET_L, pos))

/************************//**
 * @brief Macro to set the resolution divider for multi turn mode
 *
 * @param id the ID number for target servo
 * @param divider value 1-4 to divide the resoltuion by. Lower resoltution yields more turns
 *
 * @return none  
 *
 * Present position = (Real Position / Resolution Divider) + Multi-turn Offse
 *
 * Compatible Servos: MX Servos Only
 *
 * EXAMPLE: set servo resolution divider to 4
 *
 *  mxSetResolutionDivider(1,4);
 *******************************************/
#define mxSetResolutionDivider(id, divider) (dxlSetRegister(id, MX_RESOLUTION_DIVIDER, pos))


//     /$$$$$$              /$$           /$$$$$$$   /$$$$$$  /$$      /$$       /$$$$$$$                      /$$             /$$                               /$$      /$$                                                  
//    /$$__  $$            | $$          | $$__  $$ /$$__  $$| $$$    /$$$      | $$__  $$                    |__/            | $$                              | $$$    /$$$                                                  
//   | $$  \__/  /$$$$$$  /$$$$$$        | $$  \ $$| $$  \ $$| $$$$  /$$$$      | $$  \ $$  /$$$$$$   /$$$$$$  /$$  /$$$$$$$ /$$$$$$    /$$$$$$   /$$$$$$       | $$$$  /$$$$  /$$$$$$   /$$$$$$$  /$$$$$$   /$$$$$$   /$$$$$$$
//   |  $$$$$$  /$$__  $$|_  $$_/        | $$$$$$$/| $$$$$$$$| $$ $$/$$ $$      | $$$$$$$/ /$$__  $$ /$$__  $$| $$ /$$_____/|_  $$_/   /$$__  $$ /$$__  $$      | $$ $$/$$ $$ |____  $$ /$$_____/ /$$__  $$ /$$__  $$ /$$_____/
//    \____  $$| $$$$$$$$  | $$          | $$__  $$| $$__  $$| $$  $$$| $$      | $$__  $$| $$$$$$$$| $$  \ $$| $$|  $$$$$$   | $$    | $$$$$$$$| $$  \__/      | $$  $$$| $$  /$$$$$$$| $$      | $$  \__/| $$  \ $$|  $$$$$$ 
//    /$$  \ $$| $$_____/  | $$ /$$      | $$  \ $$| $$  | $$| $$\  $ | $$      | $$  \ $$| $$_____/| $$  | $$| $$ \____  $$  | $$ /$$| $$_____/| $$            | $$\  $ | $$ /$$__  $$| $$      | $$      | $$  | $$ \____  $$
//   |  $$$$$$/|  $$$$$$$  |  $$$$/      | $$  | $$| $$  | $$| $$ \/  | $$      | $$  | $$|  $$$$$$$|  $$$$$$$| $$ /$$$$$$$/  |  $$$$/|  $$$$$$$| $$            | $$ \/  | $$|  $$$$$$$|  $$$$$$$| $$      |  $$$$$$/ /$$$$$$$/
//    \______/  \_______/   \___/        |__/  |__/|__/  |__/|__/     |__/      |__/  |__/ \_______/ \____  $$|__/|_______/    \___/   \_______/|__/            |__/     |__/ \_______/ \_______/|__/       \______/ |_______/ 
//                                                                                                   /$$  \ $$                                                                                                                 
//                                                                                                  |  $$$$$$/                                                                                                                 
//                                                                                                   \______/  



/************************//**
 * @brief Macro to enable/disable torque to the servo
 *
 * @param id the ID number for target servo
 * @param torqueState '0' turns the torque off, '1' enables it
 *
 * @return none  
 *
 * After torque is disabled and enabled, a positional / speed command must be issued to enage torque.
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo 1's torque to be off
 *
 *  dxlSetTorqueEnable(1,0);
 *******************************************/                                                                                                                
#define dxlSetTorqueEnable(id, torqueState) (dxlSetRegister(id, AX_TORQUE_ENABLE, torqueState))


/************************//**
 * @brief Macro to enable/disable the LED on the servo
 *
 * @param id the ID number for target servo
 * @param ledState '0' turns the LED off, '1' turns it on
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo 1's LED on
 *
 *  dxlSetLED(1,1);
 *******************************************/  
#define dxlSetLED(id, ledState) (dxlSetRegister(id, AX_LED, ledState))


/************************//**
 * @brief Macro to set the Counter Clockwise Complaince Margin
 *
 * @param id the ID number for target servo
 * @param ccwComplainceMargin Counter Clockwise Compliance Margin, positional values from 0-255
 *
 * @return none  
 *
 * Complaince describes how the motor will act when it tries to get to its goal position. 
 * A larger compliance margin means that the motor will stop at a further distance from its goal position
 * The counter clockwise compliance margin only applies to counter clockwise movements.
 *
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A
 *
 * Compatible Servos: AX Servos only
 *
 * EXAMPLE: set servo 1's CCW Coplaince margin to be 20 positions away from the goal
 *
 *  axSetCCWComplainceMargin(1,20);
 *******************************************/  
#define axSetCCWComplainceMargin(id, ccwComplainceMargin) (dxlSetRegister(id, AX_CCW_COMPLIANCE_MARGIN, ccwComplainceMargin))


/************************//**
 * @brief Macro to set the  Clockwise Complaince Margin
 *
 * @param id the ID number for target servo
 * @param cwComplainceMargin  Clockwise Compliance Margin, positional values from 0-255
 *
 * @return none  
 *
 * Complaince describes how the motor will act when it tries to get to its goal position. 
 * A larger compliance margin means that the motor will stop at a further distance from its goal position
 * The  clockwise compliance margin only applies to  clockwise movements.
 *
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A
 *
 * Compatible Servos: AX Servos only
 *
 * EXAMPLE: set servo 1's CW Coplaince margin to be 20 positions away from the goal
 *
 *  axSetCWComplainceMargin(1,20);
 *******************************************/ 
#define axSetCWComplainceMargin(id, cwComplainceMargin) (dxlSetRegister(id, AX_CW_COMPLIANCE_MARGIN, cwComplainceMargin))


/************************//**
 * @brief Macro to set the Counter Clockwise Complaince Margin
 *
 * @param id the ID number for target servo
 * @param CCWComplianceSlope Couter Clockwise Compliance Margin, positional values from 0-255
 *
 * @return none  
 *
 * Complaince describes how the motor will act when it tries to get to its goal position. 
 * A larger compliance slope means that the motor will be more flexible
 * The  counter clockwise compliance slope only applies to counter clockwise movements.
 *
 * Compliace slope is available in 7 steps [value range (actual compliance slope)]
 *
 * 0-3 (0x02)
 *
 * 4-7 (0x07)
 *
 * 8-15 (0x0F)
 * 
 * 16-31 (0x1F)
 *
 * 32-63 (0x3F)
 *
 * 64 - 127 (0x7F)
 *
 * 128 - 254 (0xFE)
 *
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A
 *
 * Compatible Servos: AX Servos only
 *
 * EXAMPLE: set servo 1's CCW Coplaince slope to be 128 
 *
 *  axSetCCWComplainceSlope(1,128);
 *******************************************/ 
#define axSetCCWComplainceSlope(id, CCWComplianceSlope) (dxlSetRegister(id, AX_CW_COMPLIANCE_SLOPE, CCWComplianceSlope))

/************************//**
 * @brief Macro to set the  Clockwise Complaince Margin
 *
 * @param id the ID number for target servo
 * @param CWComplianceSlope  Clockwise Compliance Margin, positional values from 0-255
 *
 * @return none  
 *
 * Complaince describes how the motor will act when it tries to get to its goal position. 
 * A larger compliance slope means that the motor will be more flexible
 * The  clockwise compliance slope only applies to  clockwise movements.
 *
 * Compliace slope is available in 7 steps [value range (actual compliance slope)]
 *
 * 0-3 (0x02)
 *
 * 4-7 (0x07)
 *
 * 8-15 (0x0F)
 * 
 * 16-31 (0x1F)
 *
 * 32-63 (0x3F)
 *
 * 64 - 127 (0x7F)
 *
 * 128 - 254 (0xFE)
 *
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A
 *
 * Compatible Servos: AX Servos only
 *
 * EXAMPLE: set servo 1's CW Coplaince slope to be 128 
 *
 *  axSetCWComplainceSlope(1,128);
 *******************************************/ 
#define axSetCWComplainceSlope(id, CWComplianceSlope) (dxlSetRegister(id, AX_CCW_COMPLIANCE_SLOPE, CWComplianceSlope))





/************************//**
 * @brief Macro to set the d(derivative) gain setting on the servo
 *
 * @param id the ID number for target servo
 * @param d derivative gain  for the servo, 0-254
 *
 * @return none  
 *
 * Compatible Servos: MX Servos Only
 *
 * http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1A
 *
 * Kd = d gain * 4 / 1000
 *
 * EXAMPLE: set servo # 1 d gain to 0
 *
 *  mxSetD(1,0);
 *******************************************/
#define mxSetD(id, d) (dxlSetRegister(id, MX_D, d))

/************************//**
 * @brief Macro to set the i(integral) gain setting on the servo
 *
 * @param id the ID number for target servo
 * @param i integral gain  for the servo, 0-254
 *
 * @return none  
 *
 * Compatible Servos: MX Servos Only
 *
 * http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1A
 *
 * Ki = i gain * 1000 / 2048
 *
 * EXAMPLE: set servo # 1 i gain to 0
 *
 *  mxSetI(1,0);
 *******************************************/
#define mxSetI(id, i) (dxlSetRegister(id, MX_I, i))

/************************//**
 * @brief Macro to set the p(proportional) gain setting on the servo
 *
 * @param id the ID number for target servo
 * @param p proportional gain  for the servo, 0-254
 *
 * @return none  
 *
 * Compatible Servos: MX Servos Only
 *
 * http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1A
 *
 * Kp = p gain / 8
 *
 * EXAMPLE: set servo # 1 derivative to 32
 *
 *  mxSetP(1,32);
 *******************************************/
#define mxSetP(id, p) (dxlSetRegister(id, MX_P, p))


/************************//**
 * @brief Macro to set the servo position
 *
 * @param id the ID number for target servo
 * @param pos servo position, 0-1023 for AX servos, 0-4096 for MX servos
 *
 * @return none  
 *
 * @deprecated this macro has been deprecated. Use dxlSetGoalPosition() instead
 *
 * Compatible Servos: All AX/MX Servos
 *
 * AX Position graphic http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
 * 
 * MX Position graphic http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1E
 *
 * EXAMPLE: set servo #1 position to 512
 *
 *  dxlSetLED(1,512);
 *******************************************/  
#define SetPosition(id, pos)             (dxlSetRegister2(id, AX_GOAL_POSITION_L, pos))



/************************//**
 * @brief Macro to set the servo position
 *
 * @param id the ID number for target servo
 * @param position servo position, 0-1023 for AX servos, 0-4096 for MX servos
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * AX Position graphic http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
 * 
 * MX Position graphic http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1E
 *
 * EXAMPLE: set servo #1 position to 512
 *
 *  dxlSetLED(1,512);
 *******************************************/  
#define dxlSetGoalPosition(id, position) (dxlSetRegister2(id, AX_GOAL_POSITION_L, position))



/************************//**
 * @brief Macro to set the goal speed
 *
 * @param id the ID number for target servo
 * @param speed servo speed. Values vary depending on servo mode
 *
 * @return none  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * Joint mode: speed can be from 0-1023, with each increment being about .111 RPM for AX servos and .114 RPM for MX servos. '0' means full speed. Any values above your servos maximum speed will be treated as the maximum speed
 * 
 * Wheel mode; speed can be from 0 - 2047, with each increment being about .1% of total output for AX servos, and .114RPM for MX servos
 * 
 *     Values from 0-1023 rotate the servo Counter clockwise with 0 being stopped and 1023 being full speed
 * 
 *     Values from 1024-2047 rotate the servo clockwise with 1024 being stopped and 2047 being full speed
 * 
 *     Essentially the 10nth bit becomes the direction control
 *
 *     AX servos only allow outut control as a percent. MX servos allow for speed based /rpm control
 *
 * EXAMPLE: set servo #1 position to ~10 RPM (AX servo, 10/.111 = 90)
 *
 *  dxlSetLED(1,90);
 *******************************************/  
#define dxlSetGoalSpeed(id, speed) (dxlSetRegister2(id, AX_GOAL_SPEED_L, speed))


/************************//**
 * @brief Macro to set running torque limit for the servo
 *
 * @param id the ID number for target servo
 * @param torque running torque limit. Value 0-1023, where each unit is about .1% of the max torque (i.e. 512 = 50%, 1023 = 100%)
 *
 * @return none  
 *
 * @detailed This function sets the RAM max torque. This function wil define the torque limit during actual servo operation
 *
 * Every time the servo is reset, the EEPROM value will get loaded into the RAM max torque, overwriting this value
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 torque limit to 74.9%
 *
 *  dxlSetRunningToruqeLimit(1,767);
 *******************************************/
#define dxlSetRunningToruqeLimit(id, torque) (dxlSetRegister2(id, AX_TORQUE_LIMIT_L, torque))


/************************//**
 * @brief Macro to set lock the EEPROM
 *
 * @param id the ID number for target servo
 * @param lock 0 = EEPROM can be modifed, 1 = EEPROM cannot be modified (it is locked)
 *
 * @return none  
 *
 * @detailed If this register is set to 1, then the servo must be reset in order to write to EEPROM again
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 lock to be on
 *
 *  dxlSetEEPROMLock(1,1);
 *******************************************/
#define dxlSetEEPROMLock(id, lock) (dxlSetRegister(id, AX_LOCK, lock))

/************************//**
 * @brief Macro to set servo punch
 *
 * @param id the ID number for target servo
 * @param punch   0x00 to 0x3FF for AX servos 0x20 to 0x3FF for MX servos
 *
 * @return none  
 *
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_20
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 punch to 32
 *
 *  dxlSetPunch(1,32);
 *******************************************/
#define dxlSetPunch(id, punch) (dxlSetRegister2(id, AX_PUNCH_L, punch))

/************************//**
 * @brief Macro to set the p(proportional) gain setting on the servo
 *
 * @param id the ID number for target servo
 * @param p proportional gain  for the servo, 0-254
 *
 * @return none  
 *
 * Compatible Servos: MX 64 and 106 only
 *
 * http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_1A
 *
 * Kp = p gain / 8
 *
 * EXAMPLE: set servo # 1 derivative to 32
 *
 *  mxSetP(1,32);
 *******************************************/
//documentation says you can write to this register, not sure that is true
//#define mxSetCurrent(id, current) (dxlSetRegister(id, MX_CURRENT_L, current))




/************************//**
 * @brief Macro to set the torque mode
 *
 * @param id the ID number for target servo
 * @param mode '0' = torque mode off (wheel/joint/multiturn rules) '1' = torque mode on (all other modes ignored)
 *
 * @return none  
 *
 * Compatible Servos: MX 64 and 106 only
 *
 * In this mode the servo does not respong to goal positions or moving speed, only the goal torque value. This mode is imilar to wheel mode in its appearance
 *
 * Kp = p gain / 8
 *
 * EXAMPLE: set servo # 1 to torque mode
 *
 *  mxSetTorqueControlMode(1,1);
 *******************************************/
#define mxSetTorqueControlMode(id, mode) (dxlSetRegister(id, MX_TORQUE_CONTROL_MODE, mode))

/************************//**
 * @brief Macro to set the goal torque 
 *
 * @param id the ID number for target servo
 * @param goalTorque  values 0-2047 where each increment is about 4.5mA (torque and current are proportionally related).
 *
 * @return none  
 *
 * Compatible Servos: MX 64 and 106 only
 *
 * In this mode the servo does not respong to goal positions or moving speed, only the goal torque value. This mode is imilar to wheel mode in its appearance
 * 
 *     Values from 0-1023 rotate the servo Counter clockwise with 0 being stopped and 1023 being full speed
 * 
 *     Values from 1024-2047 rotate the servo clockwise with 1024 being stopped and 2047 being full speed
 * 
 *     Essentially the 10nth bit becomes the direction control
 *
 *    Goal torque cannot exceed the max torque limit
 *
 *
 * EXAMPLE: set servo # 1 goal torque to ~50% counter clockwise
 *
 *  mxSetGoalTorque(1,512);
 *******************************************/
#define mxSetGoalTorque(id, goalTorque) (dxlSetRegister(id, MX_GOAL_L, goalTorque))


/************************//**
 * @brief Macro to set the goal torque 
 *
 * @param id the ID number for target servo
 * @param goalTorque  values 0-254  where each increment is ~8.583 Degree / sec^2. '0' means there is no acceleration control, the servo just moves as fast as possible
 *
 * @return none  
 *
 * Compatible Servos: All MX Servos
 *
 *
 *
 * EXAMPLE: set servo # 1 goal acceleration to 85.83 Degress/sec^2 (this would mean that after 1 second an ideal motor would go fom 0rpm to 14.3 rpm)
 *
 *  mxSetGoalTorque(1,10);
 *******************************************/
#define mxSetGoalAcceleration(id, goalAcceleration) (dxlSetRegister(id, MX_ACCELERATION, goalAcceleration))



//     /$$$$$$              /$$           /$$$$$$$$ /$$$$$$$$ /$$$$$$$  /$$$$$$$   /$$$$$$  /$$      /$$       /$$$$$$$                                           
//    /$$__  $$            | $$          | $$_____/| $$_____/| $$__  $$| $$__  $$ /$$__  $$| $$$    /$$$      | $$__  $$                                          
//   | $$  \__/  /$$$$$$  /$$$$$$        | $$      | $$      | $$  \ $$| $$  \ $$| $$  \ $$| $$$$  /$$$$      | $$  \ $$  /$$$$$$  /$$$$$$/$$$$                   
//   | $$ /$$$$ /$$__  $$|_  $$_/        | $$$$$   | $$$$$   | $$$$$$$/| $$$$$$$/| $$  | $$| $$ $$/$$ $$      | $$$$$$$/ |____  $$| $$_  $$_  $$                  
//   | $$|_  $$| $$$$$$$$  | $$          | $$__/   | $$__/   | $$____/ | $$__  $$| $$  | $$| $$  $$$| $$      | $$__  $$  /$$$$$$$| $$ \ $$ \ $$                  
//   | $$  \ $$| $$_____/  | $$ /$$      | $$      | $$      | $$      | $$  \ $$| $$  | $$| $$\  $ | $$      | $$  \ $$ /$$__  $$| $$ | $$ | $$                  
//   |  $$$$$$/|  $$$$$$$  |  $$$$/      | $$$$$$$$| $$$$$$$$| $$      | $$  | $$|  $$$$$$/| $$ \/  | $$      | $$  | $$|  $$$$$$$| $$ | $$ | $$                  
//    \______/  \_______/   \___/        |________/|________/|__/      |__/  |__/ \______/ |__/     |__/      |__/  |__/ \_______/|__/ |__/ |__/                  
//                                                                                                                                                                
//                                                                                                                                                                
//    


/************************//**
 * @brief Macro to get the model number for the servo
 *
 * @param id the ID number for target servo
 *
 * @return model number of the servo.  
 *
 * Servo Name - Model number value - define macro
 *
 * AX-12A - 12 - AX_12_MODEL_NUMBER
 *
 * AX-18A -18 - AX_18_MODEL_NUMBER 18
 *
 * AX-12W - 44 - AX_12W_MODEL_NUMBER 44
 *
 * MX-12W  - 104 -MX_12W_MODEL_NUMBER 104
 *
 * MX-28 - 29 - MX_28_MODEL_NUMBER 29
 *
 * MX-64  - 54 - MX_64_MODEL_NUMBER 54
 *
 * MX-106 - 64 - MX_106_MODEL_NUMBER 64
 *
 * Compatible Servos: All AX/MX Servos
 *
 * @detailed 
 *
 *  EXAMPLE: get servo # 1's model number
 *
 *  int modelNumber = dxlSetLowVoltage(id);
 *******************************************/                                                                                                                                                                                                                                                                                                          
#define dxlGetModel(id) (dxlGetRegister(id, AX_MODEL_NUMBER_L, 2))

/************************//**
 * @brief Macro to get the firmware version number for the servo
 *
 * @param id the ID number for target servo
 *
 * @return firmware version of the servo.  
 *
 * Compatible Servos: All AX/MX Servos
 *
 * @detailed 
 *
 *  EXAMPLE: get servo # 1's firmwre version
 *
 *  int firmwareVersion = dxlGetFirmware(id);
 *******************************************/   
#define dxlGetFirmware(id) (dxlGetRegister(id, AX_VERSION, 1))

/************************//**
 * @brief Macro to get the baud rate register for the servo
 *
 * @param id the ID number for target servo
 *
 * @return register data for baud rate (not the actual buad rate)
 *
 * @detailed The actual baud rate can be calculated by 
 *
 * Compatible Servos: All AX/MX Servos
 *
 * Speed(BPS)  = 2000000/(Data+1)
 *
 * where Data is the value in the register. 
 *
 * Common Baud Rates:
 *    1 = 1,000,000 bps (1MBPS)
 *    34 =  57142.9 (Target 57600);
 *
 *  EXAMPLE: get servo # 1's baud register data
 *
 *  int baud = dxlGetBaud(id);
 *******************************************/  
#define dxlGetBaud(id) (dxlGetRegister(id, AX_BAUD_RATE, 1))


/************************//**
 * @brief Macro to get the return delay time for the servo
 *
 * @param id the ID number for target servo
 *
 * @return time (in milliseconds) between servo receiving data and returning packet
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's return delay time
 *
 *  int returnTime = dxlGetReturnDelayTime(id);
 *******************************************/  
#define dxlGetReturnDelayTime(id) (dxlGetRegister(id, AX_RETURN_DELAY_TIME, 1))

/************************//**
 * @brief Macro to get the clockwise angle limit for the servo
 *
 * @param id the ID number for target servo
 *
 * @return clockwise angle limit (in native servo position)
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's Clockwise angle limit
 *
 *  int cwAngleLimit = dxlGetCWAngleLimit(id);
 *******************************************/  
#define dxlGetCWAngleLimit(id) (dxlGetRegister(id, AX_CW_ANGLE_LIMIT_L, 2))

/************************//**
 * @brief Macro to get the counter-clockwise angle limit for the servo
 *
 * @param id the ID number for target servo
 *
 * @return counter-clockwise angle limit (in native servo position)
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's counter-clockwise angle limit
 *
 *  int ccwAngleLimit = dxlGetCCWAngleLimit(id);
 *******************************************/  
#define dxlGetCCWAngleLimit(id) (dxlGetRegister(id, AX_CCW_ANGLE_LIMIT_L, 2))

/************************//**
 * @brief Macro to get the temperature limit for the servo
 *
 * @param id the ID number for target servo
 *
 * @return temperature limit in degreees celcius
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's counter-clockwise angle limit
 *
 *  int tempLimit = dxlGetTempLimit(id);
 *******************************************/  
#define dxlGetTempLimit(id) (dxlGetRegister(id, AX_LIMIT_TEMPERATURE, 1))

/************************//**
 * @brief Macro to get the low voltage limit for the servo
 *
 * @param id the ID number for target servo
 *
 * @return low voltage limit, voltage is the register value/ 10 (ie. 99 = 9.9v)
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's lower limit voltage
 *
 *  int lowVoltage = dxlGetLowVoltage(id) / 10;
 *******************************************/  
#define dxlGetLowVoltage(id) (dxlGetRegister(id, AX_DOWN_LIMIT_VOLTAGE, 1))

/************************//**
 * @brief Macro to get the high voltage limit for the servo
 *
 * @param id the ID number for target servo
 *
 * @return high voltage limit, voltage is the register value/ 10 (ie. 128 = 12.8v)
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's high limit voltage
 *
 *  int highVoltage = dxlGetHighVoltage(id) / 10;
 *******************************************/  
#define dxlGetHighVoltage(id) (dxlGetRegister(id, AX_UP_LIMIT_VOLTAGE, 1))

/************************//**
 * @brief Macro to get the startup torque for the servo
 *
 * @param id the ID number for target servo
 *
 * @return startup torque limit. Value 0-1023, where each unit is about .1% of the max torque (i.e. 512 = 50%, 1023 = 100%)
 *
 * Compatible Servos: All AX/MX Servos
 *
 * @detailed This function gets the EEPROM max torque. Every time the servo is reset, this value will get loaded into the RAM max torque. If the RAM torque is overwritten, that will take precedece
 *
 *  EXAMPLE: get servo # 1's startup torque value
 *
 *  int startupTorque = dxlGetStartupMaxTorque(id) 
 *******************************************/  
#define dxlGetStartupMaxTorque(id) (dxlGetRegister(id, AX_MAX_TORQUE_L, 2))

/************************//**
 * @brief Macro to get the startup torque for the servo
 *
 * @param id the ID number for target servo
 *
 * @return status return level for the servo.
 *
 * Compatible Servos: All AX/MX Servos
 *
 * 2 = return packet for all commands (AX_RETURN_ALL)
 *
 * 1 = returns for READ commands only. (AX_RETURN_READ)
 * 
 * 0 = no return packets except for PING (AX_RETURN_NONE)
 *
 * @detailed This function gets the EEPROM max torque. Every time the servo is reset, this value will get loaded into the RAM max torque. If the RAM torque is overwritten, that will take precedece
 *
 *  EXAMPLE: get servo # 1's startup torque value
 *
 *  int statusReturnLevel = dxlGetStatusReturnLevel(id) 
 *******************************************/  
#define dxlGetStatusReturnLevel(id) (dxlGetRegister(id, AX_RETURN_LEVEL, 1))



/************************//**
 * @brief Macro to get the startup torque for the servo
 *
 * @param id the ID number for target servo
 *
 * @return bye that holds alarm data. eacb bit of this value is mapped to a different error. If the error occurs and the bit is set to '1', then the LED will trigger when the error triggers 
 *
 * Compatible Servos: All AX/MX Servos
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error
 *
 * Bit 5 -  Overload Error
 *
 * Bit 4 -  CheckSum Error
 *
 * Bit 3 -  Range Error
 *
 * Bit 2 -  OverHeating Error
 *
 * Bit 1 -  Angle Limit Error
 *
 * Bit 0 -  Input Voltage Error
 * 
 *  EXAMPLE: get servo # 1's alarm LED data
 *
 *  int alarmLED = dxlGetAlarmLED(id) 
 *******************************************/  
#define dxlGetAlarmLED(id) (dxlGetRegister(id, AX_ALARM_LED, 1))

/************************//**
 * @brief Macro to get the startup torque for the servo
 *
 * @param id the ID number for target servo
 *
 * @return bye that holds alarm shutdown data. each bit of this value is mapped to a different error. If the error occurs and the bit is set to '1', then the servo will shutdown when the error triggers 
 *
 * Compatible Servos: All AX/MX Servos
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error
 *
 * Bit 5 -  Overload Error
 *
 * Bit 4 -  CheckSum Error
 *
 * Bit 3 -  Range Error
 *
 * Bit 2 -  OverHeating Error
 *
 * Bit 1 -  Angle Limit Error
 *
 * Bit 0 -  Input Voltage Error
 * 
 *  EXAMPLE: get servo # 1's alarm LED data
 *
 *  int alarmShutdown = dxlGetAlarmShutdown(id) 
 *******************************************/  
#define dxlGetAlarmShutdown(id) (dxlGetRegister(id, AX_ALARM_SHUTDOWN, 1))



/************************//**
 * @brief Macro to get the multi turn offset for the servo
 *
 * @param id the ID number for target servo
 *
 * @return offset for multi turn mode
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's startup torque value
 *
 *  int multiTurnOffset = mxGetMultiTurnOffset(id) 
 *******************************************/  
#define mxGetMultiTurnOffset(id) (dxlGetRegister(id, MX_MULTI_TURN_OFFSET_L, 2))


/************************//**
 * @brief Macro to get the resolution divider for the servo
 *
 * @param id the ID number for target servo
 *
 * Compatible Servos: All MX Servos
 *
 * @return Resolution devider for multi-turn mode
 *
 *  EXAMPLE: get servo # 1's startup torque value
 *
 *  int resolutionDivider = mxGetResolutionDivider(id) 
 *******************************************/  
#define mxGetResolutionDivider(id) (dxlGetRegister(id, MX_RESOLUTION_DIVIDER, 1))


//     /$$$$$$              /$$           /$$$$$$$   /$$$$$$  /$$      /$$       /$$$$$$$                                /$$   /$$                        
//    /$$__  $$            | $$          | $$__  $$ /$$__  $$| $$$    /$$$      | $$__  $$                              |__/  | $$                        
//   | $$  \__/  /$$$$$$  /$$$$$$        | $$  \ $$| $$  \ $$| $$$$  /$$$$      | $$  \ $$  /$$$$$$   /$$$$$$   /$$$$$$$ /$$ /$$$$$$    /$$$$$$   /$$$$$$ 
//   | $$ /$$$$ /$$__  $$|_  $$_/        | $$$$$$$/| $$$$$$$$| $$ $$/$$ $$      | $$$$$$$/ /$$__  $$ /$$__  $$ /$$_____/| $$|_  $$_/   /$$__  $$ /$$__  $$
//   | $$|_  $$| $$$$$$$$  | $$          | $$__  $$| $$__  $$| $$  $$$| $$      | $$__  $$| $$$$$$$$| $$  \ $$|  $$$$$$ | $$  | $$    | $$$$$$$$| $$  \__/
//   | $$  \ $$| $$_____/  | $$ /$$      | $$  \ $$| $$  | $$| $$\  $ | $$      | $$  \ $$| $$_____/| $$  | $$ \____  $$| $$  | $$ /$$| $$_____/| $$      
//   |  $$$$$$/|  $$$$$$$  |  $$$$/      | $$  | $$| $$  | $$| $$ \/  | $$      | $$  | $$|  $$$$$$$|  $$$$$$$ /$$$$$$$/| $$  |  $$$$/|  $$$$$$$| $$      
//    \______/  \_______/   \___/        |__/  |__/|__/  |__/|__/     |__/      |__/  |__/ \_______/ \____  $$|_______/ |__/   \___/   \_______/|__/      
//                                                                                                   /$$  \ $$                                            
//                                                                                                  |  $$$$$$/                                            
//                                                                                                   \______/                                             

/************************//**
 * @brief Macro to get whether or not the servo's torque is enabled
 *
 * @param id the ID number for target servo
 *
 * @return 0 = torque disabled, 1 = torque enabled
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's torque value
 *
 *  int torqueState = dxlGetTorqueEnable(id) 
 *******************************************/  
#define dxlGetTorqueEnable(id) (dxlGetRegister(id, AX_TORQUE_ENABLE, 1)) 

/************************//**
 * @brief Macro to get whether or not the servo's LED is on
 *
 * @param id the ID number for target servo
 *
 * @return 0 = led off, 1 = led on
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's led value
 *
 *  int ledState = dxlGetLed(id) 
 *******************************************/ 
#define dxlGetLed(id) (dxlGetRegister(id, AX_LED, 1))


/************************//**
 * @brief Macro to get the servo's counter clowckwise compliance margin
 *
 * @param id the ID number for target servo
 *
 * @return counter clockwise compliance margin
 *
 * Compatible Servos: All AX Servos
 *
 *  EXAMPLE: get servo # 1's counter clockwise compliance margin
 *
 *  int ccwComplianceMargin = axGetCCWComplianceMargin(id) 
 *******************************************/ 
#define axGetCCWComplianceMargin(id) (dxlGetRegister(id, AX_CCW_COMPLIANCE_MARGIN, 1))


/************************//**
 * @brief Macro to get the servo's  clowckwise compliance margin
 *
 * @param id the ID number for target servo
 *
 * @return  clockwise compliance margin
 *
 * Compatible Servos: All AX Servos
 *
 *  EXAMPLE: get servo # 1's  clockwise compliance margin
 *
 *  int cwComplianceMargin = axGetCWComplianceMargin(id) 
 *******************************************/ 
#define axGetCWComplianceMargin(id) (dxlGetRegister(id, AX_CW_COMPLIANCE_MARGIN, 1))

/************************//**
 * @brief Macro to get the servo's counter clowckwise compliance Slope
 *
 * @param id the ID number for target servo
 *
 * @return counter clockwise compliance Slope
 *
 * Compatible Servos: All AX Servos
 *
 *  EXAMPLE: get servo # 1's counter clockwise compliance Slope
 *
 *  int ccwComplianceSlope = axGetCCWComplianceSlope(id) 
 *******************************************/ 
#define axGetCCWComplianceSlope(id) (dxlGetRegister(id, AX_CCW_COMPLIANCE_SLOPE, 1))

/************************//**
 * @brief Macro to get the servo's clowckwise compliance Slope
 *
 * @param id the ID number for target servo
 *
 * @return  clockwise compliance Slope
 *
 * Compatible Servos: All AX Servos
 *
 *  EXAMPLE: get servo # 1's  clockwise compliance Slope
 *
 *  int cwComplianceSlope = axGetCWComplianceSlope(id) 
 *******************************************/ 
#define axGetCWComplianceSlope(id) (dxlGetRegister(id, AX_CW_COMPLIANCE_SLOPE, 1))


/************************//**
 * @brief Macro to get the servo's goal position
 *
 * @param id the ID number for target servo
 *
 * @return  goal position (NOTE: This is NOT the present position, see dxlGetPosition() )
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's  goal position
 *
 *  int goalPosition = dxlGetGoalPosition(id) 
 *******************************************/ 
#define dxlGetGoalPosition(id) (dxlGetRegister(id, AX_GOAL_POSITION_L, 2))

/************************//**
 * @brief Macro to get the servo's goal speed
 *
 * @param id the ID number for target servo
 *
 * @return  goal speed (NOTE: This is NOT the present speed, see dxlGetSpeed() )
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's  goal position
 *
 *  int goalSpeed = dxlGetGoalSpeed(id) 
 *******************************************/ 
#define dxlGetGoalSpeed(id) (dxlGetRegister(id, AX_GOAL_SPEED_L, 2))

/************************//**
 * @brief Macro to get the servo's running torque limit
 *
 * @param id the ID number for target servo
 *
 * @return  running torque limit
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's  torque limit
 *
 *  int torqueLimit = dxlGetTorqueLimit(id) 
 *******************************************/ 
#define dxlGetTorqueLimit(id) (dxlGetRegister(id, AX_TORQUE_LIMIT_L, 2))

/************************//**
 * @brief Macro to get the servo's present position
 *
 * @param id the ID number for target servo
 *
 * @return  present position
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present position
 *
 *  int position = dxlGetPosition(id) 
 *******************************************/ 
#define dxlGetPosition(id) (dxlGetRegister(id, AX_PRESENT_POSITION_L, 2))

/************************//**
 * @brief Macro to get the servo's present speed
 *
 * @param id the ID number for target servo
 *
 * @return  present speed
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present speed
 *
 *  int speed = dxlGetSpeed(id) 
 *******************************************/ 
#define dxlGetSpeed(id) (dxlGetRegister(id, AX_PRESENT_SPEED_L, 2))

/************************//**
 * @brief Macro to get the servo's present torque
 *
 * @param id the ID number for target servo
 *
 * @return  present torque
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present torque
 *
 *  int torque = dxlGetTorque(id) 
 *******************************************/ 
#define dxlGetTorque(id) (dxlGetRegister(id, AX_PRESENT_LOAD_L, 2))

/************************//**
 * @brief Macro to get the servo's present voltage
 *
 * @param id the ID number for target servo
 *
 * @return  present voltage * 10 (i.e. 111 = 11.1 volts)
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present voltage
 *
 *  int voltage = dxlGetVoltage(id) 
 *******************************************/ 
#define dxlGetVoltage(id) (dxlGetRegister(id, AX_PRESENT_VOLTAGE, 1))

/************************//**
 * @brief Macro to get the servo's present temperature
 *
 * @param id the ID number for target servo
 *
 * @return  present temperature in degrees celcius
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present temperature
 *
 *  int temperature = dxlGetTemperature(id) 
 *******************************************/ 
#define dxlGetTemperature(id) (dxlGetRegister(id, AX_PRESENT_TEMPERATURE, 1))


/************************//**
 * @brief Macro to get find out if the servo has a registered command
 *
 * @param id the ID number for target servo
 *
 * @return  0 = no registered command 1 = registered command
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present torque
 *
 *  int registered = dxlGetRegistered(id) 
 *******************************************/ 
#define dxlGetRegistered(id) (dxlGetRegister(id, AX_REGISTERED_INSTRUCTION, 1))

/************************//**
 * @brief Macro to get the servo's moving register
 *
 * @param id the ID number for target servo
 *
 * @return  0 = not moving, 1 = moving
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's moving register
 *
 *  int moving = dxlGetMoving(id) 
 *******************************************/ 
#define dxlGetMoving(id) (dxlGetRegister(id, AX_MOVING, 1))

/************************//**
 * @brief Macro to get the servo's EEPROM lock data
 *
 * @param id the ID number for target servo
 *
 * @return  0 = EEPROM not locked, 1 = EEPROMlocked
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's moving register
 *
 *  int lock = dxlGetLock(id) 
 *******************************************/ 
#define dxlGetLock(id) (dxlGetRegister(id, AX_LOCK, 1))

/************************//**
 * @brief Macro to get the servo's punch data
 *
 * @param id the ID number for target servo
 *
 * @return  punch data
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's punch data
 *
 *  int punch = dxlGetPunch(id) 
 *******************************************/ 
#define dxlGetPunch(id) (dxlGetRegister(id, AX_PUNCH_L, 2))


/************************//**
 * @brief Macro to get the servo's d(derivative) gain
 *
 * @param id the ID number for target servo
 *
 * @return  d(derivative) gain
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's D Gain
 *
 *  int d = mxGetD(id) 
 *******************************************/ 
#define mxGetD(id) (dxlGetRegister(id, MX_D, 1))

/************************//**
 * @brief Macro to get the servo's i(integral) gain
 *
 * @param id the ID number for target servo
 *
 * @return  i(integral) gain
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's I Gain
 *
 *  int i = mxGetI(id) 
 *******************************************/ 
#define mxGetI(id) (dxlGetRegister(id, MX_I, 1))

/************************//**
 * @brief Macro to get the servo's p(proportional) gain
 *
 * @param id the ID number for target servo
 *
 * @return  p(proportional) gain
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's P Gain
 *
 *  int p = mxGetP(id) 
 *******************************************/ 
#define mxGetP(id) (dxlGetRegister(id, MX_P, 1))


/************************//**
 * @brief Macro to get the servo's goal acceleration
 *
 * @param id the ID number for target servo
 *
 * @return  goal acceleration
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's goal acceleration
 *
 *  int acceleration = mxGetGoalAcceleration(id) 
 *******************************************/ 
#define mxGetGoalAcceleration(id) (dxlGetRegister(id, MX_GOAL_ACCELERATION, 1))

/************************//**
 * @brief Macro to get the servo's current
 *
 * @param id the ID number for target servo
 *
 * @return  current
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's current
 *
 *  int current = mxGetCurrent(id) 
 *******************************************/ 
#define mxGetCurrent(id) (dxlGetRegister(id, MX_CURRENT_L, 1))

/************************//**
 * @brief Macro to get the servo's goal torque
 *
 * @param id the ID number for target servo
 *
 * @return  goal torque
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's goal torque
 *
 *  int goalTorque = mxGetGoalTorque(id) 
 *******************************************/ 
#define mxGetGoalTorque(id) (dxlGetRegister(id, MX_GOAL_TORQUE_L, 2))

/************************//**
 * @brief Macro to get the servo's torque mode
 *
 * @param id the ID number for target servo
 *
 * @return  torque mode
 *
 * Compatible Servos: All MX Servos
 *
 *  EXAMPLE: get servo # 1's torque mode
 *
 *  int torqueMode = mxGetTorqueMode(id) 
 *******************************************/ 
#define mxGetTorqueMode(id) (dxlGetRegister(id, MX_TORQUE_CONTROL_MODE, 1))






/************************//**
 * @brief Macro turn torque off for a single servo
 *
 * @param id the ID number for target sensor
 *
 * @return none
 * @deprecated see dxlSetPosition()
 *
 * Compatible Servos:  All AX / MX servos
 ******************************************/
#define SetPosition(id, pos) (dxlSetRegister2(id, AX_GOAL_POSITION_L, pos))

/************************//**
 * @brief Macro to get servo position
 *
 * @param id the ID number for target sensor
 *
 * @return positional data 
 * @deprecated see dxlGetPosition()
 *
 * Compatible Servos:  All AX / MX servos
 ******************************************/
#define GetPosition(id) (dxlGetRegister(id, AX_PRESENT_POSITION_L, 2))


/************************//**
 * @brief Macro to  turn torque on for a single servo
 *
 * @param id the ID number for target sensor
 *
 * @return none
 * @deprecated see dxlToqueOn()
 *
 * Compatible Servos:  All AX / MX servos
 ******************************************/
#define TorqueOn(id) (dxlSetRegister(id, AX_TORQUE_ENABLE, 1))

/************************//**
 * @brief Macro to turn torque off for a single servo
 *
 * @param id the ID number for target sensor
 *
 * @return none
 * @deprecated see dxlToqueOff()
 *
 * Compatible Servos:  All AX / MX servos
 ******************************************/
#define Relax(id) (dxlSetRegister(id, AX_TORQUE_ENABLE, 0))




/************************//**
 * @brief Macro get data from S1 sensor module
 *
 * @param id the ID number for target sensor
 *
 * @return IR Data  
 * @deprecated S1 module is no longer manufacturered
 *
 * Compatible Servos: S1 Sensor Module
 ******************************************/
#define GetLeftIRData(id) (dxlGetRegister(id, AX_LEFT_IR_DATA))

/************************//**
 * @brief Macro get data from S1 sensor module
 *
 * @param id the ID number for target sensor
 *
 * @return IR Data  
 * @deprecated S1 module is no longer manufacturered
 *
 * Compatible Servos: S1 Sensor Module
 ******************************************/
#define GetCenterIRData(id) (dxlGetRegister(id, AX_CENTER_IR_DATA))

/************************//**
 * @brief Macro get data from S1 sensor module
 *
 * @param id the ID number for target sensor
 *
 * @return IR Data  
 * @deprecated S1 module is no longer manufacturered
 *
 * Compatible Servos: S1 Sensor Module
 ******************************************/
#define GetRightIRData(id) (dxlGetRegister(id, AX_RIGHT_IR_DATA))

/************************//**
 * @brief Macro get data from S1 sensor module
 *
 * @param id the ID number for target sensor
 *
 * @return obstacle  
 * @deprecated S1 module is no longer manufacturered
 *
 * Compatible Servos: S1 Sensor Module
 ******************************************/
#define GetObstacles(id) (dxlGetRegister(id, OBSTACLE_DETECTION))

/************************//**
 * @brief Macro get data from S1 sensor module
 *
 * @param id the ID number for target sensor
 * @param note note freqeuncy to play
 *
 * @return none  
 * @deprecated S1 module is no longer manufacturered
 *
 * Compatible Servos: S1 Sensor Module
 ******************************************/
#define PlayTone(id, note) (dxlSetRegister(id, AX_BUZZER_INDEX, note))



/************************//**
 * @brief Macro to initialize servo library
 *
 * @param baud speed to start the DYNAMIXEL chain at
 *
 * @return none
 *
 * @depracted: see dxlInit();
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: start library at 1mbps (1000000 bps)
 * 
 * ax12Init(1000000);
 ******************************************/
#define ax12Init(baud) (dxlInit( baud)); 

/************************//**
 * @brief Macro to read packet
 *
 * @param id the ID number for target servo
 * @param delayTime time (in milliseconds) that the servo should wait before returning data
 *
 * @return none  
 *
 * @depracted: see dxlReadPacket();
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1 return delay time on servo # 1 to 100ms
 *
 *  ax12ReadPacket(1,100);
 *******************************************/
#define ax12ReadPacket(length) (dxlReadPacket(length));



/************************//**
 * @brief Function to get data from specified register on single servo
 *
 * @param id the ID number for target servo
 * @param regstart starting/lower register to start reading from
 * @param length how many registers to read. Usually 1 or 2
 *
 * @return error byte
 *
 * @depracted: see dxlGetRegister();
 *
 * Compatible Servos: All AX/MX Servos
 *
 *  EXAMPLE: get servo # 1's present position (register 36/37)
 *
 *  int position = dxlGetRegister(id, 36, 2);
 *******************************************/  
#define ax12GetRegister(id, regstart, length) (dxlGetRegister(id, regstart, length));



/************************//**
 * @brief Macro to set a single register on the servo
 *
 * @param id the ID number for target servo
 * @param register the target register to set the data in
 * @param data  data for the register, values 0-255 (anything larger will be truncated)
 *
 * @return none  
 *
 * @depracted: see dxlSetRegister();
 *
 * Compatible Servos: All AX/MX Servos
 *
 * EXAMPLE: set servo # 1's register #30 (goal position) to 512
 *
 *  dxlSetRegister(1,25, 1);
 *******************************************/
#define ax12SetRegister(id, register, data) (dxlSetRegister(id, regstart, data));


/************************//**
 * @brief Macro to write a single value to 2 registers
 *
 * @param id the ID number for target servo
 * @param regstart starting/lower register. Data will be written accross regstart and regstart+1
 * @param data  data for the registers, values 0-65535
 *
 * @return none  
 *
 * @depracted: see dxlSetRegister2();
 *
 * Compatible Servos: All AX/MX Servos
 *
 * This function will do the necessary  conversions to conevert the 'data' value into 2 single bytes , and then write the bytes to the corresponding registers.
 * This function is usually used to write data for values that span 2 registers like goal position, goal speed, etc
 *
 * EXAMPLE: set servo # 1's register #30 (goal position) to 512
 *
 *  dxlSetRegister2(1,30, 512);
 *******************************************/
#define ax12SetRegister2(id, regstart, data) (dxlSetRegister2(id, regstart, data));


/************************//**
 * @brief Macro to get last error byte
 *
 * @param id the ID number for target servo
 *
 * @return error byte
 *
 * @depracted: see dxlGetLastError();
 *
 * Compatible Servos: All AX/MX Servos
 *
 * By defualt, a DYNAMIXEL servo will return data after a read or write operation. 
 * Included in this data is an 'error' byte. Each bit of the byte indicates a different 
 * error. If all the bits are '0', the whole byte is 'zero' and there is no error.
 * Otherwise an error is present.
 *
 * Bit 7 - NA
 *
 * Bit 6 - Instruction Error (ERR_INSTRUCTION)
 *
 * Bit 5 -  Overload Error (ERR_OVERLOAD)
 *
 * Bit 4 -  CheckSum Error (ERR_CHECKSUM)
 *
 * Bit 3 -  Range Error (ERR_RANGE)
 *
 * Bit 2 -  OverHeating Error (ERR_OVERHEATING)
 *
 * Bit 1 -  Angle Limit Error (ERR_ANGLE_LIMIT)
 *
 * Bit 0 -  Input Voltage Error (ERR_VOLTAGE)
 *
 *  EXAMPLE: get servo # 1's error byte
 *
 *  int position = dxlGetPosition(id);
 *  
 *  int error = ax12GetLastError() 
 *******************************************/  
#define ax12GetLastError() (dxlGetLastError());







/************************//**
 * @brief Macro to set transmit for sync write
 *
 * @param baud speed to start the DYNAMIXEL chain at
 *
 * @return none
 * @deprecated use dxlSetTXall();
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * dxlSetTXall();
 ******************************************/

#define setTXall() (dxlSetTXall());

 /************************//**
 * @brief macro to helper functions to switch direction of comms
 *
 * @param id target servo id 
 * @todo (is id this actually needed?)
 * @return none
 * @deprecated use dxlSetTX();
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * setTX();
 ******************************************/
#define setTX(id) (dxlSetTX(id));


 /************************//**
 * @brief macro to helper functions to switch direction of comms
 *
 * @param id target servo id 
 * @todo (is id this actually needed?)
 * @return none
 * @deprecated use dxlSetRX();
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * setRX();
 ******************************************/
#define setRX(id) (dxlSetRX(id));

 /************************//**
 * @brief macro to  Send a character out the serial port.
 *
 * @param data data to send
 * @return none
 * @deprecated use dxlWrite();
 *
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * ax12write();
 ******************************************/
#define ax12write(data) (dxlWrite(data));

 /************************//**
 * @brief macro to Send a character out the serial port, and puts it in the tx_buffer
 *
 * @param data data to send
 * @return none
 *
 * @deprecated use dxlWriteB();
 * 
 * Compatible Servos:  All AX / MX servos
 *
 * EXAMPLE: 
 * 
 * ax12writeB();
 ******************************************/
#define ax12writeB(data) (dxlWriteB(data));




//isax
//ismx
//realAngle
//speed direction                                                                                                                                                                                                                                                                           
//baud overload



#endif
