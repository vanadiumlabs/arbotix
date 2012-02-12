/*
  Srf08.cpp - Arduino Library for Devantech SRF-08 Sonar
  Copyright (c) 2009 Michael E. Ferguson.  All right reserved.

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

#include "Arduino.h"
#include "Wire.h"
#include "Srf08.h"

/** Constructor */
Srf08::Srf08(unsigned char device){
    _device = device;
    _lastping = millis();
}

/** = version of sonar sonar */
int Srf08::getVersion(){
    Wire.beginTransmission(SRF_ADDR+_device);
    Wire.write(SRF_COMMAND);
    Wire.endTransmission();
    // read back data
    Wire.requestFrom(SRF_ADDR+_device, (int) 1);
    while(Wire.available() < 1){
        // need some sort of timeout!
    }
    // fill our data    
    return (int) Wire.read();
}

/** We must ping the sonar before we can read it back. */
void Srf08::ping(){
    // write command to read in CM
    Wire.beginTransmission(SRF_ADDR+_device);
    Wire.write(SRF_COMMAND);
    Wire.write(SRF_CMD_CM);
    Wire.endTransmission();
}

/** = a reading from a sonar, should call ping first. */
int Srf08::getData(){
    int distance
    Wire.beginTransmission(SRF_ADDR+_device);
    Wire.write(SRF_ECHO_H);
    Wire.endTransmission();
    // read back data
    Wire.requestFrom(SRF_ADDR+_device, (int) 1);
    while(Wire.available() < 2){
        // need some sort of timeout!
    }
    distance = Wire.read() << 8;
    distance += Wire.read();
    return distance;
}

/** Change address of SRF 08
int srf08ChangeAddr(unsigned char oldAddr, unsigned char newAddr){
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA0);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xAA);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA5);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    // actually send address
    i2cWrite(SRF_ADDR + (2 * newAddr));
    i2cStop(); 
} NOTE THIS CODE WORKS, BUT IS NOT STANDARD TO INCLUDE. */

