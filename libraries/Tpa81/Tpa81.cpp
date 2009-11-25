/*
  Tpa81.cpp - Arduino Library for TPA-81 ThermoPile Sensor
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

#include "wiring.h"
#include "Wire.h"
#include "Tpa81.h"

/** Constructor */
Tpa81::Tpa81(unsigned char device){
    _device = device;
}

/** = a reading of ambient temp, and loads pixel values into buffer */
int Tpa81::getData(unsigned char * pixels){
    int ambient;
    // ask for data data
    Wire.beginTransmission(TPA_ADDR+_device);
    Wire.send(TPA_AMBIENT);
    Wire.endTransmission();
    // read back data
    Wire.requestFrom(TPA_ADDR+_device, (int) 9);
    while(Wire.available() < 9){
        // need some sort of timeout!
    }
    // fill our data    
    ambient = Wire.receive();
    pixels[0] = Wire.receive();
    pixels[1] = Wire.receive();
    pixels[2] = Wire.receive();
    pixels[3] = Wire.receive();
    pixels[4] = Wire.receive();
    pixels[5] = Wire.receive();
    pixels[6] = Wire.receive();
    pixels[7] = Wire.receive();
    return ambient;
}
