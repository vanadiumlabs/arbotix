*******************************************************************************
SharpIR Library for the Arduino/Sanguino
Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.

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

*******************************************************************************
Installation:
Copy files into <arduino>/hardware/libraries/SharpIR/ as you would any other 
Arduino library.

*******************************************************************************
Documentation:
    This library will take a reading from an analog port which a Sharp IR ranging 
sensor is connected to and convert it into centimeters. It supports three 
models of ranging sensors:
  GP2D120 - short range sensors
  GP2D12 - regular range sensors
  GP2Y0A02YK - long range sensors
    The constructor to create a SharpIR object is: SharpIR(type, analog_pin), 
where analog pin is 0-7 for the Sanguino, and type is one of the names listed 
above.
    To get a reading, call getData() on your SharpIR object. 

*******************************************************************************
Example code:

// This is a simple example of how to use the SharpIR library
#include <SharpIR.h>

// Create an instance of SharpIR, constructor is SharpIR(type, analog_pin)
SharpIR frontIR = SharpIR(GP2D120,0);

void setup(){		// this is called once
	Serial.begin(38400);
}

void loop(){        // this will print the sensor distance in CM, every 2sec.
	Serial.print(frontIR.getData());
	delay(2000);	
}

