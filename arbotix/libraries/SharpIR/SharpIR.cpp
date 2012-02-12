/*
  SharpIR.cpp - Arduino Library for Sharp GP series of IR Sensors
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
*/

#include <Arduino.h>
#include "SharpIR.h"

/** = the distance to the nearest object on the requested channel in CM.*/
int gp2shortGetData(int pin){
	int sample;
	// Get data
	sample = analogRead(pin)/4;
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 15)
		return 254;	// max range
	// Magic numbers to get cm
	sample= 704/(sample+3);
	// This appears to work
	return sample - 1;
}

/** = the distance to the nearest object on the requested channel in CM.*/
int gp2d12GetData(int pin){
	int sample;
	// Get data
	sample = analogRead(pin)/4;
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 10)
		return 254;	// max range
	// Magic numbers to get cm
	sample= 1309/(sample-3);
	// This appears to work
	return sample - 1;
}

/** = the distance to the nearest object on the requested channel in inches.*/
int gp2longGetData(int pin){
	int sample;
	// Get data
	sample = analogRead(pin)/4;
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 20)
		return 320;	// max range
	// Magic numbers to get cm
	sample= 2870/(sample-3);
	// This appears to work
	return sample - 1;
}

/** = a smoothed out reading of a port - useful for IR sensors... */
int smooth(int (*func)(int ch), int channel){
    int reading = func(channel);
    reading = reading + func(channel);
    reading = reading + func(channel);
    reading = reading + func(channel);
    reading = reading/4;
    return reading; 
}

SharpIR::SharpIR(int type, int pin){
	pinMode(pin,INPUT);
	_pin = pin;
	_type = type;
}

int SharpIR::getData(){
	switch(_type){
		case GP2D12:
			return gp2d12GetData(_pin);
		case GP2D120:
			return gp2shortGetData(_pin);
		case GP2Y0A02YK:
			return gp2longGetData(_pin);
	}
	return 0;
}

int SharpIR::getSmoothData(){
	switch(_type){
		case GP2D12:
			return smooth(&gp2d12GetData,_pin);
		case GP2D120:
			return smooth(&gp2shortGetData,_pin);
		case GP2Y0A02YK:
			return smooth(&gp2longGetData,_pin);
	}
	return 0;
}

/* Revisions
 * 2-11-12 Update to Arduino 1.0
 * 5-5-09 Modified to use CM rather than inch 
 * 10-20-09 Added getSmoothData()
 */
