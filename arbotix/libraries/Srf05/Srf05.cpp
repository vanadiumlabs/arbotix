/*
  Srf05.cpp - Arduino Library for Devantech SRF-05 Sonar
  Copyright (c) 2008, 2009 Michael E. Ferguson.  All right reserved.

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
#include "Srf05.h"

Srf05::Srf05(int pin){
	_pin = pin;
	pinMode(_pin,OUTPUT);
	digitalWrite(_pin, LOW);
}

int Srf05::getData(){
	int sample;
	/* Send a 15us ping */
	pinMode(_pin,OUTPUT);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(_pin,LOW);
	/* Read a pulse */
	pinMode(_pin,INPUT);
	sample = pulseIn(_pin,HIGH,5000);
    /* Convert the pulse to distance (us/148 = inch) (us/58=cm) */
	sample = sample/58;
	return sample;
}

/* Revisions
 * 5/5/09 - Changed delay(15) to delayMicroseconds(10) as per notes on 
 *          http://robot-electronics.co.uk/htm/arduino_examples.htm
 */
