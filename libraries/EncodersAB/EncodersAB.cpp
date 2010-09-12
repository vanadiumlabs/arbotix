/*
  EncodersAB.cpp - Hardware Encoder Library for Robocontrollers
  Copyright (c) 2009,2010 Michael E. Ferguson.  All right reserved.

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

#include <avr/interrupt.h>
#include <wiring.h>
#include "EncodersAB.h"

EncodersAB::EncodersAB() : left(0), right(0) {};

EncodersAB Encoders = EncodersAB(); 

#if defined(__AVR_ATmega168__) // mini/arduino
void leftCounter(){
    // d2 & d8
    if(PINB&0x01)
        Encoders.left++; // cw is d2 == d8
    else
        Encoders.left--;
}
void rightCounter(){
    // d3 & A0 (d14)
    if(PINC&0x01)
        Encoders.right++; // cw is d3 == A0
    else
        Encoders.right--;
}

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) // arbotix/sanguino
unsigned char lastx;
ISR(PCINT2_vect){
    unsigned char x = PINC;
    if((lastx&0x10) != (x&0x10)){
        // right = D20, D21 = PC4,5
        if(((x&0x20)>>1)==(x&0x10))    // cw if D20 == D21
            Encoders.right++;
        else
            Encoders.right--;
    }
    if((lastx&0x40) != (x&0x40)){     
        // left = D22, D23 = PC6,7
        if(((x&0x80)>>1)==(x&0x40))   // cw if D22 == D23
            Encoders.left++;
        else
            Encoders.left--;
    }
    lastx = x;
}

#elif defined(__AVR_ATmega1280__) // arbotix+/mega
ISR(INT6_vect){
//void leftCounter(){
    // PE6 (int 6) & PC2 (D35)
    if(PINC&0x04)
        Encoders.left++; // cw is PE6 == d35
    else
        Encoders.left--;
}
ISR(INT7_vect){
//void rightCounter(){
    // PE7 (int 7) & PC1 (D36)
    if(PINC&0x02)
        Encoders.right++; // cw is PE7 == d36
    else
        Encoders.right--;
}
#endif

void EncodersAB::Begin(){
#if defined(__AVR_ATmega168__) // mini/arduino
	attachInterrupt(0, leftCounter, RISING);
    attachInterrupt(1, rightCounter, RISING);
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) // arbotix/sanguino
	PCICR |= (1 << PCIE2);      // enable PC interrupt on port C
    PCMSK2 |= (1<<4) + (1<<6);    // enable interrupt on D20(C4),D22(C6)
    lastx = PINC;
#elif defined(__AVR_ATmega1280__) // arbotix+/mega
	//attachInterrupt(6, leftCounter, RISING);
    //attachInterrupt(7, rightCounter, RISING);
    EICRB |= 0xf0;  // rising edge both 6 and 7
    EIMSK |= 0xC0;  // enable 6 and 7
#endif
}

void EncodersAB::Reset(){
    left = 0;
    right = 0;
}


