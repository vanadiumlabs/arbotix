/*
  Motors2.cpp - Hardware Motor Library for ArbotiX, using Timer2
  Copyright (c) 2009-2012 by Michael Ferguson. All rights reserved.

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
#include "Motors2.h"

/* we compare to OCR0A/B for R/L motor speeds */
#define M2_lPWM        OCR2B
#define M2_rPWM        OCR2A

/* Constructor: sets up microprocessor for PWM control of motors */
void Motors2::init(){
    /* change clock back to no divisor */
    TCCR2B &= ~(1<<CS22);
    TCCR2B &= ~(1<<CS22);
    TCCR2B |= (1<<CS20);

    /* set up ports */
  #ifdef M_EN
    pinMode(M1_A,OUTPUT);           // Left Dir
    pinMode(M1_B,OUTPUT);	        
    pinMode(M1_PWM,OUTPUT);         // Left PWM
    pinMode(M2_A,OUTPUT);           // Right Dir
    pinMode(M2_B,OUTPUT);
    pinMode(M2_PWM,OUTPUT);         // Right PWM
    pinMode(M_EN, OUTPUT);          // Motors Enable
    digitalWrite(M_EN, HIGH);       // Motors Enable

    /* OCR2A/B are the values that the timer is compared to; a match will
       cause the output to change; small values mean the motor runs for a
       short period (slower); larger values are longer times (faster)*/
    analogWrite(M1_PWM,0);
    analogWrite(M2_PWM,0);
  #else
    pinMode(M2_RIGHT_DIR,OUTPUT);   // Right Dir
    pinMode(M2_LEFT_PWM,OUTPUT);    // Left PWM
    pinMode(M2_RIGHT_PWM,OUTPUT);   // Right PWM
    pinMode(M2_LEFT_DIR,OUTPUT);    // Left Dir

    /* OCR0A/B are the values that the timer is compared to; a match will
       cause the output to change; small values mean the motor runs for a
       short period (slower); larger values are longer times (faster)*/ 
    digitalWrite(M2_LEFT_DIR,LOW);
    digitalWrite(M2_RIGHT_DIR,LOW);
    analogWrite(M2_LEFT_PWM,0);
    analogWrite(M2_RIGHT_PWM,0);
  #endif
    M2_lPWM = M2_rPWM = 0;    // (value is irrelevant since outputs are disconnected)
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void Motors2::left(int pwm){
    l_pwm = pwm;
  #ifdef M_EN
    if(pwm >= 0){
        digitalWrite(M1_A,LOW);
        digitalWrite(M1_B,HIGH);
        analogWrite(M1_PWM, pwm);
    }else{
        digitalWrite(M1_B,LOW);
        digitalWrite(M1_A,HIGH);
        analogWrite(M1_PWM, -pwm);
    }  
  #else
    if (pwm == 0){
        digitalWrite(M2_LEFT_DIR, LOW);
        TCCR2A &= ~((1<<COM2B1) | (1<<COM2B0));
    }else{
        if (pwm >= 0){
            digitalWrite(M2_LEFT_DIR,LOW);            
            TCCR2A |= (1<<COM2B1);
            TCCR2A &= ~(1<<COM2B0); 
        }else{
            digitalWrite(M2_LEFT_DIR,HIGH);
            TCCR2A |= ((1<<COM2B1) | (1<<COM2B0)); 
            pwm = -pwm;
        }
        if (pwm > 255)
            pwm = 255;
        M2_lPWM = pwm;        // set width for PWM
    }
  #endif
}

/* lock the wheel in place */
void Motors2::brakeLeft(int pwm){
  #ifdef M_EN
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,LOW);
    if(pwm > 255)
        pwm = 255;
    analogWrite(M1_PWM,pwm);
  #endif
    l_pwm = 0;
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void Motors2::right(int pwm){
    r_pwm = pwm;
  #ifdef M_EN
    if(pwm >= 0){
        digitalWrite(M2_A,LOW);
        digitalWrite(M2_B,HIGH);
        analogWrite(M2_PWM, pwm);
    }else{
        digitalWrite(M2_B,LOW);
        digitalWrite(M2_A,HIGH);
        analogWrite(M2_PWM, -pwm);
    }
  #else
  	if (pwm == 0){
		digitalWrite(M2_RIGHT_DIR,LOW);
		TCCR2A &= ~((1<<COM2A1) | (1<<COM2A0));
	}else{
    	if (pwm >= 0){   
    		digitalWrite(M2_RIGHT_DIR,HIGH);
			TCCR2A |= ((1<<COM2A1) | (1<<COM2A0)); 
    	}else{
			digitalWrite(M2_RIGHT_DIR,LOW);
			TCCR2A |= (1<<COM2A1);
			TCCR2A &= ~(1<<COM2A0);
			pwm = -pwm;
    	}
    	if (pwm > 255)
      		pwm = 255;
      	M2_rPWM = pwm;		// set width for PWM
  	}
  #endif
}

/* lock the wheel in place */
void Motors2::brakeRight(int pwm){
    #ifdef M_EN
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,LOW);
    if(pwm > 255)
        pwm = 255;
    analogWrite(M2_PWM,pwm);
    #endif
    r_pwm = 0;
}

void Motors2::set(int lpwm, int rpwm){
    this->left(lpwm);
    this->right(rpwm);
}

int Motors2::getLeft(){
    return this->l_pwm;
}

int Motors2::getRight(){
    return this->r_pwm;
}

