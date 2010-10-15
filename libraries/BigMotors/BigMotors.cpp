/*
  BigMotors.h - Big Motor Library for ArbotiX+ using Timer1
  Copyright (c) 2010 Michael E. Ferguson.  All right reserved.

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

#include <wiring.h>
#include "BigMotors.h"

/* we compare to OCR2A/B for R/L motor speeds */
#define M1_TIMER		OCR2B
#define M2_TIMER		OCR2A

/* Constructor: sets up microprocessor for PWM control of motors */
BigMotors::BigMotors(){
  	/* set up ports */
  	pinMode(M1_A,OUTPUT);           // Left Dir
  	pinMode(M1_B,OUTPUT);	        
    pinMode(M1_PWM,OUTPUT);         // Left PWM
    pinMode(M2_A,OUTPUT);           // Right Dir
    pinMode(M2_B,OUTPUT);
    pinMode(M2_PWM,OUTPUT);         // Right PWM
    pinMode(M_EN, OUTPUT);          // Motors Enable
    digitalWrite(M_EN, HIGH);          // Motors Enable

    /* OCR2A/B are the values that the timer is compared to; a match will
       cause the output to change; small values mean the motor runs for a
       short period (slower); larger values are longer times (faster)*/
    analogWrite(M1_PWM,0);
  	analogWrite(M2_PWM,0);
    M1_TIMER = M2_TIMER = 0;	
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void BigMotors::left(int pwm){
    m1_pwm = pwm;
    if(pwm >= 0){
        digitalWrite(M1_A,HIGH);
        digitalWrite(M1_B,LOW);
        analogWrite(M1_PWM, pwm);
    }else{
        digitalWrite(M1_B,HIGH);
        digitalWrite(M1_A,LOW);
        analogWrite(M1_PWM, -pwm);
    }  
}

/** lock the wheel in place */
void BigMotors::brakeLeft(int pwm){
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,LOW);
    if(pwm > 255)
        pwm = 255;
    analogWrite(M1_PWM,pwm);
    m1_pwm = 0;
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void BigMotors::right(int pwm){
    m2_pwm = pwm;
    if(pwm >= 0){
        digitalWrite(M2_A,LOW);
        digitalWrite(M2_B,HIGH);
        analogWrite(M2_PWM, pwm);
    }else{
        digitalWrite(M2_B,LOW);
        digitalWrite(M2_A,HIGH);
        analogWrite(M2_PWM, -pwm);
    }  
}

/* lock the wheel in place */
void BigMotors::brakeRight(int pwm){
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,LOW);
    if(pwm > 255)
        pwm = 255;
    analogWrite(M2_PWM,pwm);
    m2_pwm = 0;
}

void BigMotors::set(int lpwm, int rpwm){
	this->left(lpwm);
	this->right(rpwm);
}

int BigMotors::getLeft(){
    return this->m1_pwm;
}

int BigMotors::getRight(){
    return this->m2_pwm;
}

