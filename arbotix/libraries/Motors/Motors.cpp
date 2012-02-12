/*
  Motors.cpp - Hardware Motor Library for Arduino, using Timer0
  Created by Michael Ferguson, using modified components of the 
  Seattle Robotics Society Workshop robot Level1 Samples:
    http://seattlerobotics.org/WorkshopRobot/

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
#include "Motors.h"

/* we compare to OCR0A/B for R/L motor speeds */
#define lPWM        OCR0B
#define rPWM        OCR0A

/* Constructor: sets up microprocessor for PWM control of motors */
void Motors::init(){
    /* set up ports */
    pinMode(RIGHT_DIR,OUTPUT);    // Right Dir
    pinMode(LEFT_PWM,OUTPUT);    // Left PWM
    pinMode(RIGHT_PWM,OUTPUT);    // Right PWM
    pinMode(LEFT_DIR,OUTPUT);    // Left Dir

    /* OCR0A/B are the values that the timer is compared to; a match will
       cause the output to change; small values mean the motor runs for a
       short period (slower); larger values are longer times (faster)*/ 
    digitalWrite(LEFT_DIR,LOW);
    digitalWrite(RIGHT_DIR,LOW);
    analogWrite(LEFT_PWM,0);
    analogWrite(RIGHT_PWM,0);
    lPWM = rPWM = 0;    // (value is irrelevant since outputs are disconnected)
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void Motors::left(int pwm){
    if (pwm == 0){
        digitalWrite(LEFT_DIR, LOW);
        TCCR0A &= ~((1<<COM0B1) | (1<<COM0B0));
    }else{
        if (pwm >= 0){
            digitalWrite(LEFT_DIR,LOW);            
            TCCR0A |= (1<<COM0B1);
            TCCR0A &= ~(1<<COM0B0); 
        }else{
            digitalWrite(LEFT_DIR,HIGH);
            TCCR0A |= ((1<<COM0B1) | (1<<COM0B0)); 
              pwm = -pwm;
        }
        if (pwm > 255)
            pwm = 255;
        lPWM = pwm;        // set width for PWM
    }
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void Motors::right(int pwm){
    if (pwm == 0){
        digitalWrite(RIGHT_DIR,LOW);
        TCCR0A &= ~((1<<COM0A1) | (1<<COM0A0));
    }else{
        if (pwm >= 0){
            digitalWrite(RIGHT_DIR,LOW);
            TCCR0A |= (1<<COM0A1);
            TCCR0A &= ~(1<<COM0A0);    
        }else{
            digitalWrite(RIGHT_DIR,HIGH);
            TCCR0A |= ((1<<COM0A1) | (1<<COM0A0));
            pwm = -pwm;
        }
        if (pwm > 255)
            pwm = 255;
        rPWM = pwm;        // set width for PWM
    }
}

void Motors::set(int lpwm, int rpwm){
    this->left(lpwm);
    this->right(rpwm);
}
