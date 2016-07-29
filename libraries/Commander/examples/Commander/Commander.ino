/*
  Commander.ino - default firmware for arbotiX Commander (V1.1)
  Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.

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

#define RIGHT_V   0
#define RIGHT_H   1
#define LEFT_V    2
#define LEFT_H    3

#define BUT_L6    13
#define BUT_L5    12
#define BUT_L4    11
#define BUT_LT    9

#define BUT_R3    7
#define BUT_R2    6
#define BUT_R1    5
#define BUT_RT    8

#define BUT_RIGHT 3
#define BUT_LEFT  2

#define USER      10

#define FRAME_LEN 33         // 30hz
unsigned long ltime;         // last time we sent data

int i;
void setup(){
    Serial.begin(38400);
    ltime = millis();
    pinMode(USER,OUTPUT);    // user LED
    
    // pullups for buttons
    digitalWrite(BUT_L6, HIGH);
    digitalWrite(BUT_L5, HIGH);
    digitalWrite(BUT_L4, HIGH);
    digitalWrite(BUT_LT, HIGH);
    digitalWrite(BUT_R3, HIGH);
    digitalWrite(BUT_R2, HIGH);
    digitalWrite(BUT_R1, HIGH);
    digitalWrite(BUT_RT, HIGH);
    
    digitalWrite(BUT_RIGHT, HIGH);
    digitalWrite(BUT_LEFT, HIGH);
    i = 0;
}

void loop(){
    int right_V = (1023-analogRead(RIGHT_V)-512)/5 + 128;
    int right_H = (analogRead(RIGHT_H)-512)/5 + 128;
    int left_V = (1023-analogRead(LEFT_V)-512)/5 + 128;
    int left_H = (analogRead(LEFT_H)-512)/5 + 128;
 
    //buttons =    
    unsigned char buttons = 0;
    if(digitalRead(BUT_R1) == LOW) buttons += 1;
    if(digitalRead(BUT_R2) == LOW) buttons += 2;
    if(digitalRead(BUT_R3) == LOW) buttons += 4;
    if(digitalRead(BUT_L4) == LOW) buttons += 8;
    if(digitalRead(BUT_L5) == LOW) buttons += 16;
    if(digitalRead(BUT_L6) == LOW) buttons += 32;
    if(digitalRead(BUT_RT) == LOW) buttons += 64;
    if(digitalRead(BUT_LT) == LOW) buttons += 128;
 
    Serial.write(0xff);
    Serial.write((unsigned char) right_V);
    Serial.write((unsigned char) right_H);
    Serial.write((unsigned char) left_V);
    Serial.write((unsigned char) left_H);
    Serial.write(buttons);              // buttons
    Serial.write((unsigned char)0);     // extra
    Serial.write((unsigned char)(255 - (right_V+right_H+left_V+left_H+buttons)%256));
    
    if(i > 10){
      digitalWrite(10,HIGH-digitalRead(10));
      i=0;
    }
    i++;
    delay(FRAME_LEN);
}

/* Revisions 
 *  V1.2 - Feb 11, 2012 - Updated for Arduino 1.0
 *  V1.1 - Feb 19, 2010 - Replaced Walk/Look with Right/Left 
 *         (since apparently, I used something called "southpaw")
 *  V1.0 - Feb 10, 2010 - Firmware Finalized
 */

