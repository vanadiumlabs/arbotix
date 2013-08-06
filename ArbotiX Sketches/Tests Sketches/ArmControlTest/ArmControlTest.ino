/*
  Commander.pde - default firmware for arbotiX Commander (V1.1)
  Copyright (c) 2009, 2010 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later versio  n.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Arduino IDE Configuration:
  
*/

#define RIGHT_V   0
#define RIGHT_H   1
#define LEFT_V    2
#define LEFT_H    3
#define LEFT_Y    4
#define SLIDER    5

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

#define USER      0


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
    int XH = 2;
    int XL = 0;
    int YH = 0;
    int YL = 150;
    int ZH = 0;
    int ZL = 150;
    int WAH = 0;
    int WAL = 90;
    int WRH = 2;
    int WRL = 0;
    int GH = 2;
    int GL = 0;
    int ext = 0;
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
    Serial.write((unsigned char) XH);
    Serial.write((unsigned char) XL);
    Serial.write((unsigned char) YH);
    Serial.write((unsigned char) YL);
    Serial.write((unsigned char) ZH);
    Serial.write((unsigned char) ZL);
    Serial.write((unsigned char) WAH);
    Serial.write((unsigned char) WAL);
    Serial.write((unsigned char) WRH);
    Serial.write((unsigned char) WRL);
    Serial.write((unsigned char) GH);
    Serial.write((unsigned char) GL);
    
    Serial.write(buttons);  // buttons
    Serial.write(ext);        // extra
    Serial.write((unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+buttons)%256));
    
    if(i > 10){
      digitalWrite(0,HIGH-digitalRead(0));
      i=0;
    }
    i++;
    delay(FRAME_LEN);
}

/* Revisions 
 *  V1.1 - Feb 19, 2010 - Replaced Walk/Look with Right/Left 
 *         (since apparently, I used something called "southpaw")
 *  V1.0 - Feb 10, 2010 - Firmware Finalized
 */

