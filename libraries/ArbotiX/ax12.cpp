/** @file 
    @brief DYNAMIXEL Library Main file
   */ 
/*
  ax12.cpp - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

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

#include "ax12.h"
#ifndef __ARDUINO_X86__
#include <avr/io.h>
#endif 

/******************************************************************************
 * Hardware Serial Level, this uses the same stuff as Serial1, therefore 
 *  you should not use the Arduino Serial1 library.
 */

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];

// making these volatile keeps the compiler from optimizing loops of available()
volatile int ax_rx_Pointer;
volatile int ax_tx_Pointer;
volatile int ax_rx_int_Pointer;
#if defined(AX_RX_SWITCHED)
unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif

/** helper functions to switch direction of comms */
void dxlSetTX(int id){
    bitClear(UCSR1B, RXEN1); 
  #if defined(AX_RX_SWITCHED)
    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_WR;
    else
        SET_AX_WR;   
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif   
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    ax_tx_Pointer = 0;
}
void dxlSetRX(int id){ 
  #if defined(AX_RX_SWITCHED)
    int i;
    // Need to wait for last byte to be sent before turning the bus around.
    // Check the Transmit complete flag
    while (bit_is_clear(UCSR1A, UDRE1));
    for(i=0; i<UBRR1L*15; i++)    
        asm("nop");
    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_RD;
    else
        SET_AX_RD;
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      int i;
      // Need to wait for last byte to be sent before turning the bus around.
      // Check the Transmit complete flag
      while (bit_is_clear(UCSR1A, UDRE1));
      for(i=0; i<25; i++)    
          asm("nop");
      PORTD &= 0xEF;
    #endif 
    bitClear(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXCIE1);
  #endif  
    bitSet(UCSR1B, RXEN1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
}
// for sync write
void dxlSetTXall(){
    bitClear(UCSR1B, RXEN1);    
  #if defined(AX_RX_SWITCHED)
    SET_RX_WR;
    SET_AX_WR;   
  #else
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    ax_tx_Pointer = 0;
}

/** Sends a character out the serial port. */
void dxlWrite(unsigned char data){
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** Sends a character out the serial port, and puts it in the tx_buffer */
void dxlWriteB(unsigned char data){
    ax_tx_buffer[(ax_tx_Pointer++)] = data; 
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
ISR(USART1_RX_vect){
    ax_rx_int_buffer[(ax_rx_int_Pointer++)] = UDR1;
}

/** read back the error code for our latest packet read */
int dxlError;
int dxlGetLastError(){ return dxlError; }
/** > 0 = success */
int dxlReadPacket(int length){
    unsigned long ulCounter;
    unsigned char offset, blength, checksum, timeout;
    unsigned char volatile bcount; 

    offset = 0;
    timeout = 0;
    bcount = 0;
    while(bcount < length){
        ulCounter = 0;
        while((bcount + offset) == ax_rx_int_Pointer){
            if(ulCounter++ > 1000L){ // was 3000
                timeout = 1;
                break;
            }
        }
        if(timeout) break;
        ax_rx_buffer[bcount] = ax_rx_int_buffer[bcount + offset];
        if((bcount == 0) && (ax_rx_buffer[0] != 0xff))
            offset++;
        else if((bcount == 2) && (ax_rx_buffer[2] == 0xff))
            offset++;
        else
            bcount++;
    }

    blength = bcount;
    checksum = 0;
    for(offset=2;offset<bcount;offset++)
        checksum += ax_rx_buffer[offset];
    if((checksum%256) != 255){
        return 0;
    }else{
        return 1;
    }
}

/** initializes serial1 transmit at baud, 8-N-1 */
void dxlInit(long baud){
    UBRR1H = (F_CPU / (8 * baud) - 1 ) >> 8;
    UBRR1L = (F_CPU / (8 * baud) - 1 );
    bitSet(UCSR1A, U2X1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
    ax_tx_Pointer = 0;
#if defined(AX_RX_SWITCHED)
    INIT_AX_RX;
    bitSet(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXEN1);
    bitSet(UCSR1B, RXCIE1);
#else
  #ifdef ARBOTIX_WITH_RX
    DDRD |= 0x10;   // Servo B = output
    PORTD &= 0xEF;  // Servo B low
  #endif
    // set RX as pull up to hold bus to a known level
    PORTD |= (1<<2);
    // enable rx
    setRX(0);
#endif
}

/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int dxlGetRegister(int id, int regstart, int length){  
    dxlSetTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    dxlWriteB(0xFF);
    dxlWriteB(0xFF);
    dxlWriteB(id);
    dxlWriteB(4);    // length
    dxlWriteB(AX_READ_DATA);
    dxlWriteB(regstart);
    dxlWriteB(length);
    dxlWriteB(checksum);  
    dxlSetRX(id);    
    if(dxlReadPacket(length + 6) > 0){
        dxlError = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

/* Set the value of a single-byte register. */
void dxlSetRegister(int id, int regstart, int data){
    dxlSetTX(id);    
    int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);
    dxlWriteB(0xFF);
    dxlWriteB(0xFF);
    dxlWriteB(id);
    dxlWriteB(4);    // length
    dxlWriteB(AX_WRITE_DATA);
    dxlWriteB(regstart);
    dxlWriteB(data&0xff);
    // checksum = 
    dxlWriteB(checksum);
    dxlSetRX(id);
    //dxlReadPacket();
}
/* Set the value of a double-byte register. */
void dxlSetRegister2(int id, int regstart, int data){
    dxlSetTX(id);    
    int length = 5;                 //parameter low byte + parameter high byte + register # + id + inst
    int checksum = ~((id + length + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    dxlWriteB(0xFF);
    dxlWriteB(0xFF);
    dxlWriteB(id);
    dxlWriteB(length);    // length
    dxlWriteB(AX_WRITE_DATA);
    dxlWriteB(regstart);
    dxlWriteB(data&0xff);
    dxlWriteB((data&0xff00)>>8);
    // checksum = 
    dxlWriteB(checksum);
    dxlSetRX(id);
    //dxlReadPacket();
}


void dxlRegWrite(int id, int regstart, int data)
{
    dxlSetTX(id);                  //
    int length = 4;             //parameter low byte + register # + id + inst
    int checksum = ~((id + length + AX_REG_WRITE + regstart + (data&0xff)) % 256);
    dxlWriteB(0xFF);           //header 1  
    dxlWriteB(0xFF);           //header 2
    dxlWriteB(id);             //id for command 
    dxlWriteB(length);         //packet length
    dxlWriteB(AX_REG_WRITE);   //instruction
    dxlWriteB(regstart);       //register to write byte at
    dxlWriteB(data&0xff);      //paramater data to set
    dxlWriteB(checksum);       //checksum byte of all non-header bytes
    dxlSetRX(id);                  //get ready to read return packet

}
void dxlRegWrite2(int id, int regstart, int data)
{

    dxlSetTX(id);    
    int length = 5;                 //parameter low byte + parameter high byte + register # + id + inst
    int checksum = ~((id + length + AX_REG_WRITE + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    dxlWriteB(0xFF);               //header 1
    dxlWriteB(0xFF);               //header 2
    dxlWriteB(id);                 //id for command
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_REG_WRITE);       //instruction
    dxlWriteB(regstart);           //register to write byte at
    dxlWriteB(data&0xff);          //paramater to set (Low byte)       
    dxlWriteB((data&0xff00)>>8);   //parameter data (high byte)
    dxlWriteB(checksum);           //checksum byte of all non-header bytes
    dxlSetRX(id);                      //get ready to read return packet

}

void dxlAction()
{
    dxlAction(DXL_BROADCAST);   //when no paramater is given, assume broadcast address
}


void dxlAction(int id)
{
    dxlSetTX(id);    
    int length = 2;         //  id + inst
    int checksum = ~((id + length + AX_ACTION)  % 256);  //checksum is the inverse of all non header bytes
    dxlWriteB(0xFF);               //header 1
    dxlWriteB(0xFF);               //header 2
    dxlWriteB(id);                 //id for command
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_ACTION);          //instruction
    dxlWriteB(checksum);           //checksum byte 
    dxlSetRX(0);     //don't need this, no return packet right?                 //get ready to read return packet
}



int dxlPing(int id)
{
    dxlSetTX(id);    
    int length = 2;                 //  id + inst
    int checksum = ~((id + length + AX_PING)  % 256);    //checksum is the inverse of all non header bytes
    dxlWriteB(0xFF);               //header 1
    dxlWriteB(0xFF);               //header 2
    dxlWriteB(id);                 //id for command
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_PING);            //instruction
    dxlWriteB(checksum);           //checksum byte of all non-header bytes  
    dxlSetRX(id);             //get ready to read return packet


    if(dxlReadPacket(6) > 0)
    {
        if(ax_rx_buffer[2] == id);  //check that the return packet matches the ID
    }
    else
    {
        return -1;
    }

}


int dxlGetError(int id)
{
    dxlSetTX(id);    
    int length = 2;                 //  id + inst
    int checksum = ~((id + length + AX_PING)  % 256);    //checksum is the inverse of all non header bytes
    dxlWriteB(0xFF);               //header 1
    dxlWriteB(0xFF);               //header 2
    dxlWriteB(id);                 //id for command
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_PING);            //instruction
    dxlWriteB(checksum);           //checksum byte of all non-header bytes  
    dxlSetRX(id);                  //get ready to read return packet


    if(dxlReadPacket(6) > 0)
    {
        return ax_rx_buffer[4]; //return error byte
    }
    else
    {
        return -1; //no servo found
    }

}



void dxlReset(int id)
{
    dxlSetTX(id);    
    int length = 2;                 //  id + inst
    int checksum = ~((id + length + AX_RESET)  % 256);    //checksum is the inverse of all non header bytes
    dxlWriteB(0xFF);               //header 1
    dxlWriteB(0xFF);               //header 2
    dxlWriteB(id);                 //id for command
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_RESET);            //instruction
    dxlWriteB(checksum);           //checksum byte of all non-header bytes  
    dxlSetRX(id);             //get ready to read return packet
}

void dxlSyncWrite(int servoData[][2], int numberOfServos, int registerStart, int registerLength)
{

    int length = 4 + (numberOfServos * (registerLength + 1)); //base length is 4, then we need to add length for each servo we want to write to

    unsigned int checksum = (DXL_BROADCAST + length + AX_SYNC_WRITE + registerLength + registerStart) ;    //start building the checksum with known data, ID, length and instruction


    dxlSetTX(DXL_BROADCAST);       //set to tranmsit mode 
    dxlWriteB(0xFF);               //header byte 1, always 0xFF / 255
    dxlWriteB(0xFF);               //header byte 2, always 0xFF / 255
    dxlWriteB(DXL_BROADCAST);      //id for bulk read is broadcast / 254
    dxlWriteB(length);             //packet length
    dxlWriteB(AX_SYNC_WRITE);      //instruction
    dxlWriteB(registerStart);      //target register
    dxlWriteB(registerLength);     //register length (usuauly 1 or 2)
    
    //start building the rest of the packet from the readRequestData array
    for(int i = 0; i < numberOfServos; i++)
    {
      dxlWriteB(servoData[i][0]);      //servo ID
      dxlWriteB(servoData[i][1]&0xff); //first byte
      checksum = checksum + servoData[i][0] + (servoData[i][1]&0xff) ; //update checksum
      //send a second byte if the length is 2
      if(registerLength == 2)
      {
          dxlWriteB((servoData[i][1]>>8)&0xff); //second byte. shift source doen 8 bits and isolate the byte
          checksum = checksum + ((servoData[i][1]>>8)&0xff) ; //update checksum
      }
      
    }

    checksum = ~(checksum)  % 256;    //compute checksum (invert and isolate lowest byte) 

    dxlWriteB(checksum);           //checksum byte of all non-header bytes  
    dxlSetRX(DXL_BROADCAST);             //get ready to read return packet

}


void mxBulkRead(int readRequestData[][3], int numberOfRequests, int returnData[])
{   

    dxlSetTX(DXL_BROADCAST);                //set to tranmsit mode 
    int length = 3 * numberOfRequests + 3;  //length of packet is 3*number of servos (length, id, register) + id + instruction + 0 byte
    unsigned int checksum = (DXL_BROADCAST + length + MX_BULK_READ) ;    //start building the checksum with known data, ID, length and instruction

    dxlWriteB(0xFF);               //header byte 1, always 0xFF / 255
    dxlWriteB(0xFF);               //header byte 2, always 0xFF / 255
    dxlWriteB(DXL_BROADCAST);      //id for bulk read is braodcase / 254
    dxlWriteB(length);             //packet length
    dxlWriteB(MX_BULK_READ);       //instruction
    dxlWriteB(0x00);               //this byte is always a fixed 0x00
    
    //start building the rest of the packet from the readRequestData array
    for(int i = 0; i < numberOfRequests; i++)
    {
      dxlWriteB(readRequestData[i][0]); //packet length
      dxlWriteB(readRequestData[i][1]); //servo ID
      dxlWriteB(readRequestData[i][2]); //register number
      checksum = checksum + readRequestData[i][0] + readRequestData[i][1] + readRequestData[i][2]; //update checksum
    }

    checksum = ~(checksum)  % 256;    //compute checksum (invert and isolate lowest byte) 

    dxlWriteB(checksum);           //checksum byte of all non-header bytes  
    dxlSetRX(DXL_BROADCAST);             //get ready to read return packet

    //there should be the same number of return packets as there are servo requests we made, so iterate through them 
    for(int i = 0; i < numberOfRequests; i++)
    {    
        int packLen = readRequestData[i][0] + 6; //return packet length sepends on the length of the data we requested
        //read dxl packet
        if(dxlReadPacket(packLen) > 0)
        {
            dxlError = ax_rx_buffer[4]; //store error byte from return packet
            if(length == 1)
            {
                returnData[i] = ax_rx_buffer[5]; //if the length is 1, than we only need to return the one byte
            }
            else if (length == 2)
            {
                returnData[i] = ax_rx_buffer[5] + (ax_rx_buffer[6]<<8); //if the length is 2, than we need to combine 2 bytes 
            }
            //TODO: past 3-byte data packet? rare, but possible
        }
        else
        {
            returnData[i] = -1; //there was an error
        }
    }
}





//writes with no read?
// general write?
// general sync write?



// void centerServos()
// {

// }




float dxlGetSystemVoltage(int numberOfServos)
{

    int  servoList[numberOfServos];


    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }
    return(dxlGetSystemVoltage(numberOfServos, servoList));
}


float dxlGetSystemVoltage(int numberOfServos, const int servoList[])
{

    int voltageSum = 0;  //temporary sum of all voltage readings for average
   int servosFound = 0;    //assume that we find all of the servos
   int tempVoltage = 0;                //temporary holder for the voltage
   float finalVoltage;

   for (int i = 0; i < numberOfServos; i++)
   {


      tempVoltage = dxlGetRegister (servoList[i], AX_PRESENT_VOLTAGE, 1);
      //if the voltage is greater or equal to zero, add it to the sum
      if(tempVoltage > -1)
      {  
         voltageSum = tempVoltage + voltageSum;
         servosFound = servosFound + 1;
         //Serial.println(voltageSum);
      }
      //anything below zero indicates a missing servo
      else
      {
        
      }
      delay(33);
   }


   if(servosFound < 1)
   {
     return(-1);

   }

   finalVoltage = ((float)voltageSum / ((float)servosFound) / (float)10); //divide the voltage sum by the number of servos to get the raw average voltage. Divide by 10 to get votlage in volts
   


   return(finalVoltage);
  
}

float dxlGetSystemVoltage(int numberOfServos, int servoList[])
{
   int voltageSum = 0;  //temporary sum of all voltage readings for average
   int servosFound = 0;    //assume that we find all of the servos
   int tempVoltage = 0;                //temporary holder for the voltage
   float finalVoltage;

   for (int i = 0; i < numberOfServos; i++)
   {


      tempVoltage = dxlGetRegister (servoList[i], AX_PRESENT_VOLTAGE, 1);
      //if the voltage is greater or equal to zero, add it to the sum
      if(tempVoltage > -1)
      {  
         voltageSum = tempVoltage + voltageSum;
         servosFound = servosFound + 1;
         //Serial.println(voltageSum);
      }
      //anything below zero indicates a missing servo
      else
      {
        
      }
      delay(33);
   }


   if(servosFound < 1)
   {
     return(-1);

   }

   finalVoltage = ((float)voltageSum / ((float)servosFound) / (float)10); //divide the voltage sum by the number of servos to get the raw average voltage. Divide by 10 to get votlage in volts
   


   return(finalVoltage);


}


void dxlVoltageReport(int numberOfServos)
{

    int  servoList[numberOfServos];

    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }
    dxlVoltageReport(numberOfServos, servoList);

    
}

void dxlVoltageReport(int numberOfServos, int servoList[])
{
   float voltage = dxlGetSystemVoltage(numberOfServos, servoList); //Check Power Supply Voltage
  
   Serial.print("Average System Voltage:");  
   Serial.println(voltage);   
   
   if( voltage < 10)
   {
     
     if (voltage < 0)
     {
       Serial.println("No Servos detected - voltage cannot be obtained");
     }
     else
     {
       Serial.println("System Voltage is under 10.0v, check your battery or power supply");
     }
   
     Serial.println("We reccomend you fix your problem before proceededing. Send any character to continue anyway");
     
     while(Serial.available() < 1)
     {
       //do nothing while no characters are detected
     }
     while(Serial.available() > 0)
     {
       Serial.read();//clear serial input
     }
     
   }
   
}

void dxlVoltageReport(int numberOfServos, const int servoList[])
{
   float voltage = dxlGetSystemVoltage(numberOfServos, servoList); //Check Power Supply Voltage
  
   Serial.print("Average System Voltage:");  
   Serial.println(voltage);   
   
   if( voltage < 10)
   {
     
     if (voltage < 0)
     {
       Serial.println("No Servos detected - voltage cannot be obtained");
     }
     else
     {
       Serial.println("System Voltage is under 10.0v, check your battery or power supply");
     }
   
     Serial.println("We reccomend you fix your problem before proceededing. Send any character to continue anyway");
     
     while(Serial.available() < 1)
     {
       //do nothing while no characters are detected
     }
     while(Serial.available() > 0)
     {
       Serial.read();//clear serial input
     }
     
   }
   
}


void dxlServoReport(int numberOfServos)
{

    int  servoList[numberOfServos];

    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }
    dxlServoReport(numberOfServos, servoList);

    
}


void dxlServoReport(int numberOfServos, int servoList[])
{
    int pos;            //holds the positional value of the servo
    int missingServos = 0;    //number of servos that could not be contacted

    Serial.println("######################################################");
    Serial.println("Starting Servo Scanning Test.");
    Serial.println("######################################################");

    for(int i = 0; i < numberOfServos ; i++)
    {
        pos =  dxlGetRegister(servoList[i], 36, 2);

        int errorBit = dxlGetLastError();


        //if there is no data, retry once
        if (pos <= 0)
        {
           Serial.println("###########################");
           Serial.print("Cannot connect to servo #");
           Serial.print(servoList[i]);
           Serial.println(", retrying");
           delay(500);  //short delay to clear the bus
           pos =  dxlGetRegister(servoList[i], 36, 2);
           errorBit = dxlGetLastError();
        }

        //if there is still no data, print an error.
        if (pos <= 0)
        {
            Serial.print("ERROR! Servo ID: ");
            Serial.print(servoList[i]);
            Serial.println(" not found. Please check connection and verify correct ID is set.");
            Serial.println("###########################"); 
            missingServos = missingServos + 1;
        }

        else
        {

            Serial.print("Servo ID: ");
            Serial.println(servoList[i]);
            Serial.print("Servo Positions: ");
            Serial.println(pos);
            // Serial.print("err Bit: ");
            // Serial.println(errorBit);

            if(ERR_NONE == errorBit)
            {
                Serial.println("          No Errors Found");
            }
            if(ERR_VOLTAGE & errorBit)
            {
                Serial.println("          Voltage Error");
            }
            if(ERR_ANGLE_LIMIT & errorBit)
            {
                Serial.println("          Angle Limit Error");
            }
            if(ERR_OVERHEATING & errorBit)
            {
                Serial.println("          Overheating Error");
            }
            if(ERR_RANGE & errorBit)
            {
                Serial.println("          Range Error");
            }
            if(ERR_CHECKSUM & errorBit)
            {
                Serial.println("          Checksum Error");
            }
            if(ERR_OVERLOAD & errorBit)
            {
                Serial.println("          Overload Error");
            }
            if(ERR_INSTRUCTION & errorBit)
            {
                Serial.println("          Instruction Error");
            }
      

            delay(100);
            
        }

    }
      

    if (missingServos > 0)
    {
        Serial.println("###########################");
        Serial.print("ERROR! ");
        Serial.print(missingServos);
        Serial.println(" servos ID(s) are missing from Scan. Please check connection and verify correct ID is set.");

        Serial.println("###########################");  
    }
    else
    {
        Serial.println("All servo IDs present.");
    }   
    Serial.println("######################################################");  

}


void dxlServoReport(int numberOfServos, const int servoList[])
{
    int pos;            //holds the positional value of the servo
    int missingServos = 0;    //number of servos that could not be contacted

    Serial.println("######################################################");
    Serial.println("Starting Servo Scanning Test.");
    Serial.println("######################################################");

    for(int i = 0; i < numberOfServos ; i++)
    {
        pos =  dxlGetRegister(servoList[i], 36, 2);

        int errorBit = dxlGetLastError();


        //if there is no data, retry once
        if (pos <= 0)
        {
           Serial.println("###########################");
           Serial.print("Cannot connect to servo #");
           Serial.print(servoList[i]);
           Serial.println(", retrying");
           delay(500);  //short delay to clear the bus
           pos =  dxlGetRegister(servoList[i], 36, 2);
           errorBit = dxlGetLastError();
        }

        //if there is still no data, print an error.
        if (pos <= 0)
        {
            Serial.print("ERROR! Servo ID: ");
            Serial.print(servoList[i]);
            Serial.println(" not found. Please check connection and verify correct ID is set.");
            Serial.println("###########################"); 
            missingServos = missingServos + 1;
        }

        else
        {

            Serial.print("Servo ID: ");
            Serial.println(servoList[i]);
            Serial.print("Servo Positions: ");
            Serial.println(pos);
            // Serial.print("err Bit: ");
            // Serial.println(errorBit);

            if(ERR_NONE == errorBit)
            {
                Serial.println("          No Errors Found");
            }
            if(ERR_VOLTAGE & errorBit)
            {
                Serial.println("          Voltage Error");
            }
            if(ERR_ANGLE_LIMIT & errorBit)
            {
                Serial.println("          Angle Limit Error");
            }
            if(ERR_OVERHEATING & errorBit)
            {
                Serial.println("          Overheating Error");
            }
            if(ERR_RANGE & errorBit)
            {
                Serial.println("          Range Error");
            }
            if(ERR_CHECKSUM & errorBit)
            {
                Serial.println("          Checksum Error");
            }
            if(ERR_OVERLOAD & errorBit)
            {
                Serial.println("          Overload Error");
            }
            if(ERR_INSTRUCTION & errorBit)
            {
                Serial.println("          Instruction Error");
            }
      

            delay(100);
            
        }

    }
      

    if (missingServos > 0)
    {
        Serial.println("###########################");
        Serial.print("ERROR! ");
        Serial.print(missingServos);
        Serial.println(" servos ID(s) are missing from Scan. Please check connection and verify correct ID is set.");

        Serial.println("###########################");  
    }
    else
    {
        Serial.println("All servo IDs present.");
    }   
    Serial.println("######################################################");  

}



void dxlServoReport()
{
    int pos;            //holds the positional value of the servo
   
    Serial.println("######################################################");
    Serial.println("Starting Full Servo Scan.");
    Serial.println("######################################################");

    for(int i = 0; i < 255 ; i++)
    {
        pos =  dxlGetRegister(i, 36, 2);

        int errorBit = dxlGetLastError();


        //if there is no data, retry once
        if (pos <= 0)
        {
           delay(10);  //short delay to clear the bus
           pos =  dxlGetRegister(i, 36, 2);
           int errorBit = dxlGetLastError();
        }

        if (pos > -1)
        {


            Serial.print("Servo ID: ");
            Serial.println(i);
            Serial.print("Servo Position: ");
            Serial.println(pos);

            if(ERR_NONE & errorBit)
            {
                Serial.println("          No Errors Found");
            }
            if(ERR_VOLTAGE & errorBit)
            {
                Serial.println("          Voltage Error");
            }
            if(ERR_ANGLE_LIMIT & errorBit)
            {
                Serial.println("          Angle Limit Error");
            }
            if(ERR_OVERHEATING & errorBit)
            {
                Serial.println("          Overheating Error");
            }
            if(ERR_RANGE & errorBit)
            {
                Serial.println("          Range Error");
            }
            if(ERR_CHECKSUM & errorBit)
            {
                Serial.println("          Checksum Error");
            }
            if(ERR_OVERLOAD & errorBit)
            {
                Serial.println("          Overload Error");
            }
            if(ERR_INSTRUCTION & errorBit)
            {
                Serial.println("          Instruction Error");
            }
            delay(10);   
        }

    }
      

    Serial.println("Scan Finished");  
    Serial.println("######################################################");  

}







int dxlScanServos(int numberOfServos, int returnList[])
{

    int  servoList[numberOfServos];

    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }
    return(dxlScanServos(numberOfServos, servoList, returnList));

    
}


int dxlScanServos(int numberOfServos, int servoList[], int returnList[])
{

    int pos;            //holds the positional value of the servo. This is arbitrary as we'll be discarding this value - we're just using it to check if the servo is present

    int missingServos = 0;    //number of servos that could not be contacted
    int foundServos = 0;    //number of servos that could not be contacted

    for(int i = 0; i < numberOfServos ; i++)
    {
        pos =  dxlGetRegister(servoList[i], 36, 2);
        int errorBit = dxlGetLastError();


        //if there is no data, retry once
        if (pos <= 0)
        {
           delay(10);  //short delay to clear the bus
           pos =  dxlGetRegister(servoList[i], 36, 2);
           int errorBit = dxlGetLastError();
        }

        //if there is still no data add one to the missing servos.
        if (pos <= 0)
        {
            missingServos = missingServos + 1;
            returnList[i] = -1;
        }
        else
        {
            foundServos = foundServos + 1;
            returnList[i] = errorBit;
        }
    }
    
    return(foundServos);
}


int dxlScanServos(int numberOfServos, const int servoList[], int returnList[])
{

    int pos;            //holds the positional value of the servo. This is arbitrary as we'll be discarding this value - we're just using it to check if the servo is present

    int missingServos = 0;    //number of servos that could not be contacted
    int foundServos = 0;    //number of servos that could not be contacted

    for(int i = 0; i < numberOfServos ; i++)
    {
        pos =  dxlGetRegister(servoList[i], 36, 2);
        int errorBit = dxlGetLastError();


        //if there is no data, retry once
        if (pos <= 0)
        {
           delay(10);  //short delay to clear the bus
           pos =  dxlGetRegister(servoList[i], 36, 2);
           int errorBit = dxlGetLastError();
        }

        //if there is still no data add one to the missing servos.
        if (pos <= 0)
        {
            missingServos = missingServos + 1;
            returnList[i] = -1;
        }
        else
        {
            foundServos = foundServos + 1;
            returnList[i] = errorBit;
        }
    }
    
    return(foundServos);
}


void dxlRegisterReportRaw(int servoID)
{
    Serial.print("Register Table for Servo #");
    Serial.println(servoID);
    int regData;

    for(int i = 0; i< MAX_REGISTERS; i++)
    {
        regData = dxlGetRegister(servoID, i, 1); 
        Serial.print(i);
        Serial.print(",");
        delay(33);
        Serial.println(regData);
        delay(33);
    }
}







// void axRegisterDump(int id)
// {
//   Serial.println("******************************************");
//   Serial.print("AX REGISTER DUMP FOR SERVO ID");
//   Serial.println(id);

//   Serial.print("Model Number:   ");
//   Serial.println(dxlGetModel(id));
//   Serial.print("Firmware Version:   ");
//   Serial.println(dxlGetFirmwareVersion(id));
//   Serial.print("ID:   ");
//   Serial.println(dxlGetId(id));
//   Serial.print("Baud Rate:   ");
//   Serial.println(dxlGetBaud(id));
//   Serial.print("Return Delay Time:   ");
//   Serial.println(dxlGetReturnDelayTime(id));
//   Serial.print("CW Angle Limit:   ");
//   Serial.println(dxlGetCWAngleLimit(id));
//   Serial.print("CCW Angle Limit:   ");
//   Serial.println(dxlGetCCWAngleLimit(id));
//   Serial.print("Temp Limit High:   ");
//   Serial.println(dxlGetTempLimit(id));
//   Serial.print("Low Voltage Limit:  ");
//   Serial.println(dxlGetLowVoltage(id));
//   Serial.print("High Voltage Limit:   ");
//   Serial.println(dxlGetHighVoltage(id));
//   Serial.print("Startup Max Torque:   ");
//   Serial.println(dxlGetStartupMaxTorque(id));
//   Serial.print("Status Return Level:   ");
//   Serial.println(dxlGetStatusReturnLevel(id));
//   Serial.print("Alarm LED:   ");
//   Serial.println(dxlGetAlarmLED(id));
//   Serial.print("Alarm Shutdown:   ");
//   Serial.println(dxlGetAlarmShutdown(id));
 




// }

// void mxRegisterDump(int id)
// {

// }

// void rawRegisterDump(int id)
// {

// }

void dxlLedTest(int numberOfServos, int ledTime)
{
 

    int  servoList[numberOfServos];

    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }

    dxlLedTest(numberOfServos, servoList, ledTime);

}

void dxlLedTest(int numberOfServos, int servoList[], int ledTime)
{


    dxlSetLED(DXL_BROADCAST, 0);

    for(int i = 0; i< numberOfServos; i++) 
    {
        dxlSetLED(servoList[i], 1);
        delay(ledTime);
        dxlSetLED(servoList[i], 0);
        //delay(ledTime); //do we want time that the servo is off? I don't think so   

    }

    dxlSetLED(DXL_BROADCAST, 0);

}
void dxlLedTest(int numberOfServos, const int servoList[], int ledTime)
{


    dxlSetLED(DXL_BROADCAST, 0);

    for(int i = 0; i< numberOfServos; i++) 
    {
        dxlSetLED(servoList[i], 1);
        delay(ledTime);
        dxlSetLED(servoList[i], 0);
        //delay(ledTime); //do we want time that the servo is off? I don't think so   

    }

    dxlSetLED(DXL_BROADCAST, 0);

}
   
int dxlIsServoMX(int servoID)
{
  int modelNumber =  dxlGetModel(servoID);
  return(dxlIsModelMX(modelNumber));
}


int dxlIsServoAX(int servoID)
{
  int modelNumber =  dxlGetModel(servoID);
  return(dxlIsModelAX(modelNumber));
}



int dxlIsModelMX(int modelNumber)
{
  if((modelNumber == MX_12W_MODEL_NUMBER) || (modelNumber == MX_28_MODEL_NUMBER) || (modelNumber == MX_64_MODEL_NUMBER) || (modelNumber == MX_106_MODEL_NUMBER) )
  {
    return(1);
  }
  else
  {
    return(0);
  }
}

int dxlIsModelAX(int modelNumber)
{
  if((modelNumber == AX_12_MODEL_NUMBER) || (modelNumber == AX_18_MODEL_NUMBER) || (modelNumber == AX_12W_MODEL_NUMBER) )
  {
    return(1);
  }
  else
  {
    return(0);
  }
}



void mxSetMultiTurnMode(int servoId)
{
  dxlSetJointMode(servoId, MX_MAX_POSITION_VALUE, MX_MAX_POSITION_VALUE);
}


void mxSetJointMode(int servoId)
{
  dxlSetJointMode(servoId, 0, MX_MAX_POSITION_VALUE);
  delay(33);
  mxSetTorqueControlMode(servoId,0);
}


void axSetJointMode(int servoId)
{
  dxlSetJointMode(servoId, 0, AX_MAX_POSITION_VALUE);
}



void dxlSetJointMode(int servoId, int CWAngleLimit, int CCWAngleLimit)
{
  dxlSetCWAngleLimit(servoId, CWAngleLimit);
  delay(33);
  dxlSetCCWAngleLimit(servoId, CCWAngleLimit);

}

void axSetWheelMode(int servoId)
{
  dxlSetCWAngleLimit(servoId, 0);
  delay(33);
  dxlSetCCWAngleLimit(servoId, 0);

}


void mxSetWheelMode(int servoId)
{
  dxlSetCWAngleLimit(servoId, 0);
  delay(33);
  dxlSetCCWAngleLimit(servoId, 0);
  delay(33);
  mxSetTorqueControlMode(servoId,0);

}



int dxlGetMode(int servoId)
{
    int cwAngleLimit = dxlGetCWAngleLimit(servoId);
    delay(33);
    int ccwAngleLimit = dxlGetCCWAngleLimit(servoId);


    if(cwAngleLimit == -1 || ccwAngleLimit == -1)
    {
        return(-1);
    }


    int isMx = dxlIsServoMX(servoId);
    delay(33);

    if(isMx == 1)
    {
        if(mxGetTorqueMode(servoId) == 1)
        {
            return(4);   //mx torque mode  
        }        
        else if(cwAngleLimit == MX_MAX_POSITION_VALUE && ccwAngleLimit == MX_MAX_POSITION_VALUE)
        {
            return(3);//mx multi turn mode 
        }

    }

    //angles are both 0, 
    if(cwAngleLimit == 0 && ccwAngleLimit == 0)
    {
        return(2);//wheel mode
    }

    else 
    {
        return(1); //joint mode
    }


}



int axToMxPosition(int axPosition)
{

    float axAngle = axPosition * 300.0 * (1.0/1024.0)  ;
    float mxAngle = 30 + axAngle;

    int mxPosition = mxAngle / (360.0 * (1.0/4096));

    return(mxPosition);
}

int mxToAxPosition(int mxPosition)
{

    float mxAngle = mxPosition * 360.0 * (1.0/4096.0)  ;
    float axAngle = mxAngle - 30;
    if(axAngle < 0)
    {
        return(0);  //to low for Ax servo. 
    }
    else if(axAngle > 300)
    {
        return(1023);  //to high for ax servo. 
    }


    int axPosition = axAngle / (300.0 * (1.0/1024));

    return(axPosition);
}


void dxlRegisterReportSingle(int servoID)
{

    int  servoList[1] = {servoID};

    dxlRegisterReport(1, servoList);
}



void dxlRegisterReport(int numberOfServos)
{

 

    int  servoList[numberOfServos];

    for(int i = 0; i < numberOfServos; i++)
    {
        servoList[i] = i + 1; //starting with servo number one in array slot 0, so everything is offset by 1
    }

    dxlRegisterReport( numberOfServos, servoList);
}



void dxlRegisterReport(int numberOfServos, int servoList[])
{
 
  int regData;
  int modelData[numberOfServos];


  delay(33);

  Serial.println("Register Table for All Servos #");

  Serial.print("Register,\t\t");

    for(int i = 0; i< numberOfServos; i++)
    {
        Serial.print("ID #");
        Serial.print(servoList[i]);
        Serial.print(",\t");
        delay(33);
    }



  

    // Serial.print("SERVO #,\t");
    // for(int i = 0; i< numberOfServos; i++)
    // {
    //     Serial.print(servoList[i]);
    //     Serial.print(",");
    //     delay(33);
    // }
    Serial.println("");

    Serial.print("MODEL,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetModel(servoList[i]); 

        modelData[i] = regData; //store register data forr later printout


        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("FIRMWARE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetFirmwareVersion(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ID,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetId(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("BAUD RATE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetBaud(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("RETURN DELAY TIME,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetReturnDelayTime(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CW ANGLE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CCW ANGLE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TEMPERATURE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTempLimit(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOW VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLowVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("HIGH VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetHighVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MAX TORQUE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStartupMaxTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("STATUS RETURN LEVEL,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStatusReturnLevel(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM LED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmLED(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM SHUTDOWN,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmShutdown(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("Multi Turn Offset,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetMultiTurnOffset(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Resolution Divider,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetResolutionDivider(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");









    Serial.print("TORQUE ENABLE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueEnable(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LED,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");


    Serial.print("CW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("CCW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");





    Serial.print("CW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");

    Serial.print("CCW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("D,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetD(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("I,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetI(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("P,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetP(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");







    Serial.print("GOAL POSITION,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TORQUE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT POSITION,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT LOAD,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorque(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT VOLTAGE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetVoltage(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT TEMPERATURE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTemperature(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("REGISTERED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetRegistered(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetMoving(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOCK,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLock(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PUNCH,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPunch(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");




    Serial.print("Current(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetCurrent(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Torque Mode(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetTorqueMode(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Goal Torque(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");



    Serial.print("Goal acceleration,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalAcceleration(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");





}


void dxlRegisterReport(int numberOfServos, const int servoList[])
{
 
  int regData;
  int modelData[numberOfServos];


  delay(33);

  Serial.println("Register Table for All Servos #");

  Serial.print("Register,\t\t");

    for(int i = 0; i< numberOfServos; i++)
    {
        Serial.print("ID #");
        Serial.print(servoList[i]);
        Serial.print(",\t");
        delay(33);
    }



  

    // Serial.print("SERVO #,\t");
    // for(int i = 0; i< numberOfServos; i++)
    // {
    //     Serial.print(servoList[i]);
    //     Serial.print(",");
    //     delay(33);
    // }
    Serial.println("");

    Serial.print("MODEL,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetModel(servoList[i]); 

        modelData[i] = regData; //store register data forr later printout


        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("FIRMWARE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetFirmwareVersion(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ID,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetId(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("BAUD RATE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetBaud(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("RETURN DELAY TIME,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetReturnDelayTime(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CW ANGLE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CCW ANGLE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TEMPERATURE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTempLimit(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOW VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLowVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("HIGH VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetHighVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MAX TORQUE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStartupMaxTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("STATUS RETURN LEVEL,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStatusReturnLevel(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM LED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmLED(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM SHUTDOWN,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmShutdown(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("Multi Turn Offset,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetMultiTurnOffset(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Resolution Divider,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetResolutionDivider(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");









    Serial.print("TORQUE ENABLE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueEnable(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LED,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");


    Serial.print("CW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("CCW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");





    Serial.print("CW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");

    Serial.print("CCW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("D,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetD(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("I,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetI(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("P,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetP(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");







    Serial.print("GOAL POSITION,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TORQUE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT POSITION,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT LOAD,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorque(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT VOLTAGE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetVoltage(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT TEMPERATURE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTemperature(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("REGISTERED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetRegistered(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetMoving(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOCK,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLock(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PUNCH,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPunch(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");




    Serial.print("Current(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetCurrent(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Torque Mode(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetTorqueMode(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Goal Torque(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");



    Serial.print("Goal acceleration,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalAcceleration(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");





}

