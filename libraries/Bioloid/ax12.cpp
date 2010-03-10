/*
  ax12.cpp - arbotiX Library for AX-12 Servos
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

#include "wiring.h" // we need this for the serial port defines

#include "ax12.h"
#include <avr/interrupt.h>

// Uncomment the following line if you wish to use the RX-Bridge
//#define RX_PROTOCOL 1

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

/** helper functions to emulate half-duplex */
void setTX(){
#ifdef RX_PROTOCOL
    PORTD |= 0x10;
#endif
    bitClear(UCSR1B, RXEN1);    
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
    ax_tx_Pointer = 0;
}
void setRX(){
#ifdef RX_PROTOCOL
    // Need to wait for last byte to be sent before turning the bus around.
    // Check the Transmit complete flag
    while((UCSR1A&TXC1) == 0);
    asm("nop");
    asm("nop");
    asm("nop");
    PORTD &= 0xEF;
#endif 
    bitClear(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXEN1);
    bitSet(UCSR1B, RXCIE1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
}

/** Sends a character out the serial port. */
void ax12write(unsigned char data){
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
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
int ax12Error;
/** > 0 = success */
int ax12ReadPacket(int length){
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
void ax12Init(long baud){
    UBRR1H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
    UBRR1L = ((F_CPU / 16 + baud / 2) / baud - 1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
    ax_tx_Pointer = 0;
#ifdef RX_PROTOCOL
    DDRD |= 0x10;   // Servo B = output
    PORTD &= 0xEF;  // Servo B low
#endif
    // enable rx
    setRX();    
}

/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length){  
    setTX();
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(regstart);
    ax12writeB(length);
    ax12writeB(checksum);  
    setRX();    
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

/* Set the value of a single-byte register. */
void ax12SetRegister(int id, int regstart, int data){
    setTX();    
    int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    // checksum = 
    ax12writeB(checksum);
    setRX();
    //ax12ReadPacket();
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX();    
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(5);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    ax12writeB((data&0xff00)>>8);
    // checksum = 
    ax12writeB(checksum);
    setRX();
    //ax12ReadPacket();
}

// general write?
// general sync write?

