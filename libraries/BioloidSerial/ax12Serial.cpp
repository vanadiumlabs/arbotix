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

#include "ax12Serial.h"
#ifndef __ARDUINO_X86__
#include <avr/io.h>
#endif 
unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];

// Lets have the init setup  
static Stream* s_paxStream;

/** initializes serial1 transmit at baud, 8-N-1 */
void ax12Init(long baud, Stream* pstream ){
    // Need to enable the PU resistor on the TX pin
    s_paxStream = pstream; 


    // Lets do some init here
    if (pstream == &Serial) {
        Serial.begin(baud);
    }    
#ifdef Serial1
    if (pstream == &Serial1) {
        Serial1.begin(baud);
#if defined(__MK20DX256__)
        UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
        CORE_PIN1_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
#endif
    }    
#endif
#ifdef Serial2
    if (pstream == &Serial2) {
        Serial2.begin(baud);
        UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
        CORE_PIN10_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
#if defined(__MK20DX256__)
#endif
    }    
#endif
#ifdef Serial3
    if (pstream == &Serial3) {
        Serial3.begin(baud);
#if defined(__MK20DX256__)
        UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
        CORE_PIN8_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
#endif
    }    
#endif
    // DEBUG
#ifdef DEBUG_PINS
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
#endif    
    setRX(0);    
}

/** helper functions to switch direction of comms */
void setTX(int id){
    setTXall();
}

void setTXall(){
#if defined(__MK20DX256__)
#define UART_C3_TXDIR			(uint8_t)0x20			// Transmitter Interrupt or DMA Transfer Enable.
    // Teensy 3.1
#if AX12Serial == Serial1
	uint8_t c;
	c = UART0_C3;
	c |= UART_C3_TXDIR;
	UART0_C3 = c;
#elif AX12Serial == Serial2
	uint8_t c;
	c = UART1_C3;
	c |= UART_C3_TXDIR;
	UART1_C3 = c;
#elif AX12Serial == Serial3
	uint8_t c;
	c = UART2_C3;
	c |= UART_C3_TXDIR;
	UART2_C3 = c;
#endif
#elif defined(__ARDUINO_X86__)
    // Currently assume using USB2AX or the like
    
#else    
#if AX12Serial == Serial1
    UCSR1B = /*(1 << UDRIE1) |*/ (1 << TXEN1);
#elif AX12Serial == Serial2
    UCSR2B = /*(1 << UDRIE3) |*/ (1 << TXEN2);
#elif AX12Serial == Serial3
    UCSR3B =  /*(1 << UDRIE3) |*/ (1 << TXEN3);
#endif
#endif
}

void flushAX12InputBuffer(void)  {
    // First lets clear out any RX bytes that may be lingering in our queue
    while (s_paxStream->available()) {
        s_paxStream->read();   
    }
}

void setRX(int id){ 
  
#ifdef DEBUG_PINS
    digitalWrite(4, HIGH);
#endif
    // First clear our input buffer
	flushAX12InputBuffer();
#ifdef DEBUG_PINS
    digitalWrite(4, LOW);
    // Now wait for any pending outputs to fully transmit
    digitalWrite(5, HIGH);
#endif
    s_paxStream->flush();
#ifdef DEBUG_PINS
    digitalWrite(5, LOW);
#endif    
    // Now setup to enable the RX and disable the TX
#if defined(__MK20DX256__)
#define UART_C3_TXDIR			(uint8_t)0x20			// Transmitter Interrupt or DMA Transfer Enable.
    // Teensy 3.1
#if AX12Serial == Serial1
	uint8_t c;
	c = UART0_C3;
	c &= ~UART_C3_TXDIR;
	UART0_C3 = c;
#elif AX12Serial == Serial2
	uint8_t c;
	c = UART1_C3;
	c &= ~UART_C3_TXDIR;
	UART1_C3 = c;
#elif AX12Serial == Serial3
	uint8_t c;
	c = UART2_C3;
	c &= ~UART_C3_TXDIR;
	UART2_C3 = c;
#endif

#elif defined(__ARDUINO_X86__)
    // Currently assume using USB2AX or the like
    
#else    
#if AX12Serial == Serial1 
    UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
#elif AX12Serial == Serial2
    UCSR2B = ((1 << RXCIE2) | (1 << RXEN2);
#elif AX12Serial == Serial3
    UCSR3B = ((1 << RXCIE3) | (1 << RXEN3));
#endif
#endif
}


/** Sends a character out the serial port. */
void ax12write(unsigned char data){
    s_paxStream->write(data);
}

/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
    s_paxStream->write(data);
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */

/** read back the error code for our latest packet read */
int ax12Error;
int ax12GetLastError(){ return ax12Error; }
/** > 0 = success */

#if defined(__MK20DX256__)
#define COUNTER_TIMEOUT 12000
#else
#define COUNTER_TIMEOUT 3000
#endif

int ax12ReadPacket(int length){
    unsigned long ulCounter;
    unsigned char offset, checksum;
    unsigned char volatile bcount; 
	unsigned char *psz; 
	unsigned char *pszEnd;
    int ch;
    

#ifdef DEBUG_PINS
    digitalWrite(2, HIGH);
#endif
    offset = 0;
    bcount = 0;
	
	psz = ax_rx_buffer;
	pszEnd = &ax_rx_buffer[length];
#ifdef DEBUG
	pinMode(A4, OUTPUT);
#endif
	
    flushAX12InputBuffer();
	
	// Need to wait for a character or a timeout...
	do {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = s_paxStream->read()) == -1) {
			if (!--ulCounter) {
#ifdef DEBUG_PINS
                digitalWrite(3, !digitalRead(3));
                digitalWrite(2, LOW);
#endif
				return 0;		// Timeout
			}
		}
	} while (ch != 0xff) ;
	*psz++ = 0xff;
	while (psz != pszEnd) {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = s_paxStream->read()) == -1) {
			if (!--ulCounter)  {
#ifdef DEBUG_PINS
                digitalWrite(3, !digitalRead(3));
                digitalWrite(2, LOW);
#endif
				return 0;		// Timeout
			}
		}
		*psz++ = (unsigned char)ch;
	}
    checksum = 0;
#ifdef DEBUG_PINS
    digitalWrite(2, LOW);
#endif
    for(offset=2;offset<length;offset++)
        checksum += ax_rx_buffer[offset];
    if(checksum != 255){
#ifdef DEBUG
		Serial.println("");
		for(offset=0;offset<length;offset++) {
			Serial.print(ax_rx_buffer[offset], HEX);
			Serial.print(" ");
		}
		Serial.println("");
#endif		
#ifdef DEBUG_PINS
        digitalWrite(3, !digitalRead(3));
#endif
        return 0;
    }else{
        return 1;
    }
}


/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length){  
    setTX(id);
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
	
    setRX(id);    
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
    setTX(id);    
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
    setRX(id);
    //ax12ReadPacket();
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id);    
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
    setRX(id);
    //ax12ReadPacket();
}

