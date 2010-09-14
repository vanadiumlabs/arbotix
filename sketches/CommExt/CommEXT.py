#!/usr/bin/env python

# CommEXT.py - ArbotiX Commander Extended Instruction Set Example
# Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Vanadium Labs LLC nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time, sys, serial

# Commander definitions
BUT_R1 = 1
BUT_R2 = 2
BUT_R3 = 4
BUT_L4 = 8
BUT_L5 = 16
BUT_L6 = 32
BUT_RT = 64
BUT_LT = 128

INPUT = 0
OUTPUT = 1
LOW = 0
HIGH = 1

class CommanderEXT():
    NO_ACTION = 0x08
    pan = 512
    tilt = 512

    def __init__(self, port):
        self.ser = serial.Serial()
        self.ser.baudrate = 38400
        self.ser.port = port
        self.ser.timeout = 0.5
        self.ser.open()

    def sendPacket(self, rjoy_h, rjoy_l, ljoy_h, ljoy_l, buttons, ext):
        # send output
        self.ser.write('\xFF')
        self.ser.write(chr(rjoy_h))
        self.ser.write(chr(rjoy_l))
        self.ser.write(chr(ljoy_h))
        self.ser.write(chr(ljoy_l))
        self.ser.write(chr(buttons))
        self.ser.write(chr(ext))
        self.ser.write(chr(255 - ((rjoy_h+rjoy_l+ljoy_h+ljoy_l+buttons+ext)%256)))
                
    def readPacket(self, inst, mode = 0, value = -1):
        d = self.ser.read()
        if d == '': 
            #print "Fail Read"
            return -1

        # now process our byte
        if mode == 0:           # get our first
            if ord(d) == 0xff:   
                #print "Oxff found"
                return self.readPacket(inst, 1)
            else:
                #print "Oxff NOT found, restart: " + str(ord(d))
                return self.readPacket(inst, 0)
        elif mode == 1:         # get our instruction
            if ord(d) == inst:
                #print "Instruction found"
                return self.readPacket(inst, 2)
            else:
                #print "Instruction NOT found, restart: " + str(ord(d))
                return self.readPacket(inst, 0)
        elif mode == 2:         # get value
            return self.readPacket(inst, 3, ord(d))
        elif mode == 3:         # get checksum
            #print "Checksum found: " + str(ord(d))
            checksum = inst + value + ord(d)
            #print "Checksum computed: " + str(checksum)
            if checksum % 256 != 255:
                #print "Checksum ERROR"
                return -1
            return value
        # fail
        return -1

    def extInstruction(self, inst):
        self.sendPacket(self.pan>>8,self.pan%256,self.tilt>>8,self.tilt%256, 0, inst)

    def readAnalog(self, id):
        """ Read an analog port, id is 0 to 7. """
        self.extInstruction(0x10 + id)
        return self.readPacket(0x10 + id)

    def readDigital(self):
        """ Read all 8 digital ports as a single byte. """
        self.extInstruction(0x1B)
        return self.readPacket(0x1B)
    
    def motorsOff(self):
        self.extInstruction(0x40)

    def leftMotor(self, power):
        """ Set left motor power, -1 to 1. """
        if power <= 1.0 and power >= -1.0:
            self.extInstruction(0x50 + int(power*10))
    def rightMotor(self, power):
        """ Set right motor power, -1 to 1. """
        if power <= 1.0 and power >= -1.0:
            self.extInstruction(0x70 + int(power*10))

    def setDigital(self, id, direction, value):
        """ Set a digital pin value. id is 0 to 7. value and direction are 0 or 1. """
        self.extInstruction(0x80 + 4*id + direction*2 + value)
            
if __name__ == "__main__":
    # commanderEXT.py <serialport>
    c = CommanderEXT(sys.argv[1])

    # Cycle digital ports using extended mode
    for i in range(8):
        c.setDigital(i, OUTPUT, HIGH)
        if i > 2:
            c.setDigital(i-2, INPUT, LOW)
        time.sleep(0.25)  
    c.setDigital(4, OUTPUT, HIGH)
    c.setDigital(6, OUTPUT, HIGH)

    # Read analog inputs
    for i in range(8):
        print c.readAnalog(i)

    # Read digital inputs
    print "Digital:", c.readDigital()

    # Exercise turret
    for i in range(20):
        c.pan = 312 + i*20
        c.extInstruction(c.NO_ACTION)
        time.sleep(.2)
    c.pan = 512
    for i in range(20):
        c.tilt = 312 + i*20
        c.extInstruction(c.NO_ACTION)
        time.sleep(.2)

