#!/usr/bin/env python

""" 
  PyPose: Serial driver for direct connection via USBDynamixel

  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import serial, time, sys
from binascii import b2a_hex
from ax12 import *

class ax12:
    """ Class to open a serial port and control AX-12 servos, 
    typically through a HUV servo board. """
    def __init__(self, port="COM4",baud=1000000):
        try:
            self.ser = serial.Serial()
            self.ser.baudrate = baud
            self.ser.port = port
            self.ser.timeout = 3
            self.ser.open()
        except:
            print "Cannot open port" + str(sys.exc_info()[0])
            sys.exit(0)

    def setReg(self, index, regstart, values):
        """ Set the value of registers. Should be called as such:
        ax12.setReg(1,1,(0x01,0x05)) """ 
        self.ser.flushInput()
        length = 3 + len(values)
        checksum = 255 - ((index + length + AX_WRITE_DATA + regstart + sum(values))%256)
        # packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM
        self.ser.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_WRITE_DATA)+chr(regstart))
        for val in values:
            self.ser.write(chr(val))
        self.ser.write(chr(checksum))
        time.sleep(0.05)
        # print the return information        
        while self.ser.inWaiting() > 0:
              print b2a_hex(self.ser.read()),
        print ""

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params = None):
        # need a positive byte
        d = self.ser.read()
        if d == '': 
            print "Fail Read"
            return None

        # now process our byte
        if mode == 0:           # get our first 0xFF
            if ord(d) == 0xff:   
                print "Oxff found"
                return self.getPacket(1)
            else:
                print "Oxff NOT found, restart: " + str(ord(d))
                return self.getPacket(0)
        elif mode == 1:         # get our first 0xFF
            if ord(d) == 0xff:
                print "Oxff found"
                return self.getPacket(2)
            else:
                print "Oxff NOT found, restart"
                return self.getPacket(0)
        elif mode == 2:         # get id
            if d != 0xff:
                print "ID found: " + str(ord(d))
                return self.getPacket(3, ord(d))
            else:              
                print "0xff is not ID, restart"
                return self.getPacket(0)
        elif mode == 3:         # get length
            print "Length found: " + str(ord(d))
            return self.getPacket(4, id, ord(d))
        elif mode == 4:         # read error    
            print "Error level found: " + str(ord(d))
            return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:         # read params
            print "Parameter found: " + str(ord(d))
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:         # read checksum
            checksum = id + leng + error + sum(params) + ord(d)
            print "Checksum computed: " + str(checksum)
            if checksum % 256 != 255:
                print "Checksum ERROR"
                return None
            return params
        # fail
        return None

    def getReg(self, index, regstart, rlength):
        """ Get the value of registers, should be called as such:
        ax12.getReg(1,1,1) """
        self.ser.flushInput()   
        # send packet: FF FF ID LENGTH INS(0x02) PARAM .. CHECKSUM
        checksum = 255 - ((6 + index + regstart + rlength)%256)
        self.ser.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(0x04)+chr(AX_READ_DATA)+chr(regstart)+chr(rlength)+chr(checksum))
        # read packet: FF FF ID LENGTH ERROR PARAMS CHECKSUM
        vals = self.getPacket(0)           
        if vals == None:
            print "Read Failed: Servo ID = " + str(index)
            return -1        
        if rlength == 1:
            return vals[0]
        return vals

    def syncWrite(self, regstart, vals):
        """ Set the value of registers. Should be called as such:
        ax12.syncWrite(reg, ((id1, val1, val2), (id2, val1, val2))) """ 
        self.ser.flushInput()
        length = 4
        valsum = 0
        for i in vals:
            length = length + len(i)    
            valsum = valsum + sum(i)
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + regstart + len(vals[0]) - 1 + valsum)%256)
        # packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM
        self.ser.write(chr(0xFF)+chr(0xFF)+chr(0xFE)+chr(length)+chr(AX_SYNC_WRITE)+chr(regstart)+chr(len(vals[0])-1))
        for servo in vals:
            for value in servo:
                self.ser.write(chr(value))
        self.ser.write(chr(checksum))
        # no return info...

