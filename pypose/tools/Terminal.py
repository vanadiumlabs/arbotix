#!/usr/bin/env python

""" 
  PyPose: Bioloid pose system for arbotiX robocontroller
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

import wx
from ax12 import *
import arbotix
from ToolPane import ToolPane

# help phrases
help = ["\rPyPose Terminal VA.0",
"\r",
"\rvalid commands:",
"\rli [baud] - list the servos found on the bus, default baud = 1MBps",
"\rmv id id2 - rename any servo with ID=id, to id2",
"\rset param id val - set parameter on servo ID=id to val",
"\rget param id - get a parameter value from a servo",
"\r",
"\rvalid parameters",
"\rpos - current position of a servo, 0-1023",
"\rtemp - current temperature, degrees C, READ ONLY"]

class shell(wx.TextCtrl):
    """ The actual terminal part. """
    def __init__(self, parent, id=-1, conn=None, font_size=10, encoding="utf-8"):
        self.parent = parent
        self.encoding = encoding
        self.conn = conn
        mono = wx.Font(font_size, wx.FONTFAMILY_MODERN, wx.NORMAL, wx.NORMAL)
        dc = wx.ScreenDC()
        dc.SetFont(mono)
        (cw, ch) = dc.GetTextExtent("X")
        self.cw = cw
        self.ch = ch
        self.cols = 82
        self.rows = 25
        wx.TextCtrl.__init__(self, parent, id, size=(82*cw+2, 25*ch+2),
        style=wx.TE_MULTILINE|wx.TE_PROCESS_TAB|wx.TE_PROCESS_ENTER|wx.HSCROLL)
        
        self.SetFont(mono)
        self.Bind(wx.EVT_CHAR, self.OnEnterChar)        
        
        self.SetValue("PyPose Terminal VA.0\r>> ")
        self.SetInsertionPoint(len(self.GetValue()))

    def OnEnterChar(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_RETURN:
            # process the command!
            line = self.PositionToXY(self.GetLastPosition())[1]
            l = self.GetLineText(line)[3:].split(" ")
            try:
                if l[0] == u"help":   # display help data
                    if len(l) > 1:      # for a particular command
                        if l[1] == u"li":   
                            self.write(help[3])
                        elif l[1] == u"mv":
                            self.write(help[4])
                        elif l[1] == u"set":
                            self.write(help[5])
                        elif l[1] == u"get":   
                            self.write(help[6])
                    else:
                        for h in help:
                            self.write(h)
                elif l[0] == u"clear":
                    self.Clear()                                  
                    self.SetValue(">> ")
                    self.SetInsertionPoint(3)
                    return                      
                elif l[0] == u"serial":
                    # open a serial port
                    if self.parent.parent.port != None:
                        self.parent.parents.port.ser.close()
                    print "Opening port: " + l[1]
                    try:
                        # TODO: add ability to select type of driver
                        self.port = arbotix.ax12(str(l[1]), 38400)
                        self.parent.parent.port = self.port
                        self.parent.parent.sb.SetStatusText(str(l[1]) + "@38400",1)
                        self.write("\rOK!")
                    except:
                        self.port = None
                        self.parent.parent.sb.SetStatusText('not connected',1)
                        # print error
                        self.parent.parent.sb.SetBackgroundColour('RED')
                        self.parent.parent.sb.SetStatusText("Could Not Open Port",0) 
                        self.parent.parent.timer.Start(20)  
                elif self.parent.port == None:
                    self.write("\rNo port open!")
                elif l[0] == u"li":      # list servos
                    #baud = 1000000
                    #if len(l) > 1:       # we have a baud too!
                    #    baud = int(l[1])
                    k = 0                # how many id's have we printed...
                    self.write("\r")
                    for i in range(18):
                        if self.parent.port.getReg(i+1,P_PRESENT_POSITION_L, 1) != -1:
                            if k > 8:    # limit the width of each printout
                                k = 0
                                self.write("\r")
                            self.write(repr(i+1).rjust(4)) 
                            k = k + 1
                            wx.SafeYield()
                elif l[0] == u"mv":      # rename a servo
                    if self.parent.port.setReg(int(l[1]),P_ID,[int(l[2])]) == 0:
                        self.write("\rOK")
                elif l[0] == u"set":
                    pass
                elif l[0] == u"get":
                    pass
            except:
                self.write("\runrecognized command!")
            # new line!
            self.write("\r>> ")
        elif keycode == wx.WXK_BACK:
            if(self.PositionToXY(self.GetLastPosition())[0] > 3):
                self.Remove(self.GetLastPosition()-1,self.GetLastPosition())
        else:
            self.write(unichr(keycode))

class Terminal(ToolPane):
    """ arbotix/bioloid terminal. """
    def __init__(self, parent, port=None):
        ToolPane.__init__(self, parent, port)
        self.term = shell(self)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.term, 1, wx.EXPAND|wx.ALL, 1)
        self.SetSizer(sizer)
        self.Fit()
        sizer.SetSizeHints(self)
        self.Show(True)

NAME = "terminal"
STATUS = "opened terminal..."
