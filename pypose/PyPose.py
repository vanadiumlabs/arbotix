#!/usr/bin/env python

""" 
  PyPose: Bioloid pose system for arbotiX robocontroller

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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
"""

import sys, time
import wx

from ax12 import *
# drivers expose init(port, baud)
#                setReg(id, start_addr, vals)
#                getReg(id, start_addr, length)
#                syncWrite(regstart, ((id1, val1, val2..), (id2, val1, val2...), ..) )
import arbotix
import direct

from dialog import *
from robot import *

VERSION = "PyPose V0.91"

ID_NEW=101
ID_OPEN=102
ID_SAVE=103
ID_SAVE_AS=104
ID_EXIT=106
ID_RELAX=107
ID_CAPTURE=108
ID_SET=109
ID_EXPORT=110
ID_POSE=111
ID_POSE_ADD=115
ID_POSE_REM=116
ID_SEQ=112
ID_NUKE=113
ID_PORT=114
ID_MOVE_UP=117
ID_MOVE_DN=118

###############################################################################
# Main editor window
class editor(wx.Frame):
    """ Implements the main window. """
    def __init__(self):
        """ Creates pose editor window. """
        wx.Frame.__init__(self, None, -1, VERSION)     

        # data for our program
        self.robot = robot()
        self.seqPane = None
        self.posePane = None
        self.port = None
        self.filename = ""
        self.dirname = ""
        
        # build our menu bar  
        menubar = wx.MenuBar()
        robotmenu = wx.Menu()
        robotmenu.Append(ID_NEW, "new") # dialog with name, # of servos
        robotmenu.Append(ID_OPEN, "open") # open file dialog
        robotmenu.Append(ID_SAVE,"save") # if name unknown, ask, otherwise save
        robotmenu.Append(ID_SAVE_AS,"save as") # ask for name, save
        robotmenu.AppendSeparator()
        robotmenu.Append(ID_EXIT,"exit") 
        menubar.Append(robotmenu, "robot")

        toolsmenu = wx.Menu()
        toolsmenu.Append(ID_POSE,"pose editor") # do pose creation
        toolsmenu.Append(ID_SEQ,"sequence editor") # do sequencing
        #toolsmenu.Append(ID_NUKE,"NUKE") # do IK
        toolsmenu.Append(ID_EXPORT,"export to AVR") # save as dialog
        menubar.Append(toolsmenu,"tools")

        configmenu = wx.Menu()
        configmenu.Append(ID_PORT,"port") # dialog box: arbotix/thru, speed, port
        menubar.Append(configmenu, "config")    
        self.SetMenuBar(menubar)

        # configure events
        wx.EVT_MENU(self, ID_NEW, self.newFile)
        wx.EVT_MENU(self, ID_OPEN, self.openFile)
        wx.EVT_MENU(self, ID_SAVE, self.saveFile)
        wx.EVT_MENU(self, ID_SAVE_AS, self.saveFileAs)
        wx.EVT_MENU(self, ID_EXIT, sys.exit)
    
        wx.EVT_MENU(self, ID_POSE, self.loadPosePane)
        wx.EVT_MENU(self, ID_SEQ, self.loadSeqPane)
        wx.EVT_MENU(self, ID_EXPORT, self.export)     

        wx.EVT_MENU(self, ID_RELAX, self.relax)   
        wx.EVT_MENU(self, ID_PORT, self.doPort)

        # editor area       
        self.sb = self.CreateStatusBar(2)
        self.sb.SetStatusWidths([-1,150])
        self.sb.SetStatusText('please create or open a robot file...',0)
        self.sb.SetStatusText('not connected',1)

        self.loadPosePane()

        self.Centre()
        self.Show(True)

    ###########################################################################
    # pane handling      
    def loadPosePane(self, e=None):
        if self.posePane != None:
            self.posePane.Destroy()
        if self.seqPane != None:
            self.seqPane.Destroy()
            self.seqPane = None
        self.ClearBackground()
        self.posePane = poseEditor(self)
        self.sizer=wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.posePane,0,wx.EXPAND)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)

    def loadSeqPane(self, e=None):
        if self.posePane != None:
            self.posePane.Destroy()
            self.posePane = None
        if self.seqPane != None:
            self.seqPane.Destroy()
        self.ClearBackground()
        self.seqPane = seqEditor(self)
        self.sizer=wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.seqPane,0,wx.EXPAND)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)    

    ###########################################################################
    # file handling                
    def newFile(self, e):  
        """ Open a dialog that asks for robot name and servo count. """
        #TODO:n = newFileDialog(self, "New Robot File Settings")  
        #if n.result == None:    
        #    return
        #self.robot.new(n.result[0], int(n.result[1])) 
        #self.title(VERSION+": " + n.result[0])   
        self.loadPosePane()      

    def openFile(self, e):
        """ Loads a robot file into the GUI. """ 
        dlg = wx.FileDialog(self, "Choose a file", self.dirname, "", "*.ppr", wx.OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.filename = dlg.GetPath()
            self.dirname = dlg.GetDirectory()
            print "Opening: " + self.filename            
            self.robot.load(self.filename)  
            self.SetTitle(VERSION+": " + self.robot.name)
            dlg.Destroy()
        self.loadPosePane()
        self.sb.SetStatusText('opened ' + self.filename)

    def saveFile(self, e=None):
        """ Save a robot file from the GUI. """
        if self.filename == "": 
            dlg = wx.FileDialog(self, "Choose a file", self.dirname,"","*.ppr",wx.SAVE)
            if dlg.ShowModal() == wx.ID_OK:
                self.filename = dlg.GetPath()
                self.dirname = dlg.GetDirectory()
                dlg.Destroy()
            else:
                return
        self.robot.save(self.filename)
        self.sb.SetStatusText('saved ' + self.filename)

    def saveFileAs(self, e):
        self.filename = ""
        self.saveFile()                

    ###########################################################################
    # Export functionality
    def export(self, e):        
        """ Export a pose file for use with Sanguino Library. """
        dlg = wx.FileDialog(self, "Choose a file", self.dirname,"","*.h",wx.SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            self.robot.export(dlg.GetPath())
            self.sb.SetStatusText("exported " + dlg.getPath(),0)
            dlg.Destroy()        

    ###########################################################################
    # Port Manipulation
    def doPort(self, e):
        if self.port != None:
            self.port.ser.close()
        dlg = wx.TextEntryDialog(self,'Port (Ex. COM4 or /dev/ttyUSB0)', 'Select Communications Port')
        dlg.SetValue("/dev/ttyUSB0")
        if dlg.ShowModal() == wx.ID_OK:
            print "Opening port: " + str(dlg.GetValue())
            try:
                self.port = arbotix.ax12(str(dlg.GetValue()), 38400)
                self.sb.SetStatusText(str(dlg.GetValue()) + "@38400",1)
            except:
                self.port = None
                self.sb.SetStatusText('not connected',1)
            dlg.Destroy()

    def relax(self, e=None):
        """ Relax servos so you can pose them. """
        if self.port != None:
            for servo in range(self.robot.count):
                self.port.setReg(servo+1,P_TORQUE_ENABLE, [0,])    
        print "PyPose: relaxing servos..."        

###############################################################################
# pose editor window
class poseEditor(wx.Panel):
    """ editor for the capture and creation of poses. """
    BT_RELAX = 100
    BT_CAPTURE = 101
    BT_SET = 102
    BT_POSE_ADD = 103
    BT_POSE_REM = 104
    ID_POSE_BOX = 105

    def __init__(self, parent):
        wx.Panel.__init__(self,parent,style=wx.TAB_TRAVERSAL|wx.SUNKEN_BORDER)
        self.parent = parent  
        self.curpose = "" 

        sizer = wx.GridBagSizer(5,5)

        # build servo editors
        self.servos = list() # the editors in the window
        for i in range(self.parent.robot.count):
            temp = wx.Panel(self,-1)
            hbox = wx.BoxSizer(wx.HORIZONTAL)

            if (i+1)<10:
                tempName = wx.StaticText(temp, -1, "ID 0"+str(i+1))   
            else:
                tempName = wx.StaticText(temp, -1, "ID "+str(i+1))
            temp.position = wx.Slider(temp, -1, 512, 0, 1023, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
            hbox.Add(tempName)
            hbox.Add(temp.position)
            temp.SetSizer(hbox)
            if i%2 == 0:
                sizer.Add(temp, (i/2, 0))
            else:
                sizer.Add(temp, (i/2, 1))
            # TODO: parametric setup
            self.servos.append(temp)
        
        self.toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        toolbarsizer.AddStretchSpacer()
        toolbarsizer.Add(wx.Button(self.toolbar, self.BT_RELAX, 'relax'),1)
        toolbarsizer.Add(wx.Button(self.toolbar, self.BT_CAPTURE, 'capture'),1)        
        toolbarsizer.Add(wx.Button(self.toolbar, self.BT_SET, 'set'),1)
        toolbarsizer.AddStretchSpacer()
        self.toolbar.SetSizer(toolbarsizer)
        sizer.Add(self.toolbar, (self.parent.robot.count/2,0), wx.GBSpan(1,2))
        
        #Label(self, text="Poses:").grid(row=0,column=2,sticky=W+N,padx=5,pady=5)
        #sizer.Add(wx.StaticText(self, -1, "poses:                                    "), (0,2))
        self.posebox = wx.ListBox(self, self.ID_POSE_BOX, choices=self.parent.robot.poses.keys())
        self.posebox.SetMinSize(wx.Size(40,80))
        sizer.Add(self.posebox, (0,2), wx.GBSpan(self.parent.robot.count/2,1), wx.EXPAND | wx.ALL)    
        
        self.posebar = wx.Panel(self, -1)
        posebarsizer = wx.BoxSizer(wx.HORIZONTAL)
        posebarsizer.AddStretchSpacer()
        posebarsizer.Add(wx.Button(self.posebar, self.BT_POSE_ADD, 'add'),3)
        posebarsizer.Add(wx.Button(self.posebar, self.BT_POSE_REM, 'remove'),3)     
        posebarsizer.AddStretchSpacer()
        self.posebar.SetSizer(posebarsizer)
        sizer.Add(self.posebar, (self.parent.robot.count/2,2))
   
        wx.EVT_BUTTON(self, self.BT_RELAX, self.parent.relax)    
        wx.EVT_BUTTON(self, self.BT_CAPTURE, self.capturePose)    
        wx.EVT_BUTTON(self, self.BT_SET, self.setPose) 
        wx.EVT_BUTTON(self, self.BT_POSE_ADD, self.addPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_REM, self.remPose)   
        wx.EVT_LISTBOX(self, self.ID_POSE_BOX, self.doPose)

        self.SetSizerAndFit(sizer)

    ###########################################################################
    # Pose Manipulation
    def doPose(self, e=None):
        """ Save previous pose changes, load a pose into the slider boxes. """
        if e.IsSelection():
            if self.curpose != "":
                for servo in range(self.parent.robot.count):
                    self.parent.robot.poses[self.curpose][servo] = self.servos[servo].position.GetValue() 
            self.curpose = str(e.GetString())
            for servo in range(self.parent.robot.count):
                self.servos[servo].position.SetValue(self.parent.robot.poses[self.curpose][servo])
        
    def capturePose(self, e=None):
        """ Downloads the current pose from the robot to the GUI. """
        errors = "could not read servos: "
        if self.parent.port != None:   
            dlg = wx.ProgressDialog("capturing pose","this may take a few seconds, please wait...",self.parent.robot.count + 1)
            dlg.Update(1)
            for servo in range(self.parent.robot.count):
                pos = self.parent.port.getReg(servo+1,P_PRESENT_POSITION_L, 2)
                if pos != -1:
                    self.servos[servo].position.SetValue(pos[0] + (pos[1]<<8))
                else: 
                    errors = errors + str(servo+1) + ", "
                if self.curpose != "":                
                    self.parent.robot.poses[self.curpose][servo] = self.servos[servo].position.GetValue() 
                val = servo+2
                dlg.Update(val)  
            if errors != "could not read servos: ":
                self.parent.sb.SetStatusText(errors[0:-2],0)   
            else:
                self.parent.sb.SetStatusText("captured pose!",0)    
            dlg.Destroy()

    def setPose(self, e=None):
        """ Write a pose out to the robot. """
        if self.parent.port != None:
            #curPose = list()
            for servo in range(self.parent.robot.count):
                 pos = self.servos[servo].position.GetValue()
                 self.parent.port.setReg(servo+1, P_GOAL_POSITION_L, (pos%256, pos>>8))
                 self.parent.robot.poses[self.curpose][servo] = self.servos[servo].position.GetValue()                 
            #    pos = self.servos[servo].position.get()
            #    curPose.append( (servo+1, pos%256, pos>>8) )
            #self.pose.syncWrite(P_GOAL_POSITION_L, curPose)

    def addPose(self, e=None):
        """ Add a new pose. """
        if self.parent.robot.name != "":
            dlg = wx.TextEntryDialog(self,'Pose Name:', 'New Pose Settings')
            dlg.SetValue("")
            if dlg.ShowModal() == wx.ID_OK:
                self.posebox.Append(dlg.GetValue())
                self.parent.robot.poses[dlg.GetValue()] = pose("",self.parent.robot.count)
                dlg.Destroy()
        else:
            dlg = wx.MessageDialog(self, 'Please create a new robot first.', 'Error', wx.OK|wx.ICON_EXCLAMATION)
            dlg.ShowModal()
            dlg.Destroy()

    def remPose(self, e=None):
        """ Remove a pose. """
        if self.curpose != "":
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this pose?', 'Confirm', wx.OK|wx.CANCEL|wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                v = self.posebox.FindString(self.curpose)
                del self.parent.robot.poses[self.curpose]
                self.posebox.Delete(v)
                self.curpose = ""
                dlg.Destroy()


###############################################################################
# Sequence editor window
class seqEditor(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self,parent,style=wx.TAB_TRAVERSAL|wx.SUNKEN_BORDER)
        self.parent = parent        
        self.curseq = ""

        sizer = wx.GridBagSizer(5,5)

        # listbox for sequence
        sizer.Add(wx.StaticText(self, -1, "transitions:                          "), (0,0))
        self.tranbox = wx.ListBox(self, -1)
        sizer.Add(self.tranbox, (1,0), wx.GBSpan(4,1), wx.EXPAND | wx.ALL)
        
        sizer.Add(wx.StaticText(self,-1,""),(0,1))
        sizer.Add(wx.StaticText(self,-1,""),(0,2))
        # transition editor 
        edit = wx.StaticText(self, -1, "edit transition:")
        edit.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        sizer.Add(edit, (1,1))

        sizer.Add(wx.StaticText(self, -1, "pose:"), (2,1))
        self.tranPose = wx.ComboBox(self, -1, choices=self.parent.robot.poses.keys())
        sizer.Add(self.tranPose, (2,2)) #wx.TextCtrl(self,-1),(2,2))

#wx.ComboBox(int id, string value='', wx.Point pos=wx.DefaultPosition, wx.Size size=wx.DefaultSize,
#            wx.List choices=wx.EmptyList, int style=0, wx.Validator validator=wx.DefaultValidator,
#            string name=wx.ComboBoxNameStr)

        sizer.Add(wx.StaticText(self, -1, "delta-T:"), (3,1))
        self.tranTime = wx.SpinCtrl(self, -1, '1000', min=1, max=2500)
        sizer.Add(self.tranTime, (3,2))
        sizer.Add(wx.Button(self, ID_MOVE_UP, 'move up'), (4,1))
        sizer.Add(wx.Button(self, ID_MOVE_DN, 'move down'), (4,2))

        self.toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        toolbarsizer.AddStretchSpacer()
        toolbarsizer.Add(wx.Button(self.toolbar, ID_RELAX, 'relax'),1)
        toolbarsizer.Add(wx.Button(self.toolbar, ID_CAPTURE, 'capture'),1)  # run       
        toolbarsizer.Add(wx.Button(self.toolbar, ID_SET, 'set'),1)  #halt
        toolbarsizer.AddStretchSpacer()
        self.toolbar.SetSizer(toolbarsizer)
        sizer.Add(self.toolbar, (5,0), wx.GBSpan(1,3))

        sizer.Add(wx.StaticText(self, -1, "sequences:                                    "), (0,3))
        self.seqbox = wx.ListBox(self, -1)
        self.seqbox.SetMinSize(wx.Size(40,80))
        sizer.Add(self.seqbox, (1,3), wx.GBSpan(4,1), wx.EXPAND | wx.ALL)
        #self.posebox.bind("<ButtonRelease-1>",self.doPose)        
        
        self.seqbar = wx.Panel(self, -1)
        seqbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        seqbarsizer.AddStretchSpacer()
        seqbarsizer.Add(wx.Button(self.seqbar, ID_POSE_ADD, 'add'),3)
        seqbarsizer.Add(wx.Button(self.seqbar, ID_POSE_REM, 'remove'),3)     
        seqbarsizer.AddStretchSpacer()
        self.seqbar.SetSizer(seqbarsizer)
        sizer.Add(self.seqbar, (5,3))

        self.SetSizerAndFit(sizer)


    def reset(self):
        pass

    def addSeq(self):   
        pass
    def remSeq(self):
        pass

    def runSeq(self):
        pass
    def haltSeq(self):
        pass

    def doSeq(self):
        pass

    def moveUp(self):
        pass
    def moveDn(self):
        pass


if __name__ == "__main__":
    print "PyPose starting... "
    app = wx.PySimpleApp()
    frame = editor()
    app.MainLoop()

