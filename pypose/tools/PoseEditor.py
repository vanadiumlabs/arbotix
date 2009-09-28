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
import project
from ToolPane import ToolPane
from ax12 import *

###############################################################################
# pose editor window
class PoseEditor(ToolPane):
    """ editor for the capture and creation of poses. """
    BT_RELAX = 100
    BT_CAPTURE = 101
    BT_SET = 102
    BT_POSE_ADD = 103
    BT_POSE_REM = 104
    ID_POSE_BOX = 105

    def __init__(self, parent, port=None):
        ToolPane.__init__(self, parent, port)
        self.curpose = "" 
        self.saveReq = False

        sizer = wx.GridBagSizer(10,10)

        # pose editor, goes in a box:
        temp = wx.StaticBox(self, -1, 'edit pose')
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        editBox = wx.StaticBoxSizer(temp,orient=wx.VERTICAL) 
        poseEditSizer = wx.GridBagSizer(5,5)
        # build servo editors
        self.servos = list() # the editors in the window
        for i in range(self.parent.project.count):
            temp = wx.Panel(self,-1)
            hbox = wx.BoxSizer(wx.HORIZONTAL)

            if (i+1)<10:
                tempName = wx.StaticText(temp, -1, "ID 0"+str(i+1))   
            else:
                tempName = wx.StaticText(temp, -1, "ID "+str(i+1))
            temp.position = wx.Slider(temp, i, 512, 0, 1023, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
            hbox.Add(tempName)
            hbox.Add(temp.position)
            temp.SetSizer(hbox)
            if i == 0:
                poseEditSizer.Add(temp, (i/2, 0), wx.GBSpan(1,1), wx.TOP,10)
            elif i == 1:
                poseEditSizer.Add(temp, (i/2, 1), wx.GBSpan(1,1), wx.TOP,10)
            elif i%2 == 0:
                poseEditSizer.Add(temp, (i/2, 0))
            else:
                poseEditSizer.Add(temp, (i/2, 1))
            temp.Disable()  # servo editors start out disabled, enabled only when a pose is selected
            self.servos.append(temp)
        # grid it
        editBox.Add(poseEditSizer)
        sizer.Add(editBox, (0,0), wx.GBSpan(1,1), wx.EXPAND)

        # list of poses
        self.posebox = wx.ListBox(self, self.ID_POSE_BOX, choices=self.parent.project.poses.keys())
        sizer.Add(self.posebox, (0,1), wx.GBSpan(1,1), wx.EXPAND) 
        # and add/remove
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_POSE_ADD, 'add'))
        hbox.Add(wx.Button(self, self.BT_POSE_REM, 'remove'))     
        sizer.Add(hbox,(1,1),wx.GBSpan(1,1),wx.ALIGN_CENTER)

        # toolbar
        toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RELAX, 'relax'),1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_CAPTURE, 'capture'),1)         
        toolbarsizer.Add(wx.Button(toolbar, self.BT_SET, 'set'),1)
        toolbar.SetSizer(toolbarsizer)
        sizer.Add(toolbar, (1,0), wx.GBSpan(1,1), wx.ALIGN_CENTER)
       
        self.Bind(wx.EVT_SLIDER, self.updatePose)
        self.parent.Bind(wx.EVT_CHAR, self.onChar)
        wx.EVT_BUTTON(self, self.BT_RELAX, self.parent.doRelax)    
        wx.EVT_BUTTON(self, self.BT_CAPTURE, self.capturePose)    
        wx.EVT_BUTTON(self, self.BT_SET, self.setPose) 
        wx.EVT_BUTTON(self, self.BT_POSE_ADD, self.addPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_REM, self.remPose)   
        wx.EVT_LISTBOX(self, self.ID_POSE_BOX, self.doPose)

        self.SetSizerAndFit(sizer)
            
    def onChar(self, e=None):
        if e.ControlDown():
            c = e.GetKeyCode()  
            print c
                
        else:
            print "skip!"
            e.skip()        



    ###########################################################################
    # Pose Manipulation
    def updatePose(self, e=None):
        """ Save updates to a pose. """
        if self.curpose != "":
            self.parent.project.poses[self.curpose][e.GetId()] = e.GetInt()
    def doPose(self, e=None):
        """ Load a pose into the slider boxes. """
        if e.IsSelection():
            if self.curpose == "":   # if we haven't yet, enable servo editors
                for servo in self.servos:
                    servo.Enable()
            self.curpose = str(e.GetString())
            for servo in range(self.parent.project.count):
                self.servos[servo].position.SetValue(self.parent.project.poses[self.curpose][servo])
            self.parent.sb.SetStatusText('now editing pose: ' + self.curpose,0)
            
    def capturePose(self, e=None):
        """ Downloads the current pose from the robot to the GUI. """
        if self.port != None and self.curpose != "":   
            errors = "could not read servos: "
            dlg = wx.ProgressDialog("capturing pose","this may take a few seconds, please wait...",self.parent.project.count + 1)
            dlg.Update(1)
            for servo in range(self.parent.project.count):
                pos = self.port.getReg(servo+1,P_PRESENT_POSITION_L, 2)
                if pos != -1:
                    self.servos[servo].position.SetValue(pos[0] + (pos[1]<<8))
                else: 
                    errors = errors + str(servo+1) + ", "
                if self.curpose != "":                
                    self.parent.project.poses[self.curpose][servo] = self.servos[servo].position.GetValue() 
                val = servo+2
                dlg.Update(val)  
            if errors != "could not read servos: ":
                self.parent.sb.SetStatusText(errors[0:-2],0)   
            else:
                self.parent.sb.SetStatusText("captured pose!",0)    
            dlg.Destroy()

    def setPose(self, e=None):
        """ Write a pose out to the robot. """
        if self.port != None and self.curpose != "":
            #curPose = list() TODO: should we use a syncWrite here?
            for servo in range(self.parent.project.count):
                 pos = self.servos[servo].position.GetValue()
                 self.port.setReg(servo+1, P_GOAL_POSITION_L, [pos%256, pos>>8])
                 self.parent.project.poses[self.curpose][servo] = self.servos[servo].position.GetValue()                 
            #    pos = self.servos[servo].position.get()
            #    curPose.append( (servo+1, pos%256, pos>>8) )
            #self.pose.syncWrite(P_GOAL_POSITION_L, curPose)

    def addPose(self, e=None):
        """ Add a new pose. """
        if self.parent.project.name != "":
            dlg = wx.TextEntryDialog(self,'Pose Name:', 'New Pose Settings')
            dlg.SetValue("")
            if dlg.ShowModal() == wx.ID_OK:
                self.posebox.Append(dlg.GetValue()) 
                self.parent.project.poses[dlg.GetValue()] = project.pose("",self.parent.project.count)
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
                del self.parent.project.poses[self.curpose]
                self.posebox.Delete(v)
                self.curpose = ""
                dlg.Destroy()
                for servo in self.servos:   # disable editors if we have no pose selected
                    servo.Disable()
            self.parent.sb.SetStatusText("please create or select a pose to edit...",0)   

NAME = "pose editor"
STATUS = "please create or select a sequence to edit..."
