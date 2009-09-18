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

import sys, time
import wx

from ax12 import *
# drivers expose init(port, baud)
#                execute(id, instr, params)
#                setReg(id, start_addr, vals)
#                getReg(id, start_addr, length)
#                syncWrite(regstart, ((id1, val1, val2..), (id2, val1, val2...), ..) )
import arbotix
#import direct

from robot import *

VERSION = "PyPose v0.93"

###############################################################################
# Main editor window
class editor(wx.Frame):
    """ Implements the main window. """
    ID_NEW=101
    ID_OPEN=102
    ID_SAVE=103
    ID_SAVE_AS=104
    ID_EXIT=106
    ID_POSE=107
    ID_SEQ=108
    ID_EXPORT=109
    ID_RELAX=110
    ID_PORT=111
    ID_ABOUT=112
    ID_NUKE=113

    def __init__(self):
        """ Creates pose editor window. """
        wx.Frame.__init__(self, None, -1, VERSION)     

        # data for our program
        self.robot = robot()
        self.saveReq = False
        self.panel = None
        self.port = None
        self.filename = ""
        self.dirname = ""
        
        # build our menu bar  
        menubar = wx.MenuBar()
        robotmenu = wx.Menu()
        robotmenu.Append(self.ID_NEW, "new") # dialog with name, # of servos
        robotmenu.Append(self.ID_OPEN, "open") # open file dialog
        robotmenu.Append(self.ID_SAVE,"save") # if name unknown, ask, otherwise save
        robotmenu.Append(self.ID_SAVE_AS,"save as") # ask for name, save
        robotmenu.AppendSeparator()
        robotmenu.Append(self.ID_EXIT,"exit") 
        menubar.Append(robotmenu, "robot")

        toolsmenu = wx.Menu()
        toolsmenu.Append(self.ID_POSE,"pose editor") # do pose creation
        toolsmenu.Append(self.ID_SEQ,"sequence editor") # do sequencing
        toolsmenu.Append(self.ID_NUKE,"NUKE") # do IK
        toolsmenu.Append(self.ID_EXPORT,"export to AVR") # save as dialog
        menubar.Append(toolsmenu,"tools")

        configmenu = wx.Menu()
        configmenu.Append(self.ID_PORT,"port") # dialog box: arbotix/thru, speed, port
        # live update?
        menubar.Append(configmenu, "config")    

        helpmenu = wx.Menu()
        helpmenu.Append(self.ID_ABOUT,"about")
        menubar.Append(helpmenu,"help")

        self.SetMenuBar(menubar)    

        # configure events
        wx.EVT_MENU(self, self.ID_NEW, self.newFile)
        wx.EVT_MENU(self, self.ID_OPEN, self.openFile)
        wx.EVT_MENU(self, self.ID_SAVE, self.saveFile)
        wx.EVT_MENU(self, self.ID_SAVE_AS, self.saveFileAs)
        wx.EVT_MENU(self, self.ID_EXIT, sys.exit)
    
        wx.EVT_MENU(self, self.ID_POSE, self.loadPosePane)
        wx.EVT_MENU(self, self.ID_SEQ, self.loadSeqPane)
        wx.EVT_MENU(self, self.ID_EXPORT, self.export)     

        wx.EVT_MENU(self, self.ID_RELAX, self.doRelax)   
        wx.EVT_MENU(self, self.ID_PORT, self.doPort)
        wx.EVT_MENU(self, self.ID_ABOUT, self.doAbout)

        # editor area       
        self.sb = self.CreateStatusBar(2)
        self.sb.SetStatusWidths([-1,150])
        self.sb.SetStatusText('not connected',1)

        self.loadPosePane()
        self.sb.SetStatusText('please create or open a robot file...',0)
        self.Centre()
        self.Show(True)

    ###########################################################################
    # pane handling      
    def replacePanel(self, panelClass):
        # do save here? 
        if self.panel != None:
            self.panel.save()
            self.panel.Destroy()
        self.ClearBackground()
        self.panel = panelClass(self)
        self.sizer=wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.panel,1,wx.EXPAND|wx.ALL,10)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)

    def loadPosePane(self, e=None):
        self.replacePanel(poseEditor)
        self.sb.SetStatusText('please create or select a pose to edit...',0)

    def loadSeqPane(self, e=None):
        self.replacePanel(seqEditor) 
        self.sb.SetStatusText('please create or select a sequence to edit...',0)

    ###########################################################################
    # file handling                
    def newFile(self, e):  
        """ Open a dialog that asks for robot name and servo count. """ 
        dlg = NewRobot(self, -1, "Create New Robot")
        if dlg.ShowModal() == wx.ID_OK:
            self.robot.new(dlg.name.GetValue(), dlg.count.GetValue())
            self.loadPosePane()      
            self.sb.SetStatusText('created new robot ' + self.robot.name + ', please create a pose...')
            self.panel.saveReq = True
        dlg.Destroy()

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
        self.panel.saveReq = False

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
            self.sb.SetStatusText("exported " + dlg.GetPath(),0)
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

    def doRelax(self, e=None):
        """ Relax servos so you can pose them. """
        if self.port != None:
            for servo in range(self.robot.count):
                self.port.setReg(servo+1,P_TORQUE_ENABLE, [0,])    
        print "PyPose: relaxing servos..."      

    def doAbout(self, e=None):
        license= """This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA)
"""
        info = wx.AboutDialogInfo()
        info.SetName(VERSION)
        info.SetDescription("A lightweight pose and capture software for the arbotiX robocontroller")
        info.SetCopyright("Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.")
        info.SetLicense(license)
        info.SetWebSite("http://www.vanadiumlabs.com")
        wx.AboutBox(info)

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
        wx.Panel.__init__(self,parent,style=wx.TAB_TRAVERSAL)
        self.parent = parent  
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
        for i in range(self.parent.robot.count):
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
        self.posebox = wx.ListBox(self, self.ID_POSE_BOX, choices=self.parent.robot.poses.keys())
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
        wx.EVT_BUTTON(self, self.BT_RELAX, self.parent.doRelax)    
        wx.EVT_BUTTON(self, self.BT_CAPTURE, self.capturePose)    
        wx.EVT_BUTTON(self, self.BT_SET, self.setPose) 
        wx.EVT_BUTTON(self, self.BT_POSE_ADD, self.addPose)   
        wx.EVT_BUTTON(self, self.BT_POSE_REM, self.remPose)   
        wx.EVT_LISTBOX(self, self.ID_POSE_BOX, self.doPose)

        self.SetSizerAndFit(sizer)

    def save(self):
        pass    
    #    if self.curpose != "" and self.saveReq == True:
    #        dlg = wx.MessageDialog(self, 'Do You Want to Save Changes to this Pose?', self.curpose + ' changed', wx.OK|wx.CANCEL|wx.ICON_EXCLAMATION)
    #        if dlg.ShowModal() == wx.ID_OK:
    #            for servo in range(self.parent.robot.count):
    #                self.parent.robot.poses[self.curpose][servo] = self.servos[servo].position.GetValue() 
    #        dlg.Destroy()
    #        self.saveReq == False

            
    ###########################################################################
    # Pose Manipulation
    def updatePose(self, e=None):
        """ Save updates to a pose. """
        if self.curpose != "":
            self.parent.robot.poses[self.curpose][e.GetId()] = e.GetInt()
    def doPose(self, e=None):
        """ Load a pose into the slider boxes. """
        if e.IsSelection():
            if self.curpose == "":   # if we haven't yet, enable servo editors
                for servo in self.servos:
                    servo.Enable()
            self.curpose = str(e.GetString())
            for servo in range(self.parent.robot.count):
                self.servos[servo].position.SetValue(self.parent.robot.poses[self.curpose][servo])
            self.parent.sb.SetStatusText('now editing pose: ' + self.curpose,0)
            
    def capturePose(self, e=None):
        """ Downloads the current pose from the robot to the GUI. """
        if self.parent.port != None and self.curpose != "":   
            errors = "could not read servos: "
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
        if self.parent.port != None and self.curpose != "":
            #curPose = list()
            for servo in range(self.parent.robot.count):
                 pos = self.servos[servo].position.GetValue()
                 self.parent.port.setReg(servo+1, P_GOAL_POSITION_L, [pos%256, pos>>8])
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
                for servo in self.servos:   # disable editors if we have no pose selected
                    servo.Disable()
            self.parent.sb.SetStatusText("please create or select a pose to edit...",0)   


###############################################################################
# Sequence editor window
class seqEditor(wx.Panel):
    """ editor for the creation of sequences. """
    BT_MOVE_UP = 101
    BT_MOVE_DN = 102
    BT_RELAX = 103
    BT_RUN = 104
    BT_HALT = 105 
    BT_SEQ_ADD = 106
    BT_SEQ_REM = 107
    BT_TRAN_ADD = 108
    BT_TRAN_REM = 109

    ID_SEQ_BOX = 110
    ID_TRAN_BOX = 111
    ID_TRAN_POSE = 112
    ID_TRAN_TIME = 113

    def __init__(self, parent):
        wx.Panel.__init__(self,parent,style=wx.TAB_TRAVERSAL)
        self.parent = parent        
        self.curseq = ""
        self.curtran = -1

        sizer = wx.GridBagSizer(10,10)
    
        # sequence editor, goes in a box:
        temp = wx.StaticBox(self, -1, 'edit sequence')
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        editBox = wx.StaticBoxSizer(temp,orient=wx.VERTICAL)
        seqEditSizer = wx.GridBagSizer(5,5)
        
        # transitions list
        temp = wx.StaticText(self, -1, "transitions:")
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        seqEditSizer.Add(temp, (0,0), wx.GBSpan(1,1), wx.TOP,10)
        self.tranbox = wx.ListBox(self, self.ID_TRAN_BOX)
        seqEditSizer.Add(self.tranbox, (1,0), wx.GBSpan(5,1), wx.EXPAND|wx.ALL) 
        # and add/remove
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_TRAN_ADD, 'new'))
        hbox.Add(wx.Button(self, self.BT_TRAN_REM, 'delete'))     
        seqEditSizer.Add(hbox,(6,0),wx.GBSpan(1,1),wx.ALIGN_CENTER)
        
        # pose name & delta-T
        seqEditSizer.Add(wx.StaticText(self, -1, "pose:"), (1,1))
        self.tranPose = wx.ComboBox(self, self.ID_TRAN_POSE, choices=self.parent.robot.poses.keys())
        seqEditSizer.Add(self.tranPose, (1,2))
        seqEditSizer.Add(wx.StaticText(self, -1, "delta-T:"), (2,1))
        self.tranTime = wx.SpinCtrl(self, self.ID_TRAN_TIME, '1000', min=1, max=5000)
        seqEditSizer.Add(self.tranTime, (2,2))
        # buttons to move transition up/down
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_MOVE_UP, 'move up'))
        hbox.Add(wx.Button(self, self.BT_MOVE_DN, 'move down'))     
        seqEditSizer.Add(hbox,(4,1),wx.GBSpan(1,2),wx.ALIGN_CENTER|wx.BOTTOM,10)
        # grid it
        editBox.Add(seqEditSizer)
        sizer.Add(editBox, (0,0), wx.GBSpan(1,1), wx.EXPAND)

        # list of sequences
        self.seqbox = wx.ListBox(self, self.ID_SEQ_BOX, choices=self.parent.robot.sequences.keys())
        sizer.Add(self.seqbox, (0,1), wx.GBSpan(1,1), wx.EXPAND) 
        # and add/remove
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_SEQ_ADD, 'add'))
        hbox.Add(wx.Button(self, self.BT_SEQ_REM, 'remove'))     
        sizer.Add(hbox,(1,1),wx.GBSpan(1,1),wx.ALIGN_CENTER)

        # toolbar
        toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RELAX, 'relax'),1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RUN, 'run'),1)         
        toolbarsizer.Add(wx.Button(toolbar, self.BT_HALT, 'halt'),1)
        toolbar.SetSizer(toolbarsizer)
        sizer.Add(toolbar, (1,0), wx.GBSpan(1,1), wx.ALIGN_CENTER)

        self.SetSizerAndFit(sizer)

        wx.EVT_BUTTON(self, self.BT_RELAX, self.parent.doRelax)    
        wx.EVT_BUTTON(self, self.BT_RUN, self.runSeq)    
        wx.EVT_BUTTON(self, self.BT_HALT, self.haltSeq) 
        wx.EVT_BUTTON(self, self.BT_SEQ_ADD, self.addSeq)
        wx.EVT_BUTTON(self, self.BT_SEQ_REM, self.remSeq)   
        wx.EVT_LISTBOX(self, self.ID_SEQ_BOX, self.doSeq)
        wx.EVT_BUTTON(self, self.BT_MOVE_UP, self.moveUp)
        wx.EVT_BUTTON(self, self.BT_MOVE_DN, self.moveDn)
        wx.EVT_BUTTON(self, self.BT_TRAN_ADD, self.addTran)
        wx.EVT_BUTTON(self, self.BT_TRAN_REM, self.remTran)   
        wx.EVT_LISTBOX(self, self.ID_TRAN_BOX, self.doTran)
        
        wx.EVT_COMBOBOX(self, self.ID_TRAN_POSE, self.updateTran)
        wx.EVT_SPINCTRL(self, self.ID_TRAN_TIME, self.updateTran)
     
    def save(self):            
        if self.curseq != "":
            self.parent.robot.sequences[self.curseq] = sequence()
            for i in range(self.tranbox.GetCount()):
                self.parent.robot.sequences[self.curseq].append(self.tranbox.GetString(i).replace(",","|"))               
   
    ###########################################################################
    # Sequence Manipulation
    def doSeq(self, e=None):
        """ save previous sequence changes, load a sequence into the editor. """
        if e.IsSelection():
            self.save()            
            self.curseq = str(e.GetString())
            self.curtran = -1
            for i in range(self.tranbox.GetCount()):
                self.tranbox.Delete(0)      # TODO: There has got to be a better way to do this??
            for transition in self.parent.robot.sequences[self.curseq]:
                self.tranbox.Append(transition.replace("|",","))
            self.tranPose.SetValue("")
            self.tranTime.SetValue(500)
            self.parent.sb.SetStatusText('now editing sequence: ' + self.curseq)

    def addSeq(self, e=None):       
        """ create a new sequence. """
        if self.parent.robot.name != "":
            dlg = wx.TextEntryDialog(self,'Sequence Name:', 'New Sequence Settings')
            dlg.SetValue("")
            if dlg.ShowModal() == wx.ID_OK:
                self.seqbox.Append(dlg.GetValue())
                self.parent.robot.sequences[dlg.GetValue()] = sequence("")
                dlg.Destroy()
        else:
            dlg = wx.MessageDialog(self, 'Please create a new robot first.', 'Error', wx.OK|wx.ICON_EXCLAMATION)
            dlg.ShowModal()
            dlg.Destroy()
    def remSeq(self, e=None):
        """ remove a sequence. """
        if self.curseq != "":
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this sequence?', 'Confirm', wx.OK|wx.CANCEL|wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                # this order is VERY important!
                v = self.seqbox.FindString(self.curseq)
                del self.parent.robot.sequences[self.curseq]
                self.seqbox.Delete(v)
                self.curseq = ""
                dlg.Destroy()

    ###########################################################################
    # Transition Manipulation
    def doTran(self, e=None):
        """ load a transition into the editor. """
        if e.IsSelection():
            if self.curseq != "":
                self.curtran = e.GetInt()
                v = str(e.GetString())   
                self.tranPose.SetValue(v[0:v.find(",")])
                self.tranTime.SetValue(int(v[v.find(",")+1:]))
            
    def addTran(self, e=None):       
        """ create a new transtion in this sequence. """
        if self.curseq != "":
            if self.curtran != -1:
                self.tranbox.Insert("none,500",self.curtran+1)
            else:
                self.tranbox.Append("none,500")
    def remTran(self, e=None):
        """ remove a sequence. """
        if self.curseq != "" and self.curTran != -1:
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this transition?', 'Confirm', wx.OK|wx.CANCEL|wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                self.tranbox.Delete(self.curtran)
                self.curtran = -1
                self.tranPose.SetValue("")
                self.tranTime.SetValue(500)
                dlg.Destroy()

    def moveUp(self, e=None):
        if self.curtran > 0:
            self.tranbox.Delete(self.curtran)
            self.curtran = self.curtran - 1
            self.tranbox.Insert(self.tranPose.GetValue() + "," + str(self.tranTime.GetValue()), self.curtran)
            self.tranbox.SetSelection(self.curtran)
    def moveDn(self, e=None):
        if self.curtran < self.tranbox.GetCount()-1:
            self.tranbox.Delete(self.curtran)
            self.curtran = self.curtran + 1
            self.tranbox.Insert(self.tranPose.GetValue() + "," + str(self.tranTime.GetValue()), self.curtran)   
            self.tranbox.SetSelection(self.curtran)
    def updateTran(self, e=None):
        if self.curtran != -1:
            self.tranbox.Delete(self.curtran)
            self.tranbox.Insert(self.tranPose.GetValue() + "," + str(self.tranTime.GetValue()), self.curtran)
            print "Updated: " + self.tranPose.GetValue() + "," + str(self.tranTime.GetValue()), self.curtran
            self.tranbox.SetSelection(self.curtran)

    def extract(self, li):
        """ extract x%256,x>>8 for every x in li """
        out = list()
        for i in li:
            out = out + [i%256,i>>8]
        return out        

    def runSeq(self, e=None):
        """ download poses, seqeunce, and send. """
        self.save() # save sequence            
        if self.parent.port != None and self.curseq != "":
            poseDL = dict()     # key = pose name, val = index, download them after we build a transition list
            tranDL = list()     # list of bytes to download
            for t in self.parent.robot.sequences[self.curseq]:  
                p = t[0:t.find("|")]                    # pose name
                dt = int(t[t.find("|")+1:])             # delta-T
                if p not in poseDL.keys():
                    poseDL[p] = len(poseDL.keys())      # get ix for pose
                # create transition values to download
                tranDL.append(poseDL[p])                # ix of pose
                tranDL.append(dt%256)                   # time is an int (16-bytes)
                tranDL.append(dt>>8)
            tranDL.append(255)      # notice to stop
            tranDL.append(0)        # time is irrelevant on stop    
            tranDL.append(0)
            # set pose size -- IMPORTANT!
            print "Setting pose size at " + str(self.parent.robot.count)
            self.parent.port.execute(253, 7, [self.parent.robot.count])
            #print self.parent.port.ser.readline()
            # send out poses (each pose is sent as id, servo_1, servo_2....)
            for p in poseDL.keys():
                print "Sending pose " + str(p) + " to position " + str(poseDL[p])
                self.parent.port.execute(253, 8, [poseDL[p]] + self.extract(self.parent.robot.poses[p])) 
                #print self.parent.port.ser.readline() 
            # send out sequence
            print "Sending sequence: " + str(tranDL)
            self.parent.port.execute(253, 9, tranDL) 
            #while True:
            #    x = self.parent.port.ser.readline() 
            #    if x == '':
            #        break
            #    print x,          
            # play sequence
            self.parent.port.execute(253, 10, list())
            self.parent.sb.SetStatusText('Playing Sequence: ')
    def haltSeq(self, e=None):
        """ send halt message ("H") """ 
        if self.parent.port != None:
            self.parent.port.ser.write("H")

###############################################################################
# New Robot Dialog
class NewRobot(wx.Dialog):
    def __init__(self, parent, id, title):
        wx.Dialog.__init__(self, parent, id, title, size=(220, 140))

        panel = wx.Panel(self, -1)
        vbox = wx.BoxSizer(wx.VERTICAL)

        wx.StaticBox(panel, -1, 'Robot Parameters', (5, 5), (210, 80))
        wx.StaticText(panel, -1, 'Name:', (15,30))
        self.name = wx.TextCtrl(panel, -1, '', (105,25)) 
        wx.StaticText(panel, -1, '# of Servos:', (15,55))
        self.count = wx.SpinCtrl(self, -1, '18', (105, 50), min=1, max=30)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        okButton = wx.Button(self, wx.ID_OK, 'Ok', size=(70, 30))
        closeButton = wx.Button(self, wx.ID_CANCEL, 'Close', size=(70, 30))
        hbox.Add(okButton, 1)
        hbox.Add(closeButton, 1, wx.LEFT, 5)

        vbox.Add(panel)
        vbox.Add(hbox, 1, wx.ALIGN_CENTER | wx.TOP | wx.BOTTOM, 10)

        self.SetSizer(vbox)    


if __name__ == "__main__":
    print "PyPose starting... "
    app = wx.PySimpleApp()
    frame = editor()
    app.MainLoop()

