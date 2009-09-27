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

# TODO: Add checks for "need save" on close...

import sys, time, os
sys.path.append("tools")
import wx

from ax12 import *
# drivers expose init(port, baud)
#                execute(id, instr, params)
#                setReg(id, start_addr, vals)
#                getReg(id, start_addr, length)
#                syncWrite(regstart, ((id1, val1, val2..), (id2, val1, val2...), ..) )
import arbotix
#import direct

from PoseEditor import *
from SeqEditor import *
from project import *

VERSION = "PyPose v0.95"

###############################################################################
# Main editor window
class editor(wx.Frame):
    """ Implements the main window. """
    ID_NEW=wx.NewId()
    ID_OPEN=wx.NewId()
    ID_SAVE=wx.NewId()
    ID_SAVE_AS=wx.NewId()
    ID_EXIT=wx.NewId()
    ID_EXPORT=wx.NewId()
    ID_RELAX=wx.NewId()
    ID_PORT=wx.NewId()
    ID_ABOUT=wx.NewId()
    ID_TEST=wx.NewId()

    def __init__(self):
        """ Creates pose editor window. """
        wx.Frame.__init__(self, None, -1, VERSION)     

        # data for our program
        self.project = project() # holds data for our project
        self.tools = dict() # our tool instances
        self.toolIndex = dict() # existant tools
        self.saveReq = False
        self.panel = None
        self.port = None
        self.filename = ""
        self.dirname = ""
        
        # build our menu bar  
        menubar = wx.MenuBar()
        prjmenu = wx.Menu()
        prjmenu.Append(self.ID_NEW, "new") # dialog with name, # of servos
        prjmenu.Append(self.ID_OPEN, "open") # open file dialog
        prjmenu.Append(self.ID_SAVE,"save") # if name unknown, ask, otherwise save
        prjmenu.Append(self.ID_SAVE_AS,"save as") # ask for name, save
        prjmenu.AppendSeparator()
        prjmenu.Append(self.ID_EXIT,"exit") 
        menubar.Append(prjmenu, "project")

        toolsmenu = wx.Menu()
        # find our tools
        toolFiles = list()
        for file in os.listdir("tools"):
            if file[-3:] == '.py' and file != "__init__.py" and file != "ToolPane.py":
                toolFiles.append(file[0:-3])       
        # load tool names, give them IDs
        for t in toolFiles:
            module = __import__(t, globals(), locals(), ["NAME"])    
            name = getattr(module, "NAME")
            id = wx.NewId()
            self.toolIndex[id] = (t, name)
            toolsmenu.Append(id,name)   
        toolsmenu.Append(self.ID_EXPORT,"export to AVR") # save as dialog
        menubar.Append(toolsmenu,"tools")

        configmenu = wx.Menu()
        configmenu.Append(self.ID_PORT,"port") # dialog box: arbotix/thru, speed, port
        configmenu.Append(self.ID_TEST,"test") # for in-house testing of boards
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
    
        for t in self.toolIndex.keys():
            wx.EVT_MENU(self, t, self.loadTool)
        wx.EVT_MENU(self, self.ID_EXPORT, self.export)     

        wx.EVT_MENU(self, self.ID_RELAX, self.doRelax)   
        wx.EVT_MENU(self, self.ID_PORT, self.doPort)
        wx.EVT_MENU(self, self.ID_TEST, self.doTest)
        wx.EVT_MENU(self, self.ID_ABOUT, self.doAbout)

        # editor area       
        self.sb = self.CreateStatusBar(2)
        self.sb.SetStatusWidths([-1,150])
        self.sb.SetStatusText('not connected',1)

        self.loadTool()
        self.sb.SetStatusText('please create or open a robot file...',0)
        self.Centre()
        # small hack for windows... 9-25-09 MEF
        self.SetBackgroundColour(wx.NullColor) # wx.SystemSettings.GetColour(wx.SYS_COLOUR_MENU)
        self.Show(True)

    ###########################################################################
    # toolpane handling   
    def loadTool(self, e=None):
        if e == None:
            t = "PoseEditor"
        else:
            t = self.toolIndex[e.GetId()][0]  # get name of file for this tool  
            if self.tool == t:
                return
        if self.panel != None:
            self.panel.save()
            self.sizer.Remove(self.panel)
            self.panel.Destroy()
        self.ClearBackground()
        module = __import__(t, globals(), locals(), [t,"STATUS"])
        panelClass = getattr(module, t)
        self.panel = panelClass(self,self.port)
        self.sizer=wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.panel,1,wx.EXPAND|wx.ALL,10)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.sb.SetStatusText(getattr(module,"STATUS"),0)
        self.tool = t

    ###########################################################################
    # file handling                
    def newFile(self, e):  
        """ Open a dialog that asks for robot name and servo count. """ 
        dlg = NewProject(self, -1, "Create New Project")
        if dlg.ShowModal() == wx.ID_OK:
            self.project.new(dlg.name.GetValue(), dlg.count.GetValue())
            self.loadTool()      
            self.sb.SetStatusText('created new project ' + self.project.name + ', please create a pose...')
            self.panel.saveReq = True
        dlg.Destroy()

    def openFile(self, e):
        """ Loads a robot file into the GUI. """ 
        dlg = wx.FileDialog(self, "Choose a file", self.dirname, "", "*.ppr", wx.OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.filename = dlg.GetPath()
            self.dirname = dlg.GetDirectory()
            print "Opening: " + self.filename            
            self.project.load(self.filename)  
            self.SetTitle(VERSION+": " + self.project.name)
            dlg.Destroy()
            self.loadTool()
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
        self.project.save(self.filename)
        self.sb.SetStatusText('saved ' + self.filename)
        self.panel.saveReq = False

    def saveFileAs(self, e):
        self.filename = ""
        self.saveFile()                

    ###########################################################################
    # Export functionality
    def export(self, e):        
        """ Export a pose file for use with Sanguino Library. """
        if self.project.name == "":
            self.sb.SetStatusText('please create a project')
            return
        dlg = wx.FileDialog(self, "Choose a file", self.dirname,"","*.h",wx.SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            self.project.export(dlg.GetPath())
            self.sb.SetStatusText("exported " + dlg.GetPath(),0)
            dlg.Destroy()        

    ###########################################################################
    # Port Manipulation
    def doPort(self, e=None):
        if self.port != None:
            self.port.ser.close()
        dlg = wx.TextEntryDialog(self,'Port (Ex. COM4 or /dev/ttyUSB0)', 'Select Communications Port')
        dlg.SetValue("/dev/ttyUSB0")
        if dlg.ShowModal() == wx.ID_OK:
            print "Opening port: " + str(dlg.GetValue())
            try:
                # TODO: add ability to select types of ports
                self.port = arbotix.ax12(str(dlg.GetValue()), 38400)
                self.panel.port = self.port
                self.sb.SetStatusText(str(dlg.GetValue()) + "@38400",1)
            except:
                self.port = None
                self.sb.SetStatusText('not connected',1)
            dlg.Destroy()

    def doTest(self, e=None):
        if self.port != None:
            self.port.execute(253, 25, list())

    def doRelax(self, e=None):
        """ Relax servos so you can pose them. """
        if self.port != None:
            for servo in range(self.robot.count):
                self.port.setReg(servo+1,P_TORQUE_ENABLE, [0,])    
        print "PyPose: relaxing servos..."      

    def doAbout(self, e=None):
        license= """This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

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
# New Project Dialog
class NewProject(wx.Dialog):
    def __init__(self, parent, id, title):
        wx.Dialog.__init__(self, parent, id, title, size=(220, 140))

        panel = wx.Panel(self, -1)
        vbox = wx.BoxSizer(wx.VERTICAL)

        wx.StaticBox(panel, -1, 'Project Parameters', (5, 5), (210, 80))
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

