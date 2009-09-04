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

###############################################################################
# Pose class is a list, first element is name, rest are servo positions. 
class pose(list):
    """ A class to hold a pose. """
    def __init__(self, line, length):
        # now load the name, positions for this pose
        try:
            #self.append(line[0:line.index(":")])
            #line = line[line.index(":")+1:]
            for servo in range(length): 
                if line.find(",") > 0:
                    self.append(int(line[0:line.index(",")]))       
                else:
                    self.append(int(line[0:]))
                line = line[line.index(",")+1:] 
        # we may not have enough data, so dump it
        except:
            for i in range(length-len(self)):
                self.append(512)

    def __str__(self):
        #data = "Pose=" + self[0] + ": "
        for position in self: #[1:]:
            data = data + str(position) + ", "
        return data[0:-2]


###############################################################################
# Sequence class is a list, first element is name, rest are (pose,time) pairs 
class sequence(list):
    """ A class to hold a sequence. """
    def __init__(self, line):
        # load the name, (pose,time) pairs for this sequence
        try:
            #self.append(line[0:line.index(":")])
            #line = line[line.index(":")+1:]
            while True:
                if line.find(",") > 0:
                    self.append((line[0:line.index("|")].strip().rstrip(), int(line[line.index("|")+1:line.index(",")])))
                else:
                    self.append((line[0:line.index("|")].strip().rstrip(), int(line[line.index("|")+1:])))
                line = line[line.index(",")+1:] 
        except:
            pass

    def __str__(self):
        #data = "Seq=" + self[0] + ": "
        for translate in self: #[1:]:
            data = data + str(translate[0]) + "|" + str(translate[1]) + ", "
        return data[0:-2]


###############################################################################
# Class for dealing with robot files
class robot:
    def __init__(self):
        self.name = ""
        self.count = 0
        self.poses = dict()
        self.sequences = dict()    

    def load(self, filename):
        self.poses = dict()     
        self.sequences = dict()
        robotFile = open(filename, "r").readlines()    
        # load robot name and servo count
        self.name = robotFile[0].split(":")[0]
        self.count = int(robotFile[0].split(":")[1])    
        # load poses and sequences
        for line in robotFile[1:]:  
            if line[0:5] == "Pose=":
                self.poses[line[5:line.index(":")]] = pose(line[line.index(":")+1:].rstrip(),self.count)
            elif line[0:4] == "Seq=":
                self.sequences[line[4:line.index(":")]] = (sequence(line[line.index(":")+1:].rstrip()))    
            # these next two lines can be removed later, once everyone is moved to Ver 0.91         
            else:
                self.poses[line[0:line.index(":")]] = pose(line[line.index(":")+1:].rstrip(),self.count)   

    def save(self, filename):
        robotFile = open(filename, "w")
        print>>robotFile, self.name + ":" + str(self.count)
        for p in self.poses.keys():            
            print>>robotFile, "Pose=" + p + ":" + str(self.poses[p])
        for s in self.sequences.keys():
            print>>robotFile, "Seq=" + s + ": " + str(self.sequences[s])

    def new(self, nName, nCount):
        self.poses = dict()
        self.sequences = dict()
        self.filename = ""
        self.count = nCount
        self.name = nName

    ###########################################################################
    # Export functionality
    def export(self, filename):        
        """ Export a pose file for use with Sanguino Library. """
        posefile = open(filename, "w")
        print>>posefile, "#ifndef " + self.name.upper() + "_POSES"
        print>>posefile, "#define " + self.name.upper() + "_POSES"
        print>>posefile, ""
        print>>posefile, "#include <avr/pgmspace.h>"
        print>>posefile, ""
        for p in self.poses.keys():
            print>>posefile, "PROGMEM prog_uint16_t " + p + "[] = {0x0C,",
            p = self.poses[p]
            for x in p[1:-1]:
                print>>posefile, str(x) + ",",
            print>>posefile, str(p[-1]) + "};"
            print>>posefile, ""
        print>>posefile, "#endif"
        posefile.close()

