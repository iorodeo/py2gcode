"""

Copyright 2013 IO Rodeo Inc. 

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
from __future__ import print_function
import gcode_cmds as g

class GCodeProg(object):

    def __init__(self):
        self.listOfCmds = []
        self.lineNumbers = False
        self.lineNumberStep = 2 

    def add(self,cmd,comment=False):
        if comment:
            cmd.comment = True
        self.listOfCmds.append(cmd)

    def addGenericStart(self,feedrate=None,units='in',comment=True):
        self.add(g.Space())
        self.add(g.Comment('Generic Start'))
        self.add(g.CancelCutterCompensation(),comment=comment)
        self.add(g.CancelToolLengthOffset(),comment=comment)
        self.add(g.CancelCannedCycle(),comment=comment)
        self.add(g.CoordinateSystem(1),comment=comment)
        self.add(g.AbsoluteMode(),comment=comment)
        self.add(g.Units(units),comment=comment)
        self.add(g.ExactPathMode(),comment=comment)
        if feedrate is not None:
            self.add(g.FeedRate(feedrate),comment=comment)

    def __str__(self):
        if self.lineNumbers:
            listOfStr = ['N{0} {1}'.format(i,x) for i,x in enumerate(self.listOfCmds)] 
        else:
            listOfStr = [x.__str__() for x in self.listOfCmds]
        return '\n'.join(listOfStr)

    def write(self,filename):
        with open(filename,'w') as f:
            f.write(self.__str__());


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    prog = GCodeProg()
    prog.addGenericStart()
    prog.add(g.Space())

    prog.add(g.FeedRate(10.0))
    prog.add(g.Space())

    prog.add(g.Comment('Move to start'))
    prog.add(g.RapidMotion(x=1,y=1,z=1))
    prog.add(g.Space())

    prog.add(g.Comment('Cut box'))
    prog.add(g.LinearFeed(z=0))
    prog.add(g.LinearFeed(x=1,y=-1))
    prog.add(g.LinearFeed(x=-1,y=-1))
    prog.add(g.LinearFeed(x=-1,y=1))
    prog.add(g.LinearFeed(x=1,y=1))
    prog.add(g.RapidMotion(x=1,y=1,z=1))
    
    prog.add(g.Space())
    prog.add(g.End(),comment=True)

    print(prog)
    prog.write('test.ngc')







        

