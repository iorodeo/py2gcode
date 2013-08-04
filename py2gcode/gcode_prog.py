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
import gcode_cmds


class GCodeProg(object):

    def __init__(self):
        self.listOfCmds = []
        self.lineNumbers = False
        self.lineNumberStep = 2 

    def add(self,cmd,comment=False):
        if comment:
            cmd.comment = True
        self.listOfCmds.append(cmd)

    def __str__(self):
        if self.lineNumbers:
            listOfStr = ['N{0} {1}'.format(i,x) for i,x in enumerate(self.listOfCmds)] 
        else:
            listOfStr = [x.__str__() for x in self.listOfCmds]
        return '\n'.join(listOfStr)

    def write(self,filename):
        pass



# -----------------------------------------------------------------------------
if __name__ == '__main__':


    prog = GCodeProg()

    prog.add(gcode_cmds.CancelCutterCompensation(),comment=True)
    prog.add(gcode_cmds.CancelToolLengthOffset(),comment=True)
    prog.add(gcode_cmds.CancelCannedCycle(),comment=True)
    print(prog)






        

