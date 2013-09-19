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
import math
import gcode_cmd


class DrillBase(gcode_cmd.GCodeProg):

    def __init__(self,param):
        self.param = param
        self.makeListOfCmds()

    def makeListOfCmds(self):
        self.listOfCmds = []

    def addStartComment(self):
        # Add pocket start comment
        self.listOfCmds = []
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = 'Begin {0}'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        for k,v in self.param.iteritems():
            self.listOfCmds.append(gcode_cmd.Comment('{0}: {1}'.format(k,v)))

    def addEndComment(self):
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = 'End {0}'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        self.listOfCmds.append(gcode_cmd.Space())

    def addComment(self,comment): 
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = '{0}: {1}'.format(self.__class__.__name__, comment)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))

    def addRapidMoveToSafeZ(self):
        safeZ = float(self.param['safeZ'])
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = '{0}: rapid move to safe z'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.RapidMotion(z=safeZ))

    def addRapidMoveToPos(self,**kwarg):
        self.listOfCmds.append(gcode_cmd.Space())
        try:
            pointName = kwarg.pop('comment')
        except KeyError:
            pointName = '{0}'.format(kwarg)
        commentStr = '{0}: rapid move to {1}'.format(self.__class__.__name__, pointName)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.RapidMotion(**kwarg))

    def addMoveToStartZ(self):
        startZ = float(self.param['startZ'])
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = '{0}: move to start z'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.LinearFeed(z=startZ))

    def addDwell(self,t):
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = '{0}: dwell'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Dwell(t))


class SimpleDrill(DrillBase):

    def __init__(self, param):
        super(SimpleDrill,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        startZ = float(self.param['startZ'])
        stopZ = float(self.param['stopZ'])
        try:
            startDwell = self.param['startDwell']
        except KeyError:
            startDwell = 0.0

        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=cx,y=cy,comment='drill x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()
        drillCmd = gcode_cmd.DrillCycle(x=cx,y=cy,z=stopZ,r=startZ)
        self.listOfCmds.append(drillCmd)
        self.addRapidMoveToSafeZ()
        self.addEndComment()


# -----------------------------------------------------------------------------
if __name__ == '__main__':


    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(2.0))

    param = {
            'centerX': 0.5,
            'centerY': 0.2,
            'startZ': 0.02,
            'stopZ' : -0.5,
            'safeZ' : 0.5,
            'startDwell' : 2.0,
            }

    drill = SimpleDrill(param)

    prog.add(drill)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')
