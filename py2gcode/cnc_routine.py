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
import cnc_path


class SafeZRoutine(gcode_cmd.GCodeProg):

    DEFAULT_PARAM = {} 

    def __init__(self,param):
        self.param = dict(self.DEFAULT_PARAM)
        self.param.update(param)
        safeZ = float(self.param['safeZ'])
        startZ = float(self.param['startZ'])
        assert safeZ > startZ, 'safeZ must be > startZ'
        self.listOfCmds = []
        self.makeListOfCmds()

    def makeListOfCmds(self):
        self.listOfCmds = []

    def addStartComment(self):
        self.listOfCmds = []
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = 'Begin {0}'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        for k,v in self.param.iteritems():
            vStr = str(v)
            if len(vStr) > 50:
                vStr = 'too big' 
            self.listOfCmds.append(gcode_cmd.Comment('{0}: {1}'.format(k,vStr)))

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

    def getStartDwell(self):
        try:
            startDwell = self.param['startDwell']
        except KeyError:
            startDwell = 0.0
        startDwell = abs(float(startDwell))
        return startDwell
