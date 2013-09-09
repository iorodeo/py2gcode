
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

class RectPocketXY(gcode_cmd.GCodeProg):

    def __init__(self,param):
        """
        Generates toolpath for cutting a simple rectangulare pocket.

        params dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        width          = pocket width  
        height         = pocket height 
        depth          = pocket depth  
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        overlap        = tool path overlap (fractional value)
        overlapFinsh   = tool path overlap for bottom layer (optional)
        maxCutDepth    = maximum per pass cutting depth 
        toolDiam       = diameter of tool
        cornerCut      = add corner cutout (Ture/False)
        direction      = cut direction cw or ccw
        startDwell     = dwell duration before start (optional)
        cornerMargin   = margin for corner cuts (options) default = 0.0 
        """
        self.param = param
        self.makeListOfCmds()

    def makeListOfCmds(self):

        # Retreive numerical parameters and convert to float 
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        width = abs(float(self.param['width']))
        height = abs(float(self.param['height']))
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
        safeZ = float(self.param['safeZ'])
        overlap = float(self.param['overlap'])
        try:
            overlapFinish = float(self.param['overlapFinish'])
        except KeyError:
            overlapFinish = overlap
        maxCutDepth = float(self.param['maxCutDepth'])
        toolDiam = abs(float(self.param['toolDiam']))
        try:
            startDwell = abs(float(self.param['startDwell']))
        except KeyError:
            startDwell = 0.0
        try:
            cornerMargin = float(self.param['cornerMargin'])
        except KeyError:
            cornerMargin = 0.0

        # Check params
        if not safeZ > startZ:
            raise ValueError, 'safeZ must > startZ'
        if overlap < 0.0 or overlap >= 1.0:
            raise ValueError, 'overlap must be >= 0 and < 1'

        # Get rectangular path parameters
        if self.param['direction'] == 'cw':
            sgnDir = 1.0
        else:
            sgnDir = -1.0
        x0 = cx + 0.5*(-width + toolDiam)
        x1 = cx + 0.5*( width - toolDiam)
        y0 = cy + 0.5*sgnDir*(-height + toolDiam)
        y1 = cy + 0.5*sgnDir*( height - toolDiam)

        point0 = x0,y0
        point1 = x1,y1

        sgnX = (x1 - x0)/abs(x1 - x0)
        sgnY = (y1 - y0)/abs(y1 - y0)

        # Lead-in parameters 
        leadInDx = min([maxCutDepth,abs(x1-x0)])
        leadInDy = min([maxCutDepth,abs(y1-y0)])
        leadInX1 = x0 + sgnX*leadInDx
        leadInY1 = y0 + sgnY*leadInDy
        leadInPoint1 = leadInX1, leadInY1 

        # Add pocket start comment
        self.listOfCmds = []
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('Begin RectPocket'))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        for k,v in self.param.iteritems():
            self.listOfCmds.append(gcode_cmd.Comment('{0}: {1}'.format(k,v)))

        # Move to safe height, then to start x,y = x0,y0
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('RectPocket: move to safe height'))
        self.listOfCmds.append(gcode_cmd.RapidMotion(z=safeZ))
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('RectPocket: move to x,y start'))
        self.listOfCmds.append(gcode_cmd.RapidMotion(x=x0,y=y0))
        self.listOfCmds.append(gcode_cmd.Dwell(startDwell))

        # Move to start height
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('RectPocket: move to z start'))
        self.listOfCmds.append(gcode_cmd.LinearFeed(z=startZ))

        # Create z cutting passes
        stopZ = startZ - depth

        prevZ = startZ
        currZ = max([startZ - maxCutDepth, stopZ])

        done = False
        passCnt = 0

        while not done:

            passCnt+=1

            # Lead-in to cut depth
            self.listOfCmds.append(gcode_cmd.Space())
            self.listOfCmds.append(gcode_cmd.Comment('RectPocket: pass {0} lead-in'.format(passCnt)))
            leadInPath = cnc_path.RectPath(point0,leadInPoint1)
            leadInCmds = leadInPath.listOfCmds
            for i, cmd in enumerate(leadInCmds):
                if i == 0:
                    continue
                cmd.motionDict['z'] = prevZ + (float(i+1)/len(leadInCmds))*(currZ - prevZ)
            self.listOfCmds.extend(leadInCmds)

            # Cut filled rectangular path
            self.listOfCmds.append(gcode_cmd.Space())
            self.listOfCmds.append(gcode_cmd.Comment('RectPocket:  pass {0} filled rectangle'.format(passCnt)))
            if currZ == stopZ:
                passOverlap = overlapFinish
            else:
                passOverlap = overlap
            stepSize = toolDiam - passOverlap*toolDiam
            numStepX = int(math.ceil(0.5*width/stepSize))
            numStepY = int(math.ceil(0.5*height/stepSize))
            numStep = min([numStepX, numStepY])
            if not self.param['cornerCut']:
                rectPath = cnc_path.FilledRectPath(point0,point1,stepSize,numStep,plane='xy')
            else:
                cutLen = 0.5*toolDiam*(math.sqrt(2.0) - 1.0) + cornerMargin
                rectPath = cnc_path.FilledRectWithCornerCutPath(point0,point1,stepSize,numStep,cutLen,plane='xy')
            self.listOfCmds.extend(rectPath.listOfCmds)

            # Get next z position
            if currZ <= stopZ:
                done = True

            prevZ = currZ
            currZ = max([currZ - maxCutDepth, stopZ])

        # Return to safe height
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('RectPocket: return to starting position'))
        self.listOfCmds.append(gcode_cmd.RapidMotion(z=safeZ))

        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('End RectPocket'))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        self.listOfCmds.append(gcode_cmd.Space())


class CircPocket(gcode_cmd.GCodeProg):

    def __init__(self,param):
        """
        param dict:

        keys              values
        --------------------------------------------------------------
        centerX        = center x-coordinate
        centerY        = center y-coordinate
        radius         = radius
        depth          = pocket depth  
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        overlap        = tool path overlap (fractional value)
        overlapFinsh   = tool path overlap for bottom layer (optional)
        maxCutDepth    = maximum per pass cutting depth 
        toolDiam       = diameter of tool
        direction      = cut direction cw or ccw
        startDwell     = dwell duration before start (optional)

        """
        self.param = param
        self.makeListOfCmds()

    def makeListOfCmds(self):
        # Retreive numerical parameters and convert to float 
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        radius = abs(float(self.param['radius']))
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
        safeZ = float(self.param['safeZ'])
        overlap = float(self.param['overlap'])
        try:
            overlapFinish = float(self.param['overlapFinish'])
        except KeyError:
            overlapFinish = overlap
        maxCutDepth = float(self.param['maxCutDepth'])
        toolDiam = abs(float(self.param['toolDiam']))
        try:
            startDwell = abs(float(self.param['startDwell']))
        except KeyError:
            startDwell = 0.0

        # Add pocket start comment
        self.listOfCmds = []
        self.listOfCmds.append(gcode_cmd.Space())
        self.listOfCmds.append(gcode_cmd.Comment('Begin CircPocket'))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        for k,v in self.param.iteritems():
            self.listOfCmds.append(gcode_cmd.Comment('{0}: {1}'.format(k,v)))





# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(100.0))

    if 0:
        param = {
                'centerX'       : 1.0,
                'centerY'       : 1.0,
                'width'         : 2.0,
                'height'        : 1.0,
                'depth'         : 0.2,
                'startZ'        : 0.0,
                'safeZ'         : 0.5,
                'overlap'       : 0.1,
                'overlapFinish' : 0.8,
                'maxCutDepth'   : 0.04,
                'toolDiam'      : 0.25,
                'cornerCut'     : False,
                'direction'     : 'ccw',
                'startDwellDt'  : 2.0,
                }

        pocket = RectPocketXY(param)


    if 1:
        param = { 
                'centerX'        : 0.0, 
                'centerY'        : 0.0,
                'radius'         : 1.5,
                'depth'          : 0.25,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.1,
                'overlapFinsh'   : 0.8,
                'maxCutDepth'    : 0.03,
                'toolDiam'       : 0.25,
                'direction'      : 'ccw',
                'startDwellDt'   : 2.0,
                }

        pocket = CircPocket(param)

    prog.add(pocket)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')


