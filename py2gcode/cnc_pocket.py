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

FLOAT_TOLERANCE = 1.0e-12

class PocketBase(gcode_cmd.GCodeProg):

    def __init__(self,param):
        self.param = param
        safeZ = float(self.param['safeZ'])
        startZ = float(self.param['startZ'])
        assert safeZ > startZ, 'safeZ must be > startZ'
        self.listOfCmds = []
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


class RectPocketXY(PocketBase):

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
        super(RectPocketXY,self).__init__(param)

    def makeListOfCmds(self):

        # Retreive numerical parameters and convert to float 
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        width = abs(float(self.param['width']))
        height = abs(float(self.param['height']))
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
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
        checkRectPocketOverlap(overlap)
        checkRectPocketOverlap(overlapFinish)

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

        # Lead-in parameters 
        try:
            sgnX = (x1 - x0)/abs(x1 - x0)
        except ZeroDivisionError:
            sgnX = 1.0
        try: 
            sgnY = (y1 - y0)/abs(y1 - y0)
        except ZeroDivisionError:
            sgnY = 1.0
        leadInDx = min([maxCutDepth,abs(x1-x0)])
        leadInDy = min([maxCutDepth,abs(y1-y0)])
        leadInX1 = x0 + sgnX*leadInDx
        leadInY1 = y0 + sgnY*leadInDy
        leadInPoint1 = leadInX1, leadInY1 

        # Move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Get z cutting parameters 
        stopZ = startZ - depth
        prevZ = startZ
        currZ = max([startZ - maxCutDepth, stopZ])

        done = False
        passCnt = 0

        while not done:

            passCnt+=1

            # Lead-in to cut depth
            self.addComment('pass {0} lead-in'.format(passCnt))
            leadInPath = cnc_path.RectPath(point0,leadInPoint1)
            leadInCmds = leadInPath.listOfCmds
            for i, cmd in enumerate(leadInCmds):
                if i == 0:
                    continue
                cmd.motionDict['z'] = prevZ + (float(i+1)/len(leadInCmds))*(currZ - prevZ)
            self.listOfCmds.extend(leadInCmds)

            # Cut filled rectangular path
            self.addComment('pass {0} filled rectangle'.format(passCnt))
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

        self.addRapidMoveToSafeZ()
        self.addEndComment()


class RectAnnulusPocketXY(PocketBase):

    def __init__(self,param):
        """
        Generates toolpath for cutting a simple rectangulare pocket.

        params dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        width          = pocket outer width  
        height         = pocket inner width
        thickness      = thickness
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
        super(RectAnnulusPocketXY,self).__init__(param)

    def makeListOfCmds(self):

        # Retreive numerical parameters and convert to float and check
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        width = abs(float(self.param['width']))
        height = abs(float(self.param['height']))
        thickness = abs(float(self.param['thickness']))
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
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
        self.listOfCmds = []

        # Check params
        assert toolDiam <= thickness, 'toolDiam too large for annulus thickness'
        checkRectPocketOverlap(overlap)
        checkRectPocketOverlap(overlapFinish)

        # Get sign for rectangular toolpaths based on direction 
        if self.param['direction'] == 'cw':
            sgnDir = 1.0
        else:
            sgnDir = -1.0

        # Outer toolpath rectangle
        outerX0 = cx + 0.5*(-width + toolDiam)
        outerX1 = cx + 0.5*( width - toolDiam)
        outerY0 = cy + 0.5*sgnDir*(-height + toolDiam)
        outerY1 = cy + 0.5*sgnDir*( height - toolDiam)
        outerPoint0 = outerX0, outerY0
        outerPoint1 = outerX1, outerY1

        # Inner toolpath rectangle
        innerX0 = cx + 0.5*(-width - toolDiam) + thickness
        innerX1 = cx + 0.5*( width + toolDiam) - thickness
        innerY0 = cy + sgnDir*(0.5*(-height - toolDiam) + thickness)
        innerY1 = cy + sgnDir*(0.5*( height + toolDiam) - thickness)
        innerPoint0 = innerX0, innerY0
        innerPoint1 = innerX1, innerY1


        # Lead-in parameters 
        sgnX = (outerX1 - outerX0)/abs(outerX1 - outerX0)
        sgnY = (outerY1 - outerY0)/abs(outerY1 - outerY0)
        leadInDx = min([maxCutDepth,abs(outerX1-innerX0)])
        leadInDy = min([maxCutDepth,abs(outerY1-innerY0)])

        # Move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=outerX0,y=outerY0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Get z cutting parameters 
        stopZ = startZ - depth
        prevZ = startZ
        currZ = max([startZ - maxCutDepth, stopZ])

        done = False
        passCnt = 0

        while not done:

            passCnt+=1

            # Lead-in to cut depth
            self.addComment('pass {0} lead-in'.format(passCnt))
            if abs(outerX0 - innerX0) < maxCutDepth:
                leadInY = outerY0 + 2*sgnY*leadInDy
                self.listOfCmds.append(gcode_cmd.LinearFeed(y=leadInY, z=prevZ + 0.5*(currZ-prevZ)))
                self.listOfCmds.append(gcode_cmd.LinearFeed(y=outerY0, z=prevZ + 1.0*(currZ-prevZ)))
            else:
                leadInX1 = outerX0 + sgnX*leadInDx
                leadInY1 = outerY0 + sgnY*leadInDy
                leadInPoint1 = leadInX1, leadInY1 
                leadInPath = cnc_path.RectPath(outerPoint0,leadInPoint1)
                leadInCmds = leadInPath.listOfCmds
                for i, cmd in enumerate(leadInCmds):
                    if i == 0:
                        continue
                    cmd.motionDict['z'] = prevZ + (float(i+1)/len(leadInCmds))*(currZ - prevZ)
                self.listOfCmds.extend(leadInCmds)

            # Cut filled rectangular path
            self.addComment('pass {0} filled rectangle'.format(passCnt))
            if currZ == stopZ:
                passOverlap = overlapFinish
            else:
                passOverlap = overlap
            stepSize = toolDiam - passOverlap*toolDiam
            numStep = int(math.floor(thickness/stepSize))

            if not self.param['cornerCut']:
                rectPath = cnc_path.FilledRectPath(
                        outerPoint0,
                        outerPoint1,
                        stepSize,
                        numStep,
                        plane='xy'
                        )
            else:
                cutLen = 0.5*toolDiam*(math.sqrt(2.0) - 1.0) + cornerMargin
                rectPath = cnc_path.FilledRectWithCornerCutPath(
                        outerPoint0,
                        outerPoint1,
                        stepSize,
                        numStep,
                        cutLen,
                        plane='xy'
                        )
            self.listOfCmds.extend(rectPath.listOfCmds)

            test0 = abs(outerX0 - innerX0) > FLOAT_TOLERANCE
            test1 = abs(outerX0 - innerX0) >  (thickness - ((numStep-1)*stepSize + toolDiam)) 
            if test0 and test1:  
                rectPath = cnc_path.RectPath(innerPoint0, innerPoint1)
                self.listOfCmds.extend(rectPath.listOfCmds)

            # Get next z position
            if currZ <= stopZ:
                done = True
            prevZ = currZ
            currZ = max([currZ - maxCutDepth, stopZ])


        # Move to safe z and add end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


class CircPocket(PocketBase):

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

        # Check params
        assert (overlap >= 0.0 and overlap < 1.0), 'overlap must >=0 and < 1'
        assert 2*radius > toolDiam, 'circle diameter must be > tool diameter'

        # Get circle cutting parameters  - assumes startAngle=0
        adjustedRadius = radius - 0.5*toolDiam
        x0 = cx + adjustedRadius
        y0 = cy

        # Move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Get z cutting parameters 
        stopZ = startZ - depth
        prevZ = startZ
        currZ = max([startZ - maxCutDepth, stopZ])

        done = False
        passCnt = 0

        while not done:

            passCnt+=1

            # Add lead-in
            self.addComment('pass {0} lead-in'.format(passCnt))
            moveToStartCmd = gcode_cmd.LinearFeed(x=x0,y=y0)
            self.listOfCmds.append(moveToStartCmd)
            leadInRadius = min([maxCutDepth,adjustedRadius])
            leadInCenterX = x0  - 0.5*leadInRadius 
            leadInCenterY = y0
            leadInOffsetI = leadInCenterX - x0
            leadInOffsetJ = leadInCenterY - y0
            helicalCmd = gcode_cmd.HelicalMotionXY(
                    x=x0,
                    y=y0,
                    z = currZ,
                    i=leadInOffsetI,
                    j=leadInOffsetJ,
                    d = self.param['direction'],
                    )
            self.listOfCmds.append(helicalCmd)

            # Add filled circle
            self.addComment('pass {0} filled circle'.format(passCnt))
            if currZ == stopZ:
                passOverlap = overlapFinish
            else:
                passOverlap = overlap

            stepSize = toolDiam - passOverlap*toolDiam
            numStep = int(math.ceil(adjustedRadius/stepSize))
            circPath = cnc_path.FilledCircPath(
                    (cx,cy),
                    adjustedRadius,
                    stepSize,
                    numStep,
                    startAng=0,
                    plane='xy',
                    direction=self.param['direction'],
                    turns=1
                    )
            self.listOfCmds.extend(circPath.listOfCmds)
            centerMoveCmd = gcode_cmd.LinearFeed(x=cx,y=cy)
            self.listOfCmds.append(centerMoveCmd)

            # Get next z position
            if currZ <= stopZ:
                done = True
            prevZ = currZ
            currZ = max([currZ - maxCutDepth, stopZ])


        # Move to safe z and add end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


# Utility functions
# --------------------------------------------------------------------------------------
def checkRectPocketOverlap(overlap): 
    minOverlap = (1.0 - 1.0/math.sqrt(2.0))/1.0
    assertMsg = ' overlap must be >= {0} and < 1.0'.format(minOverlap)
    assert (overlap >= minOverlap  and overlap < 1.0), assertMsg 

# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(100.0))

    if 0:
        param = {
                'centerX'       : 0.0,
                'centerY'       : 0.0,
                'width'         : 2.0,
                'height'        : 1.0,
                'depth'         : 0.04,
                'startZ'        : 0.0,
                'safeZ'         : 0.5,
                'overlap'       : 0.3,
                'overlapFinish' : 0.5,
                'maxCutDepth'   : 0.04,
                'toolDiam'      : 0.25,
                'cornerCut'     : False,
                'direction'     : 'ccw',
                'startDwell'  : 2.0,
                }

        pocket = RectPocketXY(param)

    if 1:
        param = {
                'centerX'       : 0.0,
                'centerY'       : 0.0,
                'width'         : 1.0,
                'height'        : 1.0,
                'thickness'     : 0.35,
                'depth'         : 0.1,
                'startZ'        : 0.0,
                'safeZ'         : 0.5,
                'overlap'       : 0.3,
                'overlapFinish' : 0.6,
                'maxCutDepth'   : 0.04,
                'toolDiam'      : 0.2,
                'cornerCut'     : False,
                'direction'     : 'ccw',
                'startDwell'  : 2.0,
                }

        pocket = RectAnnulusPocketXY(param)


    if 0:
        param = { 
                'centerX'        : 0.0, 
                'centerY'        : 0.0,
                'radius'         : 0.4,
                'depth'          : 0.1,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.1,
                'overlapFinish'  : 0.1,
                'maxCutDepth'    : 0.03,
                'toolDiam'       : 0.25,
                'direction'      : 'ccw',
                'startDwell'   : 2.0,
                }

        pocket = CircPocket(param)

    prog.add(pocket)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')


