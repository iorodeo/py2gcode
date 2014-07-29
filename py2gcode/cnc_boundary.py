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
import cnc_routine


class BoundaryBase(cnc_routine.SafeZRoutine):
    """
    Base class for boundary cutting routines
    """

    def __init__(self,param):
        super(BoundaryBase,self).__init__(param)

    def getZList(self):
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
        maxCutDepth = abs(float(self.param['maxCutDepth']))
        zList = [startZ]
        stopZ = startZ - depth
        while zList[-1] > stopZ:
            zList.append(max([zList[-1] - maxCutDepth, stopZ]))
        return zList

    def getZPairsList(self):
        zList = self.getZList()
        zPairsList = zip(zList[:-1], zList[1:])
        zPairsList.append((zList[-1],zList[-1]))
        return zPairsList


class RectBoundaryXY(BoundaryBase):
    """ 
    
    Generates a tool path for cutting a rectangular boundary

    """

    def __init__(self,param):
        """
        param dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        width          = rectangle width  
        height         = rectangle height 
        depth          = cut depth  
        radius         = corner radius
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        direction      = cut direction 'cw' or 'ccw'
        toolDiam       = tool diameter
        toolOffset     = inside, outside, none
        maxCutDepth    = maximum per pass cutting depth 
        startDwell     = dwell duration before start (optional)
        """
        super(RectBoundaryXY,self).__init__(param)

    def makeListOfCmds(self):

        # Extract basic cutting parameters
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        width = abs(float(self.param['width']))
        height = abs(float(self.param['height']))
        safeZ = float(self.param['safeZ'])
        direction = self.param['direction']
        try:
            radius = self.param['radius']
        except KeyError:
            radius = None
        if radius is not None:
            radius = abs(float(radius))

        startDwell = self.getStartDwell()

        # Compensate for tool offset 
        toolOffset = self.param['toolOffset']
        if toolOffset in ('inside', 'outside'):
            toolDiam = abs(float(self.param['toolDiam']))
            if toolOffset == 'inside':
                width -= toolDiam
                height -= toolDiam
            elif toolOffset == 'outside':
                width += toolDiam
                height += toolDiam
        else:
            if toolOffset is not None:
                raise ValueError, 'uknown tool offset'.format(toolOffset)


        # Get list of rectPaths
        zPairsList = self.getZPairsList()
        rectPathList = []
        for z0, z1 in zPairsList:
            rectPath = cnc_path.RectPath.fromCenter(
                    cx,
                    cy,
                    width,
                    height,
                    direction,
                    radius=radius,
                    plane='xy',
                    helix = (z0,z1),
                    )
            rectPathList.append(rectPath)

        # Get x,y coord of first point
        firstRectPath = rectPathList[0]
        firstPointList = firstRectPath.getPathPointList()
        x0, y0 = firstPointList[0][:2]

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        for i, rectPath in enumerate(rectPathList):
            self.addComment('RectPath {0}'.format(i))
            self.listOfCmds.extend(rectPath.listOfCmds)

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


class CircBoundaryXY(BoundaryBase):
    """

    Generates toolpath for cutting a circular boundary.

    """

    def __init__(self,param):
        """
        param dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        radius         = circular radius
        depth          = cut depth  
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        direction      = cut direction 'cw' or 'ccw'
        toolDiam       = tool diameter
        toolOffset     = inside, outside, none
        maxCutDepth    = maximum per pass cutting depth 
        startDwell     = dwell duration before start (optional)
        """
        super(CircBoundaryXY,self).__init__(param)

    def makeListOfCmds(self):

        # Extract basic cutting parameters
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        radius = abs(float(self.param['radius']))
        safeZ = float(self.param['safeZ'])
        direction = self.param['direction']
        startDwell = self.getStartDwell()
        try:
            startAng = self.param['startAng']
        except KeyError:
            startAng = 0.0
        startAng = float(startAng)

        # Compensate for tool offset 
        toolOffset = self.param['toolOffset']
        if toolOffset in ('inside', 'outside'):
            toolDiam = abs(float(self.param['toolDiam']))
            if toolOffset == 'inside':
                radius -= 0.5*toolDiam
            elif toolOffset == 'outside':
                radius += 0.5*toolDiam
        else:
            if toolOffset is not None:
                raise ValueError, 'uknown tool offset'.format(toolOffset)

        zPairsList = self.getZPairsList()

        # Get list of circPaths
        circPathList = []
        for z0, z1 in zPairsList:
            circPath = cnc_path.CircPath(
                    (cx, cy),
                    radius,
                    startAng=startAng,
                    plane='xy', 
                    direction=direction,
                    helix = (z0,z1)
                    )
            circPathList.append(circPath)

        # Get x,y coord of first point
        firstCircPath = circPathList[0]
        x0, y0 = firstCircPath.getStartPoint()[:2]

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        for i,circPath in enumerate(circPathList):
            self.addComment('CircPath {0}'.format(i))
            self.listOfCmds.extend(circPath.listOfCmds)

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


class LineSegBoundaryXY(BoundaryBase):
    """

    Generates toolpath for cutting a boundary based on a segment path.

    """
    
    def __init__(self,param):
        """
        param dict

        keys          values
        --------------------------------------------------------------
        pointList      = list of (x,y) points in line segement boundary  
        depth          = cut depth  
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        toolDiam       = tool diameter
        cutterComp     = left, right, inside, outside, none (NOT DONE)
        maxCutDepth    = maximum per pass cutting depth 
        startDwell     = dwell duration before start (optional)
        closed         = whether or not path is open or closed.
        """
        super(LineSegBoundaryXY,self).__init__(param)

    def checkForSelfIntersect(self):
        pass

    def getCutterCompLeadIn(self,p0,p1):
        toolDiam = abs(float(self.param['toolDiam']))
        x0, y0 = p0
        x1, y1 = p1
        moveLen = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        xNorm = (x1-x0)/moveLen 
        yNorm = (y1-y0)/moveLen 
        xLead = x0 - xNorm*toolDiam
        yLead = y0 - yNorm*toolDiam
        return xLead, yLead

    def makeListOfCmds(self):
        pointList = self.param['pointList']
        safeZ = float(self.param['safeZ'])
        startDwell = self.getStartDwell()
        toolDiam = abs(float(self.param['toolDiam']))

        if self.param['closed']:
            pointList.append(pointList[0])
        else:
            pointListRev = pointList[::-1]
            pointList.extend(pointListRev)

        # Get list of line segment paths
        zPairsList = self.getZPairsList()
        lineSegPathList = []
        for z0, z1 in zPairsList:
            lineSegPath = cnc_path.LineSegPath(
                    pointList,
                    closed=False,
                    plane='xy',
                    helix=(z0,z1)
                    )
            lineSegPathList.append(lineSegPath)

        cutterComp = self.param['cutterComp']
        if cutterComp is not None:
            if cutterComp not in ('inside', 'outside', 'left', 'right'):
                raise ValueError, 'unknown cutter compensation value {0}'.format(cutterComp)
            if cutterComp in ('inside', 'outside'):
                # TO DO
                # ---------------------------------------------------------------------------
                # Convert cutter comp to 'left' or 'right' based on line seg path
                # ---------------------------------------------------------------------------
                raise ValueError, 'inside/outside cutter compensation not implemented yet'
            
        # Get x,y coord of first point
        firstPath = lineSegPathList[0]
        x0, y0 = firstPath.getStartPoint()[:2]

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        
        if cutterComp is not None:
            # Move back to leadin start 
            xLead, yLead = self.getCutterCompLeadIn(pointList[0],pointList[1])
            self.addRapidMoveToPos(x=xLead,y=yLead, comment='cutterComp start move')

            # Add cutter compensation
            compCmd = gcode_cmd.CutterCompensation(cutterComp,diameter=toolDiam)
            self.listOfCmds.append(compCmd)

            # Return to start 
            self.addRapidMoveToPos(x=x0,y=y0, comment='cutterComp start move')

        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Add cutting paths
        for i, path in enumerate(lineSegPathList):
            self.addComment('LineSegPath {0}'.format(i))
            self.listOfCmds.extend(path.listOfCmds)

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()

        # Cancel cutter compensation
        self.listOfCmds.append(gcode_cmd.CancelCutterCompensation())
        xLead, yLead = self.getCutterCompLeadIn(pointList[-1], pointList[-2])
        self.addRapidMoveToPos(x=xLead,y=yLead,comment='cancel cutter comp move')
        xEnd, yEnd = pointList[-1]
        self.addRapidMoveToPos(x=xEnd,y=yEnd,comment='cancel cutter comp move') 
        self.addEndComment()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Devel Tests

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(100.0))

    if 0:
        param = { 
                'centerX'      : 0.0,
                'centerY'      : 0.0,
                'width'        : 2.0,
                'height'       : 2.0,
                'depth'        : 0.2,
                'radius'       : 0.25,
                'startZ'       : 0.0,
                'safeZ'        : 0.15,
                'toolDiam'     : 0.25,
                'toolOffset'   : 'outside',
                'direction'    : 'ccw',
                'maxCutDepth'  : 0.03,
                'startDwell'   : 2.0,
                
                }
        boundary = RectBoundaryXY(param)

    if 0:
        param = { 
                'centerX'      : 0.0,
                'centerY'      : 0.0,
                'radius'       : 0.5,
                'depth'        : 0.2,
                'startZ'       : 0.0,
                'safeZ'        : 0.15,
                'toolDiam'     : 0.25,
                'toolOffset'   : 'outside',
                'direction'    : 'ccw',
                'maxCutDepth'  : 0.03,
                'startDwell'   : 2.0,
                }
        boundary = CircBoundaryXY(param)

    if 1:

        pointList = [
                (0,0),
                (1,0),
                (2,1),
                (2,1.5),
                (1,1.5),
                (0,1),
                (-1,1),
                (-1,0),
                ]

        param = {
                'pointList'   : pointList,
                'depth'       : 0.03,
                'startZ'      : 0.0,
                'safeZ'       : 0.15,
                'toolDiam'    : 0.25,
                'cutterComp'  : 'right',
                'maxCutDepth' : 0.03,
                'startDwell'  : 2.0,
                'closed'      : True,
                }
        boundary = LineSegBoundaryXY(param)

    if 0:
        pointList = [
                (0,0),
                (1,0),
                (2,1),
                (2,1.5),
                (1,1.5),
                (0,1),
                (-1,1),
                (-1,0),
                ]

        param = {
                'pointList'   : pointList,
                'depth'       : 0.03,
                'startZ'      : 0.0,
                'safeZ'       : 0.15,
                'toolDiam'    : 0.25,
                'cutterComp'  : 'right',
                'maxCutDepth' : 0.03,
                'startDwell'  : 2.0,
                'closed'      : False,
                }
        boundary = LineSegBoundaryXY(param)


    prog.add(boundary)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')


