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


class RectBoundaryXY(cnc_routine.SafeZRoutine):

    def __init__(self,param):
        """
        Generates a tool path for cutting a rectangular boundary

        param dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        width          = pocket width  
        height         = pocket height 
        depth          = pocket depth  
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
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
        safeZ = float(self.param['safeZ'])
        maxCutDepth = abs(float(self.param['maxCutDepth']))
        direction = self.param['direction']
        try:
            radius = self.param['radius']
        except KeyError:
            radius = None
        if radius is not None:
            radius = abs(float(radius))
        try:
            startDwell = self.param['startDwell']
        except KeyError:
            startDwell = 0.0
        startDwell = abs(float(startDwell))

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

        # Get z steps 
        zList = [startZ]
        stopZ = startZ - depth
        while zList[-1] > stopZ:
            zList.append(max([zList[-1] - maxCutDepth, stopZ]))

        # Get pairs of z stpes for rectPath helcies
        zPairsList = zip(zList[:-1], zList[1:])
        zPairsList.append((zList[-1],zList[-1]))

        # Get list of rectPaths
        rectPathList = []
        for z0, z1 in zPairsList:
            rectPath = cnc_path.RectPath.fromCenter(
                    cx,
                    cy,
                    width,
                    height,
                    direction,
                    radius=radius,
                    helix = (z0,z1)
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

        for rectPath in rectPathList:
            self.listOfCmds.extend(rectPath.listOfCmds)

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


class CircBoundaryXY(cnc_routine.SafeZRoutine):

    def __init__(self,param):
        """
        Generates toolpath for cutting a circular boundary.

        param dict

        keys          values
        --------------------------------------------------------------
        centerX        = center position x-coord 
        centerY        = center position y-coord
        radius         = circular radius
        depth          = pocket depth  
        startZ         = height at which to start cutting 
        safeZ          = safe tool height 
        direction      = cut direction 'cw' or 'ccw'
        toolDiam       = tool diameter
        toolOffset     = inside, outside, none
        maxCutDepth    = maximum per pass cutting depth 
        startDwell     = dwell duration before start (optional)
        """
        super(CircBoundaryXY,self).__init__(param)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(100.0))

    if 1:
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
                
                }
        boundary = RectBoundaryXY(param)

    prog.add(boundary)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')


