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
        cx = float(self.param['centerX'])
        cy = float(self.param['centerY'])
        width = abs(float(self.param['width']))
        height = abs(float(self.param['height']))
        depth = abs(float(self.param['depth']))
        startZ = float(self.param['startZ'])
        safeZ = float(self.param['safeZ'])
        maxCutDepth = abs(float(self.param['maxCutDepth']))
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
        direction = self.param['direction']
        toolOffset = self.param['toolOffset']
        if toolOffset in ('inside', 'outside'):

            toolDiam = abs(float(self.param['toolDiam']))
            if toolOffset == 'inside':
                width -= toolDiam
                height -= toolDiam
            elif toolOffset == 'outside':
                width += toolDiam
                height += toolDiam

        # Get z steps 
        stopZ = startZ - depth
        zList = [startZ]
        while zList[-1] > stopZ:
            zList.append(max([zList[-1] - maxCutDepth, stopZ]))

        print('zList: ', zList)

            
        # Get basic rectangular tool path
        rectPathBase = cnc_path.RectPath.fromCenter(cx,cy,width,height,direction,radius=radius)

        for i, cmd in enumerate(rectPathBase.listOfCmds):
            if isinstance(cmd,gcode_cmd.LinearFeed):
                print(i, cmd.motionDict)

        # --------------------------------------------------------------------------------------
        # NOT DONE
        # --------------------------------------------------------------------------------------





        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()

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
                'toolOffset'   : None,
                'direction'    : 'cw',
                'maxCutDepth'  : 0.02,
                
                }
        boundary = RectBoundaryXY(param)

    prog.add(boundary)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')


