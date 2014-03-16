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
import gcode_cmd
import cnc_path
import cnc_routine

class SurfaceBase(cnc_routine.SafeZRoutine):
    """

    Base class for surfacing routines.

    """

    def __init__(self,param):
        super(SurfaceRaster,self).__init__(param)


class SurfaceRasterXY(SurfaceBase):
    """

    Generates a toolpath for xy-plane surfacing using a raster scan.

    """

    def __init__(self,param):
        super(SurfaceBase,sefl).__init__(param)



class SurfaceRasterXZ(SurfaceBase):
    """

    Generates a toolpath for xz-plane surfacing using a raster scan

    """

    def __init__(self,param):
        """
        param dict

        side           = side of part to surface, ('+' or '-')  specified 
                         by sign of normal along y-axis  
        positionY      = position of surface (plane) along x-axis
        mimimumX       = minimum y value to surface
        maximumX       = maximum y value to surface
        startZ         = z value t at which to start cutting 
        depth          = depth of cut in z direction  
        returnDist     = distance from part (in y direction) for non-cutting
                         return passes
        safeZ          = safe tool height 
        toolDiam       = tool diameter
        maxCutDepth    = maximum per pass cutting depth 
        cutDirection   = specifies cutting direction along x-axis, ('+', '-', '+-')
        startDwell     = dwell duration before start (optional)
        """
        super(SurfaceBase,self).__init__(param)

    def makeListOfCmds(self):
        side = self.param['side']
        positionY = float(self.param['positionY'])
        minimumX = float(self.param['minimumX'])
        maximumX = float(self.param['maximumX'])
        startZ = float(self.param['startZ'])
        depth = abs(float(self.param['depth']))
        returnDist = abs(float(self.param['returnDist']))
        toolDiam = abs(float(self.param['toolDiam']))
        maxCutDepth = abs(float(self.param['maxCutDepth']))
        cutDirection = self.param['cutDirection']
        startDwell = self.getStartDwell()

        if minimumX >= maximumX:
            raise ValueError, 'minimumY must be < maximumY'

        # Get y position compensated for side and tool diameter 
        if side == '+':
            yCut = positionY + 0.5*toolDiam
            yRet = positionY + 0.5*toolDiam + returnDist
        elif side == '-':
            yCut = positionY - 0.5*toolDiam 
            yRet = positionY - 0.5*toolDiam - returnDist 
        else:
            raise ValueError, 'unknown side {0}'.format(side)

        # Start and stop x and z points based on cut direction
        if cutDirection == '+':
            x0, x1 = minimumX, maximumX
        elif cutDirection == '-':
            x0, x1 = maximumX, minimumX
        elif cutDirection in ('+-', '-+'):
            raise ValueError, 'bidirectional cutting not supported yet'
        else:
            raise ValueError, 'uknown cutting direction {0}'.format(cutDirection)
        z0 = startZ
        z1 = startZ - depth
        point0 = x0, z0
        point1 = x1, z1

        rasterPath = cnc_path.UniDirRasterRectPath(
                point0,
                point1, 
                maxCutDepth,
                yCut,
                yRet,
                plane='xz',
                direction='x'
                )

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=x0,y=yRet,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Add surface raster
        self.addComment('Raster')
        self.listOfCmds.extend(rasterPath.listOfCmds)
        

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()


class SurfaceRasterYZ(SurfaceBase):
    """

    Generates a toolpath for yz-plane surfacing using a raster scan

    """

    def __init__(self,param):
        """
        param dict

        side           = side of part to surface, ('+' or '-')  specified 
                         by sign of normal along x-axis  
        positionX      = position of surface (plane) along x-axis
        mimimumY       = minimum y value to surface
        maximumY       = maximum y value to surface
        startZ         = z value t at which to start cutting 
        depth          = depth of cut in z direction  
        returnDist     = distance from part (in x direction) for non-cutting
                         return passes
        safeZ          = safe tool height 
        toolDiam       = tool diameter
        maxCutDepth    = maximum per pass cutting depth 
        cutDirection   = specifies cutting direction along y-axis, ('+', '-', '+-')
        startDwell     = dwell duration before start (optional)
        """
        super(SurfaceBase,self).__init__(param)

    def makeListOfCmds(self):
        side = self.param['side']
        positionX = float(self.param['positionX'])
        minimumY = float(self.param['minimumY'])
        maximumY = float(self.param['maximumY'])
        startZ = float(self.param['startZ'])
        depth = abs(float(self.param['depth']))
        returnDist = abs(float(self.param['returnDist']))
        toolDiam = abs(float(self.param['toolDiam']))
        maxCutDepth = abs(float(self.param['maxCutDepth']))
        cutDirection = self.param['cutDirection']
        startDwell = self.getStartDwell()

        if minimumY >= maximumY:
            raise ValueError, 'minimumY must be < maximumY'

        # Get x position compensated for side and tool diameter 
        if side == '+':
            xCut = positionX + 0.5*toolDiam
            xRet = positionX + 0.5*toolDiam + returnDist
        elif side == '-':
            xCut = positionX - 0.5*toolDiam 
            xRet = positionX - 0.5*toolDiam - returnDist 
        else:
            raise ValueError, 'unknown side {0}'.format(side)

        # Start and stop y and z points based on cut direction
        if cutDirection == '+':
            y0, y1 = minimumY, maximumY
        elif cutDirection == '-':
            y0, y1 = maximumY, minimumY
        elif cutDirection in ('+-', '-+'):
            raise ValueError, 'bidirectional cutting not supported yet'
        else:
            raise ValueError, 'uknown cutting direction {0}'.format(cutDirection)
        z0 = startZ
        z1 = startZ - depth
        point0 = y0, z0
        point1 = y1, z1

        rasterPath = cnc_path.UniDirRasterRectPath(
                point0,
                point1, 
                maxCutDepth,
                xCut,
                xRet,
                plane='yz',
                direction='y'
                )

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()
        self.addRapidMoveToSafeZ()
        self.addRapidMoveToPos(x=xRet,y=y0,comment='start x,y')
        self.addDwell(startDwell)
        self.addMoveToStartZ()

        # Add surface raster
        self.addComment('Raster')
        self.listOfCmds.extend(rasterPath.listOfCmds)
        

        # Routine end - move to safe height and post end comment
        self.addRapidMoveToSafeZ()
        self.addEndComment()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Tests
    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(20.0))
    
    if 0:
        param = { 
                'side'            : '-',       
                'positionX'       : 0.0,
                'minimumY'        : -0.5,
                'maximumY'        : 0.5,
                'startZ'          : 0.0,
                'depth'           : 0.25,
                'returnDist'      : 0.1,
                'safeZ'           : 0.2,
                'toolDiam'        : 0.25,
                'maxCutDepth'     : 0.03,
                'cutDirection'    : '-',
                'startDwell'      : 2.0,
                }
        surface = SurfaceRasterYZ(param)

    if 1:
        param = { 
                'side'            : '+',       
                'positionY'       : 0.0,
                'minimumX'        : -0.5,
                'maximumX'        : 0.5,
                'startZ'          : 0.0,
                'depth'           : 0.25,
                'returnDist'      : 0.1,
                'safeZ'           : 0.2,
                'toolDiam'        : 0.25,
                'maxCutDepth'     : 0.03,
                'cutDirection'    : '-',
                'startDwell'      : 2.0,
                }
        surface = SurfaceRasterXZ(param)


    prog.add(surface)
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')

        
