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
import math
from gcode_cmds import *

# Constants
# ----------------------------------------------------------------------------------
PLANE_COORD = {'xy': ('x','y'), 'xz': ('x','z'), 'yz': ('y','z')} 
PLANE_NORM_COORD = {'xy': 'z', 'xz': 'y', 'yz': 'x'}
HELICAL_DIRECTIONS = ('cw', 'ccw')
HELICAL_OFFSETS = {'xy': ('i','j'), 'xz': ('i', 'k'), 'yz': ('j', 'k')}


# Basic program starts (TODO: move this to separate module)
# ----------------------------------------------------------------------------------

class GenericStart(GCodeProg):

    """
    Simple startup routine ... cancels tool offset, cutter compensation,
    puts system in absolute mode, set units, sets feedrate (optional). 
    """

    def __init__(self,feedrate=None, units='in',coord=1,comment=True):
        super(GenericStart,self).__init__()
        self.add(Space())
        self.add(Comment('Generic Start'))
        self.add(CancelCutterCompensation(),comment=comment)
        self.add(CancelToolLengthOffset(),comment=comment)
        self.add(CancelCannedCycle(),comment=comment)
        self.add(CoordinateSystem(coord),comment=comment)
        self.add(AbsoluteMode(),comment=comment)
        self.add(Units(units),comment=comment)
        self.add(ExactPathMode(),comment=comment)
        if feedrate is not None:
            self.add(FeedRate(feedrate),comment=comment)


# Rectangular paths
# ----------------------------------------------------------------------------------

class RectPath(GCodeProg):
    """
    Rectangular path made of LinearFeeds which is defined by points 'point0',
    point1' and the selected plane. Note, prior to rectangle tool is moved from
    current position to start point p via a LinearFeed.  There is no move to
    safe height etc.

    Note, point0 is the start and end point of the path.
    """

    def __init__(self, point0, point1, plane='xy'):
        super(RectPath,self).__init__()
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.plane = plane
        self.makeListOfCmds()

    def makeListOfCmds(self):
        x0, y0 = self.point0
        x1, y1 = self.point1 
        pointList = [(x0,y0), (x0,y1), (x1,y1), (x1,y0), (x0,y0)]
        kx, ky = PLANE_COORD[self.plane]
        self.listOfCmds = [LinearFeed(**{kx: x, ky: y}) for x,y in pointList]


class RectWithCornerCutPath(GCodeProg):

    """
    Rectangular path in specified plane  with radial corner  cuts.

    Note, point0 is the start and end point of the path.
    """

    defaultCornerCutDict = {'00':True, '01':True, '11':True, '10':True}
    numToCornerStr = {1:'01', 2:'11',3:'10',4:'00'}

    def __init__(
            self, 
            point0,
            point1,
            cornerCutLen, 
            cornerCutDict=defaultCornerCutDict,
            plane='xy'
            ):
        super(RectWithCornerCutPath,self).__init__()
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.cornerCutLen = cornerCutLen
        self.cornerCutDict = cornerCutDict
        self.plane = plane
        for k in self.defaultCornerCutDict:
            if k not in self.cornerCutDict:
                self.cornerCutDict[k] = False
        self.makeListOfCmds()

    def makeListOfCmds(self):
        rectPath = RectPath(self.point0,self.point1,plane=self.plane)
        x0, y0 = self.point0
        x1, y1 = self.point1
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)
        kx, ky = PLANE_COORD[self.plane]

        self.listOfCmds = []
        for i, cmd in enumerate(rectPath.listOfCmds):
            self.listOfCmds.append(cmd)
            if i > 0:
                cornerStr = self.numToCornerStr[i]
                if self.cornerCutDict[cornerStr]:
                    x = cmd.motionDict[kx] 
                    y = cmd.motionDict[ky]
                    normConst = math.sqrt((x-xMid)**2 + (y-yMid)**2)
                    u = (x-xMid)/abs(x-xMid)
                    v = (y-yMid)/abs(y-yMid)
                    xCut = x + self.cornerCutLen*u/math.sqrt(2.0)
                    yCut = y + self.cornerCutLen*v/math.sqrt(2.0)
                    self.listOfCmds.append(LinearFeed(**{kx: xCut, ky: yCut})) 
                    self.listOfCmds.append(LinearFeed(**{kx: x, ky: y}))


class FilledRectPath(GCodeProg): 

    """ 
    Filled Rectangular path in xy plane made up of LinearFeeds. Path is defined
    by which is defined by point0, point1 and the step size and the number of
    steps to take.
    """

    def __init__(self,point0,point1,step,number,plane='xy'):
        super(FilledRectPath,self).__init__()
        checkFilledRectStep(point0,point1,step)
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        self.plane = plane
        self.makeListOfCmds()

    def makeListOfCmds(self):
        x0, y0 = self.point0
        x1, y1 = self.point1
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)

        if x0 < x1:
            dx0 =  self.step
            dx1 = -self.step
            def xDoneTest(x0,x1):
                return x0 > x1
        else:
            dx0 = -self.step
            dx1 =  self.step
            def xDoneTest(x0,x1):
                return x0 < x1
        if y0 < y1:
            yLen0 =  self.step
            yLen1 = -self.step
            def yDoneTest(y0,y1):
                return y0 > y1
        else:
            yLen0 = -self.step
            yLen1 =  self.step
            def yDoneTest(y0,y1):
                return y0 <= y1

        self.listOfCmds = []
        for i in range(self.number):
            p0 = (x0,y0)
            p1 = (x1,y1)
            rectPath = RectPath(p0,p1,plane=self.plane)
            self.listOfCmds.extend(rectPath.listOfCmds)
            x0 += dx0
            x1 += dx1
            y0 += yLen0
            y1 += yLen1
            if xDoneTest(x0,x1):
                break
            if yDoneTest(y0,y1):
                break


class FilledRectWithCornerCutPath(GCodeProg):

    defaultCornerCutDict = RectWithCornerCutPath.defaultCornerCutDict

    def __init__(
            self, 
            point0, 
            point1, 
            step, 
            number, 
            cutLen, 
            cornerCutDict=defaultCornerCutDict, 
            plane='xy'
            ):
        super(FilledRectWithCornerCutPath,self).__init__()
        checkFilledRectStep(point0,point1,step);
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        self.cutLen = cutLen
        self.cornerCutDict = cornerCutDict
        self.plane = plane
        self.makeListOfCmds()

    def makeListOfCmds(self):
        filledPath = FilledRectPath(
                self.point0, 
                self.point1,
                self.step, 
                self.number,
                plane = self.plane,
                )
        cornerCutPath = RectWithCornerCutPath(
                self.point0, 
                self.point1, 
                self.cutLen,
                cornerCutDict = self.cornerCutDict,
                plane = self.plane,
                )
        self.listOfCmds = cornerCutPath.listOfCmds + filledPath.listOfCmds[5:]


class BiDirRasterRectPath(GCodeProg):

    """
    Generates a bi-direction rastered rectangle path. The rectangle is
    specified via the adjacent corners (point0 and point1), the step between
    raster rows, the plane of the raster and the direction of the raster.
    """

    def __init__(self,point0,point1,step,plane='xy',direction='x'):
        super(BiDirRasterRectPath,self).__init__()
        checkFilledRectStep(point0,point1,step)
        checkPlane(plane)
        checkDirection(direction,plane)
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.plane = plane
        self.direction = direction
        self.makeListOfCmds()

    def makeListOfCmds(self):
        n = getCoordOrder(self.direction,self.plane)
        self.listOfCmds = getBiDirRasterRect(
                self.point0[::n],
                self.point1[::n],
                self.step,
                keys = PLANE_COORD[self.plane][::n]
                )

class UniDirRasterRectPath(GCodeProg):

    def __init__(self,point0,point1,step,cutLevel,retLevel,plane='xy',direction='x'):
        super(UniDirRasterRectPath,self).__init__()
        checkFilledRectStep(point0,point1,step)
        checkPlane(plane)
        checkDirection(direction,plane)
        self.point0 = point0
        self.point1 = point1
        self.step = step
        self.cutLevel = cutLevel
        self.retLevel = retLevel
        self.plane = plane
        self.direction = direction
        self.makeListOfCmds()
        
    def makeListOfCmds(self):
        n = getCoordOrder(self.direction,self.plane)
        rasterKeys = PLANE_COORD[self.plane][::n] + (PLANE_NORM_COORD[self.plane],)
        self.listOfCmds = getUniDirRasterRect(
                self.point0[::n],
                self.point1[::n],
                self.step,
                self.cutLevel,
                self.retLevel,
                keys = rasterKeys
                )


# Circular/Helical paths
# -----------------------------------------------------------------------------



class CircArcPath(GCodeProg):

    def __init__(self, center, radius, ang=(0.0,360.0), plane='xy',direction='cw'):
        """
        Generates a circular arc path with given center and radius. 
        
        Options:

        ang: Specifies the start at angle ang[0] and end angle ang[1]. The default
        values are (0.0,360.0).

        plane: the plane of the circular arc either 'xy', 'xz' or 'yz'. 

        direction: 'cw' for clockwise, 'ccw' for counter clockwise.

        """
        super(CircArcPath,self).__init__()
        checkCircPathAng(ang)
        checkPlane(plane)
        checkHelicalDirection(direction)

        self.center = float(center[0]), float(center[1]) 
        self.radius = float(radius) 
        self.ang = float(ang[0]), float(ang[1]) 
        self.plane = plane
        self.direction = direction
        self.makeListOfCmds()

    def makeListOfCmds(self):
        kx, ky = PLANE_COORD[self.plane]
        ki, kj = HELICAL_OFFSETS[self.plane]
        cx, cy = self.center
        r = self.radius
        angRad = tuple([math.pi*val/180.0 for val in self.ang])
        if self.direction == 'cw':
            angRad = [-val for val in angRad]

        # Number of turns and start and end positions
        turns = int(math.floor((self.ang[1] - self.ang[0])/360))
        x0 = cx + r*math.cos(angRad[0])
        y0 = cy + r*math.sin(angRad[0])
        x1 = cx + r*math.cos(angRad[1])
        y1 = cy + r*math.sin(angRad[1])

        if self.plane == 'xy':
            helixMotionClass = HelicalMotionXY
        elif plane == 'xz':
            helixMotionClass = HelicalMotionXZ
        elif plane == 'yz':
            helixMotionClass = HelicalMotionYZ
        else:
            raise ValueError, 'uknown plane {0}'.format(plane)

        self.listOfCmds = []
        # Add linear feed to start position
        cmdArgs = {kx:x0, ky:y0}
        self.listOfCmds.append(LinearFeed(**cmdArgs))

        # Add helical feed
        cmdArgs = {kx:x1, ky:y1, ki:cx-x0, kj:cy-y0, 'd':self.direction}
        if turns > 1:
            cmdArgs['p'] = turns
        self.listOfCmds.append(helixMotionClass(**cmdArgs))

class CircPath(CircArcPath):

    def __init__(self,center,radius,startAng=0,turns=1,plane='xy',direction='cw'):
        self.startAng = float(startAng)
        self.turns = int(turns)
        if self.turns < 1:
            raise ValueError, 'number of turns must >= 1'
        ang = self.startAng, self.startAng + self.turns*360
        super(CircPath,self).__init__(center,radius,ang=ang,plane=plane,direction=direction)

class FilledCircPath(GCodeProg):

    def __init__(self,center,radius,step,number,startAng=0,plane='xy',direction='cw'):
        super(FilledCircPath,self).__init__()
        checkFilledCircStep(radius,step)
        checkPlane(plane)
        checkHelicalDirection(direction)

        self.center = float(center[0]), float(center[1])
        self.radius = float(radius) 
        self.step = step
        self.number = int(number)
        self.startAng = float(startAng)

        self.plane = plane
        self.direction = direction
        self.makeListOfCmds()

    def makeListOfCmds(self):
        self.listOfCmds = []
        for i in range(self.number):
            currRadius = self.radius - i*self.step
            if currRadius <= 0:
                break
            circPath = CircPath(
                    self.center,
                    currRadius,
                    startAng=self.startAng,
                    plane=self.plane,
                    direction=self.direction
                    )
            self.listOfCmds.extend(circPath.listOfCmds)


# Utility functions
# -----------------------------------------------------------------------------

def swapKeys(d,keySwapDict):
    """
    Swap keys in dictionary according to keySwap dictionary 
    """
    dNew = {}
    for key, keyNew in keySwapDict.iteritems():
        if key in d:
            dNew[keyNew] = d[key]
    for key in d:
        if key not in keySwapDict:
            dNew[key] = d[key]
    return dNew

def checkFilledRectStep(point0,point1,step): 
    """
    Checks that step size is small enough for filled rectangular paths. Note,
    this include raster paths.
    """
    x0,y0 = point0
    x1,y1 = point1
    xLen = abs(x1-x0)
    yLen = abs(y1-y0)
    if step > xLen or step > yLen:
        raise ValueError, 'step size too large'

def checkFilledCircStep(radius,step):
    """
    Checks that the step size is small enough for filled circular paths.
    """
    if step > radius:
        raise ValueError, 'step size > radius'

def checkCircPathAng(ang):
    """
    Checks validity of cirular path angle argument.
    """
    if not ang[0] < ang[1]: 
        raise ValueError, 'initial angle must be < final angle'

def checkPlane(plane): 
    """
    check that plane is in list of allowed planes
    """
    if not plane in PLANE_COORD: 
        raise ValueError, 'unknown plane {0}'.format(plane)

def checkDirection(direction,plane):
    """
    Check that the direction is allowed given the plane
    """
    if direction not in PLANE_COORD[plane]:
        raise ValueError, 'uknown direction {0} in plane {1}'.format(direction,plane)


def getCoordOrder(direction,plane):
    """
    Returns 1 for coordinate order given by PLANE_COORD[plane] tuple
    and -1 for reverse order.
    """
    if direction == PLANE_COORD[plane][0]:
        return 1
    else:
        return -1

def checkHelicalDirection(direction):
    """
    Check that helical direction is allowed
    """
    if direction not in HELICAL_DIRECTIONS:
            raise ValueError, 'uknown helical direction {0}'.format(direction)

def getBiDirRasterRect(point0,point1,step,keys=('x','y')):
    """
    Generates a bi-directional rastered rectangle  path defined by
    point0=(x0,y0) and point1=(x1,y1). The raster scan is in the direction of
    the 1st coordinate and path starts by initially cutting from x0 to x1.  The
    spacing between rows in is determined by step. 
    """
    x0,y0 = point0
    x1,y1 = point1
    cmdList = []
    # Get step size and raster completion function
    if y0 < y1:
        dy = step
        def rasterDone(y):
            return (y+dy) > y1 
    else:
        dy = -step
        def rasterDone(y):
            return (y+dy) < y1

    # Get function for alternating x direction
    def getAlternateX(xLast):
        if xLast == x0:
            return x1
        else:
            return x0

    # Generate raster
    x, y = x0, y0
    kx, ky = keys
    cmdList.append(LinearFeed(**{kx: x,ky: y}))
    while 1:
        x = getAlternateX(x)
        cmdList.append(LinearFeed(**{kx: x,ky: y}))
        if rasterDone(y):
            break
        y += dy
        cmdList.append(LinearFeed(**{kx: x,ky: y}))
    if y != y1:
        y = y1
        cmdList.append(LinearFeed(**{kx: x,ky: y}))
        x = getAlternateX(x)
        cmdList.append(LinearFeed(**{kx: x,ky: y}))
    return cmdList


def getUniDirRasterRect(point0,point1,step,cutZ,retZ,keys=('x','y','z')):
    """
    Generates a uni-directional rastered rectangle path.

    Details: ...
    """
    x0,y0 = point0
    x1,y1 = point1
    cmdList = []
    # Get step size and raster completion function
    if y0 < y1:
        dy = step
        def outOfBounds(y):
            return y > y1
    else:
        dy = -step
        def outOfBounds(y):
            return y < y1

    # Get function for alternating x direction
    def getAlternateX(xLast):
        if xLast == x0:
            return x1
        else:
            return x0

    # Generate raster
    x, y = x0, y0
    kx, ky, kz = keys
    isFirst = True
    isLast = False
    cmdList.append(LinearFeed(**{kx: x, ky: y, kz: cutZ}))  
    while 1:
        x = getAlternateX(x)
        if x==x0:
            cmdList.append(LinearFeed(**{kz: retZ}))
            cmdList.append(RapidMotion(**{kx:x, ky:y}))
        else:
            cmdList.append(LinearFeed(**{kz: cutZ}))
            if isFirst:
                isFirst = False
            else:
                y += dy
                if outOfBounds(y):
                    y = y1
                    isLast = True
                cmdList.append(LinearFeed(**{ky: y}))
            cmdList.append(LinearFeed(**{kx: x, ky: y}))
        if isLast:
            break
    return cmdList


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Test program
    prog = GCodeProg()

    prog.add(GenericStart())
    prog.add(Space())
    prog.add(FeedRate(100.0))
    prog.add(Space())

    if 0:
        p = 0,0
        q = 1,2
        prog.add(Comment('RectPath'))
        prog.add(RectPath(p,q,plane='xy'))

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 8 
        prog.add(Comment('FilledRectPath'))
        prog.add(FilledRectPath(p,q,step,num))
        prog.add(Space())

    if 0:
        p = (0,0)
        q = (4,5)
        cutLen = 0.5
        prog.add(RectWithCornerCutPath(p,q,cutLen,plane= 'xy'))

    if 0:
        p = (0,0)
        q = (4,5)
        cutLen = 0.5
        cornerCutDict = {'00':True,'11':True}
        prog.add(RectWithCornerCutPath(p,q,cutLen,cornerCutDict=cornerCutDict))

    if 0: 
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25

        prog.add(Comment('FilledRectWithCornerCutPath'))
        prog.add(FilledRectWithCornerCutPath(p,q,step,num,cutLen,plane='xy'))
        prog.add(Space())

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25
        cornerCutDict = {'11':True}

        prog.add(Comment('FilledRectWithCornerCutPath'))
        path = FilledRectWithCornerCutPath(p,q,step,num,cutLen,cornerCutDict=cornerCutDict,plane='xy')
        prog.add(path)
        prog.add(Space())

    if 0:
        prog.add(QuadraticBSplineXY(1.0,1.0, 1.0, 0.5))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xy',direction='x'))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xy',direction='y'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xz',direction='x'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xz',direction='z'))

    if 0:
        p = 1.5, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='yz',direction='y'))

    if 0:
        p = 1.5, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='yz',direction='z'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutZ,retZ,plane='xy',direction='x'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutZ,retZ,plane='xy',direction='y'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='xz',direction='x'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='xz',direction='z'))

    if 0:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='yz',direction='y'))

    if 0:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='yz',direction='z'))

    if 0:

        center = 0,0
        radius = 1
        ang = 45,2*360+45
        #ang = 0,2*360
        direction = 'ccw'
        plane = 'xy'
        prog.add(Comment('CircArcPath'))
        prog.add(CircArcPath(center,radius,ang=ang,direction=direction,plane=plane))

    if 0:

        center = 0,0
        radius = 1
        ang = 0,360
        direction = 'ccw'
        plane = 'xz'
        prog.add(Comment('CircArcPath'))
        prog.add(SelectPlaneXZ())
        prog.add(CircArcPath(center,radius,ang=ang,direction=direction,plane=plane))
        prog.add(SelectPlaneXY())

    if 0:

        center = 1,1
        radius = 1 
        startAng = 90
        direction = 'cw'
        plane = 'xy'
        prog.add(Comment('CircPath'))
        prog.add(CircPath(center,radius,startAng=startAng,plane=plane,direction=direction))

    if 1:

        center = 1,1
        radius = 1 
        step = 0.1
        num = 15
        startAng = -90
        direction = 'ccw'
        plane = 'xy'
        prog.add(Comment('FilledCircPath'))
        filledCircPath = FilledCircPath(center,radius,step,num,startAng=startAng,plane=plane,direction=direction)
        prog.add(filledCircPath)



    prog.add(Space())
    prog.add(End(),comment=True)

    print(prog)
    prog.write('test.ngc')


