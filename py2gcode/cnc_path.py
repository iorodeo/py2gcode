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

PLANE_COORD = { 
        'xy': ('x','y'), 
        'xz': ('x','z'), 
        'yz': ('y','z'), 
        } 

PLANE_NORM_COORD = {'xy': 'z', 'xz': 'y', 'yz': 'x'}


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


class BaseFeedPath(GCodeProg):

    def __init__(self):
        super(BaseFeedPath,self).__init__()

    @property
    def pointList(self):
        return []

    @property
    def endPoint(self):
        return self.pointList[-1]

    @property
    def startPoint(self):
        return self.pointList[0]

    @property 
    def listOfCmds(self):
        return []

    @listOfCmds.setter
    def listOfCmds(self,value):
        pass


class LinearFeedPath(BaseFeedPath):

    """
    Base class for gcode programs consisting of a sequence of LinearFeeds.
    """

    def __init__(self):
        super(LinearFeedPath,self).__init__()

    @property
    def listOfCmds(self):
        listOfCmds = []
        for point in self.pointList: 
            if 'type' in point:
                motionType = point['type']
                del point['type']
                if not motionType in ('linearFeed','rapidMotion'):
                    raise ValueError, 'unknown motion type {0}'.format(motionType)
            else:
                motionType = 'linearFeed'
            if motionType == 'linearFeed':
                listOfCmds.append(LinearFeed(**point))
            else:
                listOfCmds.append(RapidMotion(**point))
        return listOfCmds


class HelicalFeedPath(BaseFeedPath):

    def __init__(self):
        super(HelicalFeedPath,self).__init__()

    @property
    def listOfcmds(self):
        listOfCmds = []
        for poitn in self.pointList:
            pass

class GeneralFeedPath(BaseFeedPath):

    def __init__(self):
        super(GeneralFeedPath,self).__init__()

    @property
    def listOfCmds(self):
        listOfCmds = []
        for point in self.pointList:
            pass

class RectPath(LinearFeedPath):
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

    @property
    def pointList(self):
        x0, y0 = self.point0
        x1, y1 = self.point1 
        pointList = [(x0,y0), (x0,y1), (x1,y1), (x1,y0), (x0,y0)]
        kx, ky = PLANE_COORD[self.plane]
        pointList = [{kx: x, ky: y} for x,y in pointList]
        return pointList


class RectWithCornerCutPath(LinearFeedPath):

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

    @property
    def pointList(self):
        rectPath = RectPath(self.point0,self.point1,plane=self.plane)
        x0, y0 = self.point0
        x1, y1 = self.point1
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)
        kx, ky = PLANE_COORD[self.plane]

        pointList = []
        for i, p in enumerate(rectPath.pointList):
            pointList.append(p)
            if i > 0:
                cornerStr = self.numToCornerStr[i]
                if self.cornerCutDict[cornerStr]:
                    x, y = p[kx], p[ky]
                    normConst = math.sqrt((x-xMid)**2 + (y-yMid)**2)
                    u = (x-xMid)/abs(x-xMid)
                    v = (y-yMid)/abs(y-yMid)
                    xCut = x + self.cornerCutLen*u/math.sqrt(2.0)
                    yCut = y + self.cornerCutLen*v/math.sqrt(2.0)
                    pointList.extend([{kx: xCut, ky: yCut}, {kx: x, ky: y}])
        return pointList


class FilledRectPath(LinearFeedPath): 

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

    @property
    def pointList(self):
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

        pointList = []
        for i in range(self.number):
            p0 = (x0,y0)
            p1 = (x1,y1)
            rectPath = RectPath(p0,p1,plane=self.plane)
            pointList.extend(rectPath.pointList)
            x0 += dx0
            x1 += dx1
            y0 += yLen0
            y1 += yLen1
            if xDoneTest(x0,x1):
                break
            if yDoneTest(y0,y1):
                break
        return pointList


class FilledRectWithCornerCutPath(LinearFeedPath):

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

        checkFilledRectStep(point0,point1,step);
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        self.cutLen = cutLen
        self.cornerCutDict = cornerCutDict
        self.plane = plane

    @property
    def pointList(self):
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
        pointList = cornerCutPath.pointList + filledPath.pointList[5:]
        return pointList


class BiDirRasterRectPath(LinearFeedPath):

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

    @property
    def pointList(self):
        n = getCoordOrder(self.direction,self.plane)
        pointList = getBiDirRasterRectPointList(
                self.point0[::n],
                self.point1[::n],
                self.step,
                keys = PLANE_COORD[self.plane][::n]
                )
        return pointList


class UniDirRasterRectPath(LinearFeedPath):

    def __init__(self,point0,point1,step,cutLevel,retLevel,plane='xy',direction='x'):
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

    @property
    def pointList(self):
        n = getCoordOrder(self.direction,self.plane)
        rasterKeys = PLANE_COORD[self.plane][::n] + (PLANE_NORM_COORD[self.plane],)
        pointList = getUniDirRasterRectPath(
                self.point0[::n],
                self.point1[::n],
                self.step,
                self.cutLevel,
                self.retLevel,
                keys = rasterKeys
                )
        return pointList


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
    return direction in PLANE_COORD[plane] 

def getCoordOrder(direction,plane):
    """
    Returns 1 for coordinate order given by PLANE_COORD[plane] tuple
    and -1 for reverse order.
    """
    if direction == PLANE_COORD[plane][0]:
        return 1
    else:
        return -1

def getBiDirRasterRectPointList(point0,point1,step,keys=('x','y')):
    """
    Generates a bi-directional rastered rectangle  path defined by
    point0=(x0,y0) and point1=(x1,y1). The raster scan is in the direction of
    the 1st coordinate and path starts by initially cutting from x0 to x1.  The
    spacing between rows in is determined by step. 
    """
    x0,y0 = point0
    x1,y1 = point1
    pointList = []
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
    pointList.append({kx: x,ky: y})
    while 1:
        x = getAlternateX(x)
        pointList.append({kx: x,ky: y})
        if rasterDone(y):
            break
        y += dy
        pointList.append({kx: x,ky: y})
    if y != y1:
        y = y1
        pointList.append({kx: x,ky: y})
        x = getAlternateX(x)
        pointList.append({kx: x,ky: y})
    return pointList


def getUniDirRasterRectPath(point0,point1,step,cutZ,retZ,keys=('x','y','z')):
    """
    Generates a uni-directional rastered rectangle path.

    Details: ...
    """
    x0,y0 = point0
    x1,y1 = point1
    pointList = []
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
    pointList.append({kx: x, ky: y, kz: cutZ})  
    while 1:
        x = getAlternateX(x)
        if x==x0:
            pointList.append({kz: retZ})
            pointList.append({kx:x, ky:y, 'type':'rapidMotion'})
        else:
            pointList.append({kz: cutZ})
            if isFirst:
                isFirst = False
            else:
                y += dy
                if outOfBounds(y):
                    y = y1
                    isLast = True
                pointList.append({ky: y})
            pointList.append({kx: x, ky: y})
        if isLast:
            break
    return pointList


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Test program
    prog = GCodeProg()

    prog.add(GenericStart())
    prog.add(Space())

    prog.add(FeedRate(10.0))
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
        prog.add(RectWithCornerCutPath(p,q,cutLen,plane= 'xz'))

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
        prog.add(FilledRectWithCornerCutPath(p,q,step,num,cutLen,plane='xz'))
        prog.add(Space())

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25
        cornerCutDict = {'11':True}

        prog.add(Comment('FilledRectWithCornerCutPath'))
        path = FilledRectWithCornerCutPath(p,q,step,num,cutLen,cornerCutDict=cornerCutDict,plane=xy)
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

    if 1:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='yz',direction='z'))


    prog.add(Space())
    prog.add(End(),comment=True)

    print(prog)
    prog.write('test.ngc')


