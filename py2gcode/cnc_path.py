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


class LinearFeedPath(GCodeProg):

    """
    Base class for gcode programs consisting of a sequence of LinearFeeds.
    """

    def __init__(self):
        super(LinearFeedPath,self).__init__()

    @property
    def pointList(self):
        return []

    @property
    def endPoint(self):
        return pointList[-1]

    @property
    def startPoint(self):
        return pointList[0]

    @property
    def listOfCmds(self):
        listOfCmds = []
        for point in self.pointList:
            if isinstance(point,dict):
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
            else:
                listOfCmds.append(LinearFeed(*point))
        return listOfCmds

    @listOfCmds.setter
    def listOfCmds(self,value):
        pass


class RectPathXY(LinearFeedPath):
    """
    Rectangular path  in xy plane made of LinearFeeds which is defined by
    points 'point0' and 'point1'. Note, prior to rectangle tool is moved from
    current position to start point p via a LinearFeed.  There is no move to
    safe height etc.

    Note, point0 is the start and end point of the path.
    """

    def __init__(self, point0, point1):
        super(RectPathXY,self).__init__()
        self.point0 = point0
        self.point1 = point1

    @property
    def pointList(self):
        x0, y0 = self.point0
        x1, y1 = self.point1 
        pointList = [(x0,y0), (x0,y1), (x1,y1), (x1,y0), (x0,y0)]
        return pointList


class RectWithCornerCutPathXY(LinearFeedPath):

    """
    Rectangular path in xy plane with radial corner  cuts.

    Note, point0 is the start and end point of the path.
    """

    def __init__(self,point0,point1,cornerCutLen):
        self.point0 = point0
        self.point1 = point1
        self.cornerCutLen = cornerCutLen

    @property
    def pointList(self):
        rectPath = RectPathXY(self.point0,self.point1)
        x0, y0 = self.point0
        x1, y1 = self.point1
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)

        pointList = []
        for i, p in enumerate(rectPath.pointList):
            pointList.append(p)
            if i > 0:
                x,y = p
                normConst = math.sqrt((x-xMid)**2 + (y-yMid)**2)
                u = (x-xMid)/normConst
                v = (y-yMid)/normConst
                xx = x + self.cornerCutLen*u/math.sqrt(2.0)
                yy = y + self.cornerCutLen*v/math.sqrt(2.0)
                pointList.extend([(xx,yy),(x,y)])
        return pointList


class FilledRectPathXY(LinearFeedPath): 

    """ 
    Filled Rectangular path in xy plane made up of LinearFeeds. Path is defined
    by which is defined by point0, point1 and the step size and the number of
    steps to take.
    """

    def __init__(self,point0,point1,step,number):
        super(FilledRectPathXY,self).__init__()
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        checkFilledRectStep(self.point0,self.point1,self.step)

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
            rectPath = RectPathXY(p0,p1)
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


class FilledRectWithCornerCutPathXY(LinearFeedPath):

    def __init__(self,point0,point1,step,number,cutLen):
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        self.cutLen = cutLen

    @property
    def pointList(self):
        filledPath = FilledRectPathXY(
                self.point0, 
                self.point1,
                self.step, 
                self.number
                )
        cornerCutPath = RectWithCornerCutPathXY(
                self.point0, 
                self.point1, 
                self.cutLen
                )
        pointList = cornerCutPath.pointList + filledPath.pointList[5:]
        return pointList


class BiDirRasterRectPathBase(LinearFeedPath):

    """ 
    Base class bi-directional for rastered rectangle paths.  
    """

    allowedDirections = ()

    def __init__(self,point0,point1,step,direction=None):
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        if direction in self.allowedDirections:
            self.direction = direction
        else:
            raise ValueError, 'uknown direction {0}'.format(direction)
        checkFilledRectStep(self.point0,self.point1,self.step)

    @property
    def pointList(self):
        return []


class BiDirRasterRectPathXY(BiDirRasterRectPathBase):

    """
    Bi-directional rastered rectangle  path in xy plane.
    """

    allowedDirections = 'x','y'

    def __init__(self,point0,point1,step,direction='x'):
        super(BiDirRasterRectPathXY,self).__init__(point0,point1,step,direction=direction)

    @property
    def pointList(self):
        if self.direction == 'x':
            pointList = getBiDirRasterRectPointList(self.point0,self.point1,self.step)
        else:
            x0, y0 = self.point0
            x1, y1 = self.point1
            pointList =  getBiDirRasterRectPointList((y0,x0),(y1,x1),self.step)
            pointList = [(x,y) for y,x in pointList]
        return pointList


class BiDirRasterRectPathXZ(BiDirRasterRectPathBase):

    """
    Bi-directional rastered rectangle raster path in xz plane.
    """

    allowedDirections = 'x', 'z'

    def __init__(self,point0,point1,step,direction='x'):
        super(BiDirRasterRectPathXZ,self).__init__(point0,point1,step,direction=direction)

    @property
    def pointList(self):
        if self.direction == 'x':
            pointList = getBiDirRasterRectPointList(self.point0,self.point1,self.step)
            pointList = [(x,0,z) for x,z in pointList]
        else:
            x0, z0 = self.point0
            x1, z1 = self.point1
            pointList = getBiDirRasterRectPointList((z0,x0),(z1,x1),self.step)
            pointList = [(x,0,z) for z,x in pointList]
        return pointList

class BiDirRasterRectPathYZ(BiDirRasterRectPathBase):

    """
    Bi-directional rastered rectangle path in yz plane.
    """

    allowedDirections = 'y', 'z'

    def __init__(self,point0,point1,step,direction='y'):
        super(BiDirRasterRectPathYZ,self).__init__(point0,point1,step,direction=direction)

    @property
    def pointList(self):
        if self.direction == 'y':
            pointList = getBiDirRasterRectPointList(self.point0,self.point1,self.step)
            pointList = [(0,y,z) for y,z in pointList]
        else:
            y0, z0 = self.point0
            y1, z1 = self.point1
            pointList = getBiDirRasterRectPointList((z0,y0),(z1,y1),self.step)
            pointList = [(0,y,z) for z,y in pointList]
        return pointList


class UniDirRasterRectPathBase(LinearFeedPath):

    allowedDirections = ()

    def __init__(self,point0,point1,step,direction=None):
        super(UniDirRasterRectPathBase,self).__init__()
        self.point0 = point0
        self.point1 = point1
        self.step = step
        if direction in self.allowedDirections:
            self.direction = direction
        else:
            raise ValueError, 'uknown direction {0}'.format(direction)
        checkFilledRectStep(self.point0,self.point1,self.step)

    @property
    def pointList(self):
        return []


class UniDirRasterRectPathXY(UniDirRasterRectPathBase):

    allowedDirections = ('x', 'y')

    def __init__(self,point0,point1,step,cutZ,retZ,direction='x'):
        super(UniDirRasterRectPathXY,self).__init__(point1,point1,step,direction=direction)
        self.cutZ = cutZ
        self.retZ = retZ

    @property
    def pointList(self):
        if self.direction == 'x':
            pointList = getUniDirRasterRectPath(
                    self.point0,
                    self.point1,
                    self.step,
                    self.cutZ,
                    self.retZ
                    )
        else:
            x0, y0 = self.point0
            x1, y1 = self.point1
            pointList = getUniDirRasterRectPath(
                    (y0,x0),
                    (y1,x1),
                    self.step,
                    self.cutZ,
                    self.retZ,
                    )
            keySwapDict = {'x': 'y', 'y': 'x'}
            pointList = [swapKeys(point,keySwapDict) for point in pointList]
        return pointList


class UniDirRasterRectPathXZ(UniDirRasterRectPathBase):

    allowedDirections = ('x', 'z')

    def __init__(self,point0,point1,step,cutY,retY,direction='x'):
        super(UniDirRasterRectPathXZ,self).__init__(point0,point1,step,direction=direction)
        self.cutY = cutY
        self.retY = retY

    @property
    def pointList(self):
        if self.direction == 'x':
            pointList = getUniDirRasterRectPath(
                    self.point0,
                    self.point1,
                    self.step,
                    self.cutY,
                    self.retY
                    )
            keySwapDict = {'y': 'z', 'z': 'y'}
            pointList = [swapKeys(point,keySwapDict) for point in pointList]
        else:
            x0, y0 = self.point0
            x1, y1 = self.point1
            pointList = getUniDirRasterRectPath(
                    (y0,x0),
                    (y1,x1),
                    self.step,
                    self.cutY,
                    self.retY,
                    )
            keySwapDict = {'x': 'z', 'y': 'x', 'z': 'y'}
            pointList = [swapKeys(point,keySwapDict) for point in pointList]
        return pointList


class UniDirRasterRectPathYZ(UniDirRasterRectPathBase):

    allowedDirections = ('y','z')

    def __init__(self,point0,point1,step,cutX,retX,direction='y'):
        super(UniDirRasterRectPathYZ,self).__init__(point0,point1,step,direction=direction)
        self.cutX = cutX
        self.retX = retX

    @property
    def pointList(self):
        if self.direction == 'y':
            pointList = getUniDirRasterRectPath(
                    self.point0,
                    self.point1,
                    self.step,
                    self.cutX,
                    self.retX
                    )
            keySwapDict = {'x': 'y', 'z': 'x', 'y': 'z'}
            pointList = [swapKeys(point,keySwapDict) for point in pointList]
        else:
            print('a')
            x0, y0 = self.point0
            x1, y1 = self.point1
            pointList = getUniDirRasterRectPath(
                    (y0,x0),
                    (y1,x1),
                    self.step,
                    self.cutX,
                    self.retX,
                    )
            keySwapDict = {'x': 'z', 'z': 'x'}
            pointList = [swapKeys(point,keySwapDict) for point in pointList]
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
        raise ValueError, 'step size too small'


def getBiDirRasterRectPointList(point0,point1,step):
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
    x,y = x0,y0
    pointList.append((x,y))
    while 1:
        x = getAlternateX(x)
        pointList.append((x,y))
        if rasterDone(y):
            break
        y += dy
        pointList.append((x,y))
    if y != y1:
        y = y1
        pointList.append((x,y))
        x = getAlternateX(x)
        pointList.append((x,y))
    return pointList


def getUniDirRasterRectPath(point0,point1,step,cutZ,retZ):
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
    isFirst = True
    isLast = False
    pointList.append({'x': x, 'y': y, 'z': cutZ})  
    while 1:
        x = getAlternateX(x)
        if x==x0:
            pointList.append({'z': retZ})
            pointList.append({'x':x, 'y':y, 'type':'rapidMotion'})
        else:
            pointList.append({'z': cutZ})
            if isFirst:
                isFirst = False
            else:
                y += dy
                if outOfBounds(y):
                    y = y1
                    isLast = True
                pointList.append({'y': y})
            pointList.append({'x': x, 'y': y})
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
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 8 
        prog.add(Comment('FilledRectPathXY'))
        prog.add(FilledRectPathXY(p,q,step,num))
        prog.add(Space())

    if 0:
        p = (0,0)
        q = (4,5)
        cutLen = 0.5
        prog.add(RectWithCornerCutPathXY(p,q,cutLen))

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25

        prog.add(Comment('FilledRectWithCornerCutPathXY'))
        prog.add(FilledRectWithCornerCutPathXY(p,q,step,num,cutLen))
        prog.add(Space())

    if 0:
        prog.add(QuadraticBSplineXY(1.0,1.0, 1.0, 0.5))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(Comment('BiDirRasterRectPathXY'))
        prog.add(BiDirRasterRectPathXY(p,q,step,direction='x'))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(Comment('BiDirRasterRectPathXY'))
        prog.add(BiDirRasterRectPathXY(p,q,step,direction='x'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPathXZ'))
        prog.add(BiDirRasterRectPathXZ(p,q,step,direction='x'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPathXZ'))
        prog.add(BiDirRasterRectPathXZ(p,q,step,direction='z'))

    if 0:
        p = 1.5, 1
        q = 0, 0
        step = 0.05
        prog.add(Comment('BiDirRasterRectPathXZ'))
        prog.add(BiDirRasterRectPathYZ(p,q,step,direction='y'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(Comment('UniDirRasterRectPathXY'))
        prog.add(UniDirRasterRectPathXY(p,q,step,cutZ,retZ,direction='x'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(Comment('UniDirRasterRectPathXY'))
        prog.add(UniDirRasterRectPathXY(p,q,step,cutZ,retZ,direction='y'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPathXZ'))
        prog.add(UniDirRasterRectPathXZ(p,q,step,cutY,retY,direction='x'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPathXZ'))
        prog.add(UniDirRasterRectPathXZ(p,q,step,cutY,retY,direction='z'))

    if 0:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPathYZ'))
        prog.add(UniDirRasterRectPathYZ(p,q,step,cutY,retY,direction='y'))

    if 1:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(Comment('UniDirRasterRectPathYZ'))
        prog.add(UniDirRasterRectPathYZ(p,q,step,cutY,retY,direction='z'))


    prog.add(Space())
    prog.add(End(),comment=True)

    print(prog)
    prog.write('test.ngc')


