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
import pylab

# Constants
# ----------------------------------------------------------------------------------
PLANE_COORD = {'xy': ('x','y'), 'xz': ('x','z'), 'yz': ('y','z')} 
PLANE_NORM_COORD = {'xy': 'z', 'xz': 'y', 'yz': 'x'}
HELICAL_DIRECTIONS = ('cw', 'ccw')
HELICAL_OFFSETS = {'xy': ('i','j'), 'xz': ('i', 'k'), 'yz': ('j', 'k')}
MINIMUM_NONZERO_RADIUS = 1.0e-8
PLANE_TO_HELIX_MOTION = {
        'xy': gcode_cmd.HelicalMotionXY,
        'xz': gcode_cmd.HelicalMotionXZ, 
        'yz': gcode_cmd.HelicalMotionYZ,
        }

MINIMUM_RADIUS = 1.e-4

# Rectangular paths
# ----------------------------------------------------------------------------------

class RectPath(gcode_cmd.GCodeProg):

    """
    Rectangular path made of LinearFeeds which is defined by points 'point0',
    point1' and the selected plane. Note, prior to rectangle tool is moved from
    current position to start point p via a gcode_cmd.LinearFeed.  There is no move to
    safe height etc.

    Note, point0 is the start and end point of the path.

    """

    def __init__(self, point0, point1, radius=None, plane='xy', helix=None):
        super(RectPath,self).__init__()
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.plane = plane
        self.radius = radius
        if self.radius is not None:
            if self.radius < 0:
                raise ValueError, 'radius must >= 0'
            dx = abs(point1[0] - point0[0])
            dy = abs(point1[1] - point0[1])
            if self.radius > 0.5*min([dx,dy]):
                raise ValueError, 'corner radius is too large'
            if self.radius < MINIMUM_RADIUS:
                self.radius = None
        self.helix = helix
        self.makeListOfCmds()

    @classmethod
    def fromCenter(cls,centerX,centerY,width,height,direction,*args,**kwargs):
        point0, point1 = RectPath.getCornerPointsFromCenter(
                centerX,
                centerY,
                width,
                height,
                direction
                )
        return cls(point0, point1,*args,**kwargs)

    @staticmethod
    def getCornerPointsFromCenter(centerX,centerY,width,height,direction):
        if direction == 'cw':
            point0 = (centerX - 0.5*width, centerY - 0.5*height)
            point1 = (centerX + 0.5*width, centerY + 0.5*height)
        elif direction == 'ccw':
            point0 = (centerX - 0.5*width, centerY + 0.5*height)
            point1 = (centerX + 0.5*width, centerY - 0.5*height)
        else:
            raise ValueError, 'unknown direction {0}'.format(direction)
        return point0, point1

    def getPathPointList2D(self):
        x0, y0 = self.point0
        x1, y1 = self.point1 
        if self.radius is None:
            pointList = [(x0,y0), (x0,y1), (x1,y1), (x1,y0), (x0,y0)]
        else:
            # Get x and y direction signs
            sgnX = 1 if x1 > x0 else -1
            sgnY = 1 if y1 > y0 else -1
            # Get list of points between segments and arcs
            pointList =  [
                    (x0, y0+sgnY*self.radius), 
                    (x0, y1-sgnY*self.radius), 
                    (x0+sgnX*self.radius, y1), 
                    (x1-sgnX*self.radius, y1), 
                    (x1, y1 - sgnY*self.radius), 
                    (x1, y0 + sgnY*self.radius), 
                    (x1 - sgnX*self.radius, y0), 
                    (x0 + sgnX*self.radius, y0), 
                    (x0, y0+sgnY*self.radius),
                    ]
        return pointList

    def getPathPointList3D(self):
        z0, z1 = self.helix[0],self.helix[1]
        pointList2D = self.getPathPointList2D()
        pointPairs2D = zip(pointList2D[:-1],pointList2D[1:])

        # Get  travel distance for points in point list
        distList = [0.0]
        if self.radius is None:
            distList.extend([pointDist2D(*x) for x in pointPairs2D])
        else:
            for i,pair in enumerate(pointPairs2D):
                if i%2==0:
                    distList.append(pointDist2D(*pair))
                else:
                    distList.append(arcDist(self.radius,math.pi/2))

        # Get z depth points
        totalDist = sum(distList)
        zList = []
        distCum = 0.0
        for dist in distList:
            distCum += dist
            z = z0 + (z1-z0)*(distCum/totalDist)
            zList.append(z)

        # Create list of 3D points
        xList, yList = zip(*pointList2D)
        pointList3D = zip(xList,yList,zList)
        return pointList3D
        

    def getPathPointList(self):
        if self.helix is None:
            return self.getPathPointList2D()
        else:
            return self.getPathPointList3D()

    def makeListOfCmds(self):
        kx, ky = PLANE_COORD[self.plane]
        kz = PLANE_NORM_COORD[self.plane]
        pointList = self.getPathPointList()
        if self.radius is None:
            if self.helix is None:
                self.listOfCmds = [gcode_cmd.LinearFeed(**{kx: x, ky: y}) for x,y in pointList]
            else:
                self.listOfCmds = [gcode_cmd.LinearFeed(**{kx: x, ky: y, kz: z}) for x,y,z in pointList]
        else:
            ki, kj = HELICAL_OFFSETS[self.plane]
            cornerArcClass = PLANE_TO_HELIX_MOTION[self.plane]
            arcDir = getDirFromPts(pointList[0][:2],pointList[1][:2],pointList[2][:2])
            self.listOfCmds = []
            for i in range(0,len(pointList)-1):
                # Create linear feed
                x0,y0 = pointList[i][:2]
                linearFeedDict = {kx: x0, ky: y0} 
                if self.helix is not None:
                    z0 = pointList[i][2]
                    linearFeedDict[kz] = z0
                linearFeed = gcode_cmd.LinearFeed(**linearFeedDict)
                self.listOfCmds.append(linearFeed)
                # Create corner arc feed
                if (i-1)%2 == 0:
                    x1, y1 = pointList[i+1][:2]
                    if arcDir == 'ccw':
                        cx = 0.5*( y0 - y1 + x0 + x1)
                        cy = 0.5*( y0 + y1 - x0 + x1)
                    else:
                        cx = 0.5*(-y0 + y1 + x0 + x1)
                        cy = 0.5*( y0 + y1 + x0 - x1)
                    offsetx = cx - x0 
                    offsety = cy - y0
                    cornerArcDict = {ki: offsetx, kj: offsety, kx: x1, ky: y1, 'd': arcDir}
                    if self.helix is not None:
                        z1 = pointList[i+1][2]
                        cornerArcDict[kz] = z1 
                    cornerArcFeed = cornerArcClass(**cornerArcDict)
                    self.listOfCmds.append(cornerArcFeed)


class RectWithCornerCutPath(gcode_cmd.GCodeProg):

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
                    cmd = gcode_cmd.LinearFeed(**{kx: xCut, ky: yCut})
                    self.listOfCmds.append(cmd) 
                    cmd = gcode_cmd.LinearFeed(**{kx: x, ky: y})
                    self.listOfCmds.append(cmd)


class FilledRectPath(gcode_cmd.GCodeProg): 

    """ 
    Filled Rectangular path in xy plane made up of gcode_cmd.LinearFeeds. Path is defined
    by which is defined by point0, point1 and the step size and the number of
    steps to take.
    """

    def __init__(self,point0,point1,step,number, radius=None, plane='xy'):
        super(FilledRectPath,self).__init__()
        checkFilledRectStep(point0,point1,step)
        checkPlane(plane)
        self.point0 = point0
        self.point1 = point1
        self.step = abs(step)
        self.number = number
        self.plane = plane
        self.radius = radius
        self.makeListOfCmds()

    def makeListOfCmds(self):
        x0, y0 = self.point0
        x1, y1 = self.point1
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)
        xLenInit = abs(x1-x0)
        yLenInit = abs(y1-y0)

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
            dy0 =  self.step
            dy1 = -self.step
            def yDoneTest(y0,y1):
                return y0 > y1
        else:
            dy0 = -self.step
            dy1 =  self.step
            def yDoneTest(y0,y1):
                return y0 < y1

        self.listOfCmds = []
        for i in range(self.number):
            p0 = (x0,y0)
            p1 = (x1,y1)
            radius = None
            if self.radius is not None:
                xLen = abs(x1-x0)
                yLen = abs(y1-y0)
                shrinkFactor = min([xLen/xLenInit, yLen/yLenInit])
                if shrinkFactor > 0:
                    radius = self.radius*shrinkFactor
            rectPath = RectPath(p0,p1,radius=radius,plane=self.plane)
            self.listOfCmds.extend(rectPath.listOfCmds)
            x0 += dx0
            x1 += dx1
            y0 += dy0
            y1 += dy1
            if xDoneTest(x0,x1) and yDoneTest(y0,y1):
                break
            if xDoneTest(x0,x1):
                x0, x1 = (0.5*(x0+x1), 0.5*(x0+x1))
                p0 = (x0,y0)
                p1 = (x1,y1)
                rectPath = RectPath(p0,p1,radius=None,plane=self.plane)
                self.listOfCmds.extend(rectPath.listOfCmds)
                break
            if yDoneTest(y0,y1):
                y0, y1 = (0.5*(y0+y1), 0.5*(y0+y1))
                p0 = (x0,y0)
                p1 = (x1,y1)
                rectPath = RectPath(p0,p1,radius=None,plane=self.plane)
                self.listOfCmds.extend(rectPath.listOfCmds)
                break


class FilledRectWithCornerCutPath(gcode_cmd.GCodeProg):

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


class BiDirRasterRectPath(gcode_cmd.GCodeProg):

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

class UniDirRasterRectPath(gcode_cmd.GCodeProg):

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

class CircArcPath(gcode_cmd.GCodeProg):

    def __init__(self, center, radius, ang=(0.0,360.0), plane='xy',direction='cw',helix=None):
        """
        Generates a circular arc path with given center and radius. 
        
        Options:

        ang: Specifies the start at angle ang[0] and end angle ang[1]. The default
        values are (0.0,360.0).

        plane: the plane of the circular arc either 'xy', 'xz' or 'yz'. 

        direction: 'cw' for clockwise, 'ccw' for counter clockwise.

        helix = (startDepth, stopDepth)

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
        self.helix = helix
        if self.helix is not None:
            self.helix = float(helix[0]), float(helix[1])

        self.makeListOfCmds()

    def getStartPoint(self):
        angRad = self.getAngRad()
        cx, cy = self.center
        r = self.radius
        x0 = cx + r*math.cos(angRad[0])
        y0 = cy + r*math.sin(angRad[0])
        if self.helix is not None:
            pt = x0, y0, self.helix[0]
        else:
            pt = x0, y0
        return pt

    def getStopPoint(self):
        angRad = self.getAngRad()
        cx, cy = self.center
        r = self.radius
        x1 = cx + r*math.cos(angRad[1])
        y1 = cy + r*math.sin(angRad[1])
        if self.helix is not None:
            pt = x0, y0, self.helix[1]
        else:
            pt = x0, y0
        return pt

    def getAngRad(self):
        angRad = tuple([math.pi*val/180.0 for val in self.ang])
        if self.direction == 'cw':
            angRad = [-val for val in angRad]
        return angRad

    def getNumTurns(self):
        turns = int(math.floor((self.ang[1] - self.ang[0])/360))
        return turns

    def makeListOfCmds(self):
        kx, ky = PLANE_COORD[self.plane]
        ki, kj = HELICAL_OFFSETS[self.plane]
        kz = PLANE_NORM_COORD[self.plane] 
        cx, cy = self.center
        x0, y0 = self.getStartPoint()[:2]
        x1, y1 = self.getStartPoint()[:2]
        turns = self.getNumTurns()

        # Get helix motion class based on plane
        helixMotionClass = PLANE_TO_HELIX_MOTION[self.plane]

        self.listOfCmds = []

        # Add linear feed to start position
        linearFeedArgs = {kx:x0, ky:y0}
        if self.helix is not None:
            linearFeedArgs[kz] = self.helix[0]
        self.listOfCmds.append(gcode_cmd.LinearFeed(**linearFeedArgs))

        # Add helical motion feed
        helixFeedArgs = {kx:x1, ky:y1, ki:cx-x0, kj:cy-y0, 'd':self.direction}
        if self.helix is not None:
            helixFeedArgs[kz] = self.helix[1]
        if turns > 1:
            helixFeedArgs['p'] = turns
        self.listOfCmds.append(helixMotionClass(**helixFeedArgs))

class CircPath(CircArcPath):

    def __init__(self,center,radius,startAng=0,plane='xy',direction='cw',turns=1,helix=None):
        checkCircPathTurns(turns)
        self.startAng = float(startAng)
        self.turns = int(turns)
        ang = self.startAng, self.startAng + self.turns*360
        super(CircPath,self).__init__(center,radius,ang=ang,plane=plane,direction=direction,helix=helix)

class FilledCircPath(gcode_cmd.GCodeProg):

    def __init__(self,center,radius,step,number,startAng=0,plane='xy',direction='cw',turns=1):
        super(FilledCircPath,self).__init__()
        checkFilledCircStep(radius,step)
        checkPlane(plane)
        checkHelicalDirection(direction)
        checkCircPathTurns(turns)

        self.center = float(center[0]), float(center[1])
        self.radius = float(radius) 
        self.step = step
        self.number = int(number)
        self.startAng = float(startAng)
        self.plane = plane
        self.direction = direction
        self.turns = int(turns)
        self.makeListOfCmds()

    def makeListOfCmds(self):
        self.listOfCmds = []
        for i in range(self.number):
            currRadius = self.radius - i*self.step
            if currRadius <= MINIMUM_NONZERO_RADIUS:
                break
            circPath = CircPath(
                    self.center,
                    currRadius,
                    startAng=self.startAng,
                    plane=self.plane,
                    direction=self.direction,
                    turns=self.turns,
                    )
            self.listOfCmds.extend(circPath.listOfCmds)

# Line and "Line and Arc" segment paths
# ----------------------------------------------------------------------------

class LineSegPath(gcode_cmd.GCodeProg):

    def __init__(self, pointList, closed=False, plane='xy', helix=None):
        checkPlane(plane)
        self.pointList = pointList
        self.pointListDim = self.getPointListDim(pointList) 
        self.closed = closed
        self.plane = plane
        self.helix = helix
        if (self.helix is not None) and self.pointListDim == 3:
            raise ValueError, 'points must be 2d is helix is given'
        self.makeListOfCmds()

    def getPointListDim(self,pointList):
        is2d = True
        is3d = True
        for p in pointList:
            if len(p) != 2:
                is2d = False
            if len(p) != 3:
                is3d = False
        if is2d:
            return 2
        elif is3d:
            return 3
        else:
            raise ValueError, 'dimensions of points must be all either 2 or 3'

    def getStartPoint(self):
        pointListMod = self.getModifiedPointList()
        return pointListMod[0]

    def getStopPoint(self):
        pointListMod = self.getModifiedPointList()
        return pointListMod[-1]

    def addHelixToPointList(self,pointList):
        z0, z1 = self.helix[0], self.helix[1]
        pointPairs2D = zip(pointList[:-1],pointList[1:])
    
        # Get  travel distance for points in point list
        distList = [0.0]
        distList.extend([pointDist2D(*x) for x in pointPairs2D])
    
        # Get z depth points
        totalDist = sum(distList)
        zList = []
        distCum = 0.0
        for dist in distList:
            distCum += dist
            z = z0 + (z1-z0)*(distCum/totalDist)
            zList.append(z)
    
        # Create list of 3D points
        xList, yList = zip(*pointList)
        pointListWithHelix = zip(xList,yList,zList)
        return pointListWithHelix

    def getLinearFeedFromPt(self,p): 
        kx, ky = PLANE_COORD[self.plane]
        if len(p) == 2: 
            feedArgs = {kx: p[0], ky: p[1]}
        else:
            kz = PLANE_NORM_COORD[self.plane]
            feedArgs = {kx: p[0], ky: p[1], kz: p[2]}
        return gcode_cmd.LinearFeed(**feedArgs)

    def getModifiedPointList(self):
        """
        Adds helix and closure to point list
        """
        pointListMod = []
        pointListMod.extend(self.pointList)
        if self.closed:
            pointListMod.append(pointList[0])
        if self.helix is not None:
            pointListMod = self.addHelixToPointList(pointListMod)
        return pointListMod

    def makeListOfCmds(self):
        self.listOfCmds = []
        pointListMod = self.getModifiedPointList()
        for p in pointListMod:
            linearFeed = self.getLinearFeedFromPt(p)
            self.listOfCmds.append(linearFeed)



# Utility functions
# -----------------------------------------------------------------------------

def pointDist2D(p,q): 
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def arcDist(radius, angle):
    return angle*radius

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
    if xLen > 0 and step > xLen:
        raise ValueError, 'step size too large'
    if xLen > 0 and step > yLen:
        raise ValueError, 'step size too large'

def checkFilledCircStep(radius,step):
    """
    Checks that the step size is small enough for filled circular paths.
    """
    if step > 2*radius:
        raise ValueError, 'step size > radius'

def checkCircPathAng(ang):
    """
    Checks validity of cirular path angle argument.
    """
    if not ang[0] < ang[1]: 
        raise ValueError, 'initial angle must be < final angle'

def checkCircPathTurns(turns):
    """
    Checks the validity of the turns argument for circular  paths.
    """
    if int(turns) < 1:
        raise ValueError, 'number of turns must >= 1'

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
    cmdList.append(gcode_cmd.LinearFeed(**{kx: x,ky: y}))
    while 1:
        x = getAlternateX(x)
        cmdList.append(gcode_cmd.LinearFeed(**{kx: x,ky: y}))
        if rasterDone(y):
            break
        y += dy
        cmdList.append(gcode_cmd.LinearFeed(**{kx: x,ky: y}))
    if y != y1:
        y = y1
        cmdList.append(gcode_cmd.LinearFeed(**{kx: x,ky: y}))
        x = getAlternateX(x)
        cmdList.append(gcode_cmd.LinearFeed(**{kx: x,ky: y}))
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
    cmdList.append(gcode_cmd.LinearFeed(**{kx: x, ky: y, kz: cutZ}))  
    while 1:
        x = getAlternateX(x)
        if x==x0:
            cmdList.append(gcode_cmd.LinearFeed(**{kz: retZ}))
            cmdList.append(gcode_cmd.RapidMotion(**{kx:x, ky:y}))
        else:
            cmdList.append(gcode_cmd.LinearFeed(**{kz: cutZ}))
            if isFirst:
                isFirst = False
            else:
                y += dy
                if outOfBounds(y):
                    y = y1
                    isLast = True
                cmdList.append(gcode_cmd.LinearFeed(**{ky: y}))
            cmdList.append(gcode_cmd.LinearFeed(**{kx: x, ky: y}))
        if isLast:
            break
    return cmdList


def getDirFromPts(p0,p1,p2):
    """
    Determines the directin of the path formed by following 
    points p0 -> p1 -> p2.
    """
    x0,y0 = p0
    x1,y1 = p1
    x2,y2 = p2
    ccwTest = (x1-x0)*(y2-y0) > (y1-y0)*(x2-x0)
    if ccwTest:
        return 'ccw'
    else:
        return 'cw'




# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Test program
    prog = gcode_cmd.GCodeProg()

    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(100.0))
    prog.add(gcode_cmd.Space())

    if 0:
        p = 0,0
        q = 1,2
        prog.add(gcode_cmd.Comment('RectPath'))
        prog.add(RectPath(p,q,plane='xy'))

    if 0:
        point0 = 0.0, 0.0
        point1 = 4.0,  1.0
        radius = 0.4
        roundedRectPath = RectPath(point0,point1,radius=radius, plane='xy')
        prog.add(roundedRectPath)

    if 0:
        cx = 0.0
        cy = 0.0
        width = 2.0
        height = 1.0
        direction = 'ccw'
        radius = 0.25 
        rectPath = RectPath.fromCenter(cx,cy,width,height,direction,radius=radius)
        prog.add(rectPath)

    if 0:
        cx = 0.0
        cy = 0.0
        width = 2.0
        height = 1.0
        direction = 'ccw'
        radius = 0.25 
        #radius = None
        helix = (0.0, -0.1)
        #helix = None
        rectPath = RectPath.fromCenter(cx,cy,width,height,direction,radius=radius,helix=helix)
        prog.add(rectPath)
        

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 8 
        prog.add(gcode_cmd.Comment('FilledRectPath'))
        prog.add(FilledRectPath(p,q,step,num))
        prog.add(gcode_cmd.Space())

    if 0:
        p = ( 2.0,  1.0)
        q = (-2.0, -1.0)
        step = 0.1
        num = 11 
        radius = 0.2
        prog.add(gcode_cmd.Comment('FilledRectPath'))
        prog.add(FilledRectPath(p,q,step,num,radius=radius))
        prog.add(gcode_cmd.Space())

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

        prog.add(gcode_cmd.Comment('FilledRectWithCornerCutPath'))
        prog.add(FilledRectWithCornerCutPath(p,q,step,num,cutLen,plane='xy'))
        prog.add(gcode_cmd.Space())

    if 0:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25
        cornerCutDict = {'11':True}

        prog.add(gcode_cmd.Comment('FilledRectWithCornerCutPath'))
        path = FilledRectWithCornerCutPath(p,q,step,num,cutLen,cornerCutDict=cornerCutDict,plane='xy')
        prog.add(path)
        prog.add(gcode_cmd.Space())

    if 0:
        prog.add(QuadraticBSplineXY(1.0,1.0, 1.0, 0.5))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xy',direction='x'))

    if 0:
        p = -5.0, -1.5
        q =  5.0,  1.5
        step = 0.1
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xy',direction='y'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xz',direction='x'))

    if 0:
        p = 3, 1
        q = 0, 0
        step = 0.05
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='xz',direction='z'))

    if 0:
        p = 1.5, 1
        q = 0, 0
        step = 0.05
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='yz',direction='y'))

    if 0:
        p = 1.5, 1
        q = 0, 0
        step = 0.05
        prog.add(gcode_cmd.Comment('BiDirRasterRectPath'))
        prog.add(BiDirRasterRectPath(p,q,step,plane='yz',direction='z'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutZ,retZ,plane='xy',direction='x'))

    if 0:
        p = 0,0
        q = 2,1
        step = 0.05
        cutZ = -0.1 
        retZ =  0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutZ,retZ,plane='xy',direction='y'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='xz',direction='x'))

    if 0:
        p = 0,0
        q = 2,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='xz',direction='z'))

    if 0:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='yz',direction='y'))

    if 0:
        p = 2, 0
        q = 0,-1
        step = 0.05
        cutY = 0.1 
        retY = -0.1
        prog.add(gcode_cmd.Comment('UniDirRasterRectPath'))
        prog.add(UniDirRasterRectPath(p,q,step,cutY,retY,plane='yz',direction='z'))

    if 0:

        center = 0,0
        radius = 1
        ang = 45,2*360+45
        #ang = 0,2*360
        direction = 'ccw'
        plane = 'xy'
        prog.add(gcode_cmd.Comment('CircArcPath'))
        prog.add(CircArcPath(center,radius,ang=ang,direction=direction,plane=plane))

    if 0:

        center = 0,0
        radius = 1
        ang = 0,360
        direction = 'ccw'
        plane = 'xz'
        prog.add(gcode_cmd.Comment('CircArcPath'))
        prog.add(SelectPlaneXZ())
        prog.add(CircArcPath(center,radius,ang=ang,direction=direction,plane=plane))
        prog.add(SelectPlaneXY())

    if 0:

        center = 0,0
        radius = 1
        ang = 0,360
        direction = 'ccw'
        plane = 'xy'
        helix = (0.0, -0.5)
        prog.add(gcode_cmd.Comment('CircArcPath'))
        prog.add(CircArcPath(center,radius,ang=ang,direction=direction,plane=plane,helix=helix))

    if 0:

        center = 1,1
        radius = 1 
        startAng = 90
        direction = 'cw'
        plane = 'xy'
        turns = 2
        prog.add(gcode_cmd.Comment('CircPath'))
        prog.add(CircPath(center,radius,startAng=startAng,plane=plane,direction=direction,turns=turns))

    if 0:

        center = 1,1
        radius = 1 
        step = 0.1
        num = 15
        startAng = -90
        direction = 'ccw'
        plane = 'xy'
        turns = 2
        prog.add(gcode_cmd.Comment('FilledCircPath'))
        filledCircPath = FilledCircPath(
                center,
                radius,
                step,
                num,
                startAng=startAng,
                plane=plane,
                direction=direction,
                turns=turns
                )
        prog.add(filledCircPath)

    if 1:
        pointList = [
                (0,0),
                (2,0),
                (2,1),
                (1,1),
                (1,0.5),
                (0,0.5),
                (-1,1),
                (-2,1),
                (-2,0),
                ]
        closed = True
        plane = 'xy'
        helix = (0,-0.5)
        polyPath = LineSegPath(pointList,closed=closed,plane=plane,helix=helix)
        prog.add(polyPath)



    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)

    print(prog)
    prog.write('test.ngc')





