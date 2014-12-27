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
import operator
import math
import numpy
import gcode_cmd
import cnc_path

try:
    import matplotlib.pyplot as plt
    havePlt = True
except ImportError:
    havePlt = False



# 3D Segments
# -----------------------------------------------------------------------------

class LineSeg3D(object):
    pass


class ArcSeg3D(object):
    pass


# 2D Segments  
# -----------------------------------------------------------------------------

class LineSeg2D(object):
    """
    Simple 2D line segment
    """

    def __init__(self,startPoint,endPoint): 
        self.dim = 2
        self.startPoint = float(startPoint[0]), float(startPoint[1])
        self.endPoint = float(endPoint[0]), float(endPoint[1])

    @property
    def length(self):
        return dist2D(self.startPoint, self.endPoint)

    @property
    def midPoint(self):
        return midPoint2D(self.startPoint, self.endPoint)

    def getFeedCmd(self,plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.endPoint[0], ky: self.endPoint[1]}
        return gcode_cmd.LinearFeed(**feedArgs)

    def getFeedToStartCmd(self, plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.startPoint[0], ky: self.startPoint[1]}
        return gcode_cmd.LinearFeed(**feedArgs)

    def getRapidCmd(self,plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.endPoint[0], ky: self.endPoint[1]}
        return gcode_cmd.RapidMotion(feedArgs)

    def getRapidToStartCmd(self,plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.startPoint[0], ky: self.startPoint[1]}
        return gcode_cmd.RapidMotion(**feedArgs)

    def plot(self,color=None):
        if havePlt:
            x0,y0 = self.startPoint
            x1,y1 = self.endPoint
            if color is None:
                color = 'b'
            plt.plot([x0,x1],[y0,y1],color=color)

    def divideEqual(self,num):
        """
        Divide line segment into 'num' equal length segments 
        """
        # --------------------------------------------------
        # TODO  - not done
        # --------------------------------------------------
        segList = []
        return segList


    def reverse(self):
        return  LineSeg2D(self.endPoint, self.startPoint)



class ArcSeg2D(object):
    """
    Simple 2D arc segment.
    """
    
    def __init__(self,center,radius,startAngle,endAngle,direction):
        self.dim = 2
        self.center = float(center[0]), float(center[1])
        self.radius = float(radius)
        self.startAngle = float(startAngle)
        self.endAngle = float(endAngle)
        self.direction = direction
        self.checkAngles()
        self.checkDirection()

    def checkAngles(self):
        self.startAngle = self.startAngle%(2.0*math.pi)
        self.endAngle = self.endAngle%(2.0*math.pi)

    def checkDirection(self):
        if self.direction not in ('cw', 'ccw'):
            raise ValueError, 'unknown direction {0}'.format(self.direction)

    @property
    def endAngleAdj(self):
        if self.endAngle < self.startAngle:
            endAngleAdj = self.endAngle + 2.0*math.pi 
        else:
            endAngleAdj = self.endAngle
        return endAngleAdj

    @property
    def startAngleDeg(self):
        return math.degrees(self.startAngle)

    @property
    def endAngleDeg(self):
        return math.degrees(self.endAngle)

    @property
    def endAngleAdjDeg(self):
        return math.degrees(self.endAngleAdj)

    @property
    def startPoint(self):
        x = self.center[0] + self.radius*math.cos(self.startAngle)
        y = self.center[1] + self.radius*math.sin(self.startAngle)
        return x,y

    @property
    def endPoint(self):
        x = self.center[0] + self.radius*math.cos(self.endAngleAdj)
        y = self.center[1] + self.radius*math.sin(self.endAngleAdj)
        return x,y

    @property
    def length(self):
        if self.direction == 'ccw':
            totalAngle = self.endAngleAdj - self.startAngle
        else: 
            totalAngle = 2.0*math.pi - (self.endAngleAdj - self.startAngle)
        return totalAngle*self.radius

    def getFeedCmd(self,plane='xy'):
        cmd = cnc_path.CircArcPath(
                self.center, 
                self.radius, 
                ang = (self.startAngleDeg, self.endAngleDeg),
                plane=plane,
                direction = self.direction,
                helix = None,
                includeFeedToStart = False,
                )
        return cmd

    def getFeedToStartCmd(self,plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.startPoint[0], ky: self.startPoint[1]}
        return gcode_cmd.LinearFeed(**feedArgs)

    def getRapidToStartCmd(self,plane='xy'):
        kx, ky = tuple(plane)
        feedArgs = {kx: self.startPoint[0], ky: self.startPoint[1]}
        return gcode_cmd.RapidMotion(**feedArgs)

    def convertToLineSegList(self, maxArcLen=1.0e-5):
        totalAngle = abs(self.endAngleAdj - self.startAngle)
        maxStepAngle = maxArcLen/self.radius
        numPts = int(math.ceil(totalAngle/maxStepAngle))

        if self.direction == 'ccw':
            stepAngleArray = numpy.linspace(self.startAngle, self.endAngleAdj, numPts)
        else:
            stepAngleArray = numpy.linspace(self.startAngle+2.0*math.pi, self.endAngleAdj, numPts)

        lineSegList = []
        for ang0, ang1 in zip(stepAngleArray[:-1], stepAngleArray[1:]):
            x0 = self.center[0] + self.radius*math.cos(ang0)
            y0 = self.center[1] + self.radius*math.sin(ang0)
            x1 = self.center[0] + self.radius*math.cos(ang1)
            y1 = self.center[1] + self.radius*math.sin(ang1)
            lineSeg = LineSeg2D((x0,y0), (x1,y1))
            lineSegList.append(lineSeg)
        return lineSegList

    def divideEqual(self,num):
        """
        Divide into 'num' equal length sub-arcs
        """
        # -------------------------------------------
        # TODO - not done
        # -------------------------------------------
        segList = []
        return segList

    def reverse(self):
        return ArcSeg2D(self.center, self.radius, self.endAngle, self.startAngle)

    def plot(self,color=None,maxArcLen=1.0e-2,showStartPoint=False):
        if havePlt:
            lineSegList = self.convertToLineSegList(maxArcLen=maxArcLen)
            if color is None:
                color = 'b'
            if showStartPoint:
                startPoint = lineSegList[0].startPoint
                plt.plot([startPoint[0]],[startPoint[1]], 'o'+color)
            for i, lineSeg in enumerate(lineSegList):
                lineSeg.plot(color=color)


# SegList functions
# ------------------------------------------------------------------------------

def checkSegListContinuity(segList,closed=False,ptEquivTol=1.0e-5):
    segListDim = getSegListDim(segList)
    if segListDim is None:
        raise ValueError, 'segment list dimension not defined'

    test = True
    if segListDim == 2:
        for segCurr, segNext in zip(segList[:-1], segList[1:]):
            if dist2D(segCurr.endPoint, segNext.startPoint) > ptEquivTol:
                test = False
                break
        if closed:
            segFirst = segList[0]
            segLast = segList[-1]
            if dist2D(segLast.endPoint, segFirst.startPoint) > ptEquivTol:
                test = False
    elif segListDim == 3:
        raise RuntimeError, 'not yet implemented to 3d segment lists'
    else:
        raise RuntimeError, 'not implement to segment list with dim={0}'.format(segListDim)

    return test

def reverseSegList(segList):
    return [seg.reverse() for seg in segList[::-1]]

def getSegListDim(segList): 
    is2D = True
    is3D = True
    for seg in segList:
        if seg.dim != 2:
            is2D = False
        if seg.dim != 3:
            is3D = False
    if is2D:
        return 2
    elif is3D:
        return 3
    else:
        return None

def getSegListLength(segList):
    segLengthList = [seg.length for seg in segList]
    return reduce(operator.add,segLengthList)

def plotSegList(segList,color=None):
    for seg in segList:
        seg.plot(color=color)


# Basic geometery
# -----------------------------------------------------------------------------

def dist2D(p,q):
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def dist3D(p,q):
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2)

def midPoint2D(p,q): 
    return 0.5*(p[0] + q[0]), 0.5*(p[1] + q[1])



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        lineSeg = LineSeg2D((0,0),(2,5))
        if havePlt:
            fig = plt.figure()
            lineSeg.plot()
            plt.show()

    if 1:
        arcSeg = ArcSeg2D((0,0), 1.0, 0.0, math.pi/2.0, 'cw')
        print(arcSeg.length)

        if 1 and havePlt:
            fig = plt.figure()
            arcSeg.plot()
            plt.axis('equal')
            plt.show()

