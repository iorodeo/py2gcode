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
import numpy
try:
    import matplotlib.pyplot as plt
    havePlt = True
except ImportError:
    havePlt = False



class LineSeg(object):

    def __init__(self,startPoint,endPoint): 
        self.startPoint = startPoint
        self.endPoint = endPoint

    def plot(self,color=None):
        if havePlt:
            x0,y0 = self.startPoint
            x1,y1 = self.endPoint
            if color is None:
                color = 'b'
            plt.plot([x0,x1],[y0,y1],color=color)


class ArcSeg(object):
    
    def __init__(self,center,radius,startAngle,endAngle):
        self.center = center
        self.radius = radius
        self.startAngle = startAngle
        self.endAngle = endAngle
        self.checkAngles()

    def checkAngles(self):
        if self.startAngle < 0 or self.startAngle > 2.0*math.pi:
            raise RuntimeError('startAngle must be between 0 and 2*pi')
        if self.endAngle < 0 or self.endAngle > 2.0*math.pi:
            raise RuntimeError('endAngle must be between 0 and 2*pi')

    @property
    def endAngleAdj(self):
        if self.endAngle < self.startAngle:
            endAngleAdj = self.endAngle + 2.0*math.pi 
        else:
            endAngleAdj = self.endAngle
        return endAngleAdj

    @property
    def startPoint(self):
        pass

    @property
    def endPoint(self):
        pass



    def convertToLineSegList(self, maxArcLen=1.0e-5):
        totalAngle = abs(self.endAngleAdj - self.startAngle)
        maxStepAngle = maxArcLen/self.radius
        numPts = int(math.ceil(totalAngle/maxStepAngle))
        stepAngleArray = numpy.linspace(self.startAngle, self.endAngleAdj, numPts)
        lineSegList = []
        for ang0, ang1 in zip(stepAngleArray[:-1], stepAngleArray[1:]):
            x0 = self.center[0] + self.radius*math.cos(ang0)
            y0 = self.center[1] + self.radius*math.sin(ang0)
            x1 = self.center[0] + self.radius*math.cos(ang1)
            y1 = self.center[1] + self.radius*math.sin(ang1)
            lineSeg = LineSeg((x0,y0), (x1,y1))
            lineSegList.append(lineSeg)
        return lineSegList

    def plot(self,color=None,maxArcLen=1.0e-2):
        if havePlt:
            lineSegList = self.convertToLineSegList(maxArcLen=maxArcLen)
            if color is None:
                color = 'b'
            for lineSeg in lineSegList:
                lineSeg.plot(color=color)


def checkSegListContinuity(segList,ptEquivtol=1.0e-5):

    for segCurr, segNext in zip(segList[:-1], segList[1:]):
        pass




#def getDxfArcStartAndEndPts(arc): 
#    xc = arc.center[0]
#    yc = arc.center[1]
#    r = arc.radius
#    angStart = (math.pi/180.0)*arc.startangle
#    angEnd = (math.pi/180.0)*arc.endangle
#    if angEnd < angStart:
#        angEnd += 2.0*math.pi 
#    x0 = xc + r*math.cos(angStart)
#    y0 = yc + r*math.sin(angStart)
#    x1 = xc + r*math.cos(angEnd)
#    y1 = yc + r*math.sin(angEnd)
#    startPt = x0,y0
#    endPt = x1,y1
#    return startPt,endPt
#
#
#def getEntityStartAndEndPts(entity):
#    if entity.dxftype == 'LINE':
#        startPt, endPt = entity.start[:2], entity.end[:2]
#    elif entity.dxftype == 'ARC':
#        startPt, endPt = getDxfArcStartAndEndPts(entity)
#    else:
#        raise ValueError('entity type not yet supported')
#    return startPt, endPt


            

def dist2D(p,q):
    return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

def midPoint2D(p,q): 
    return 0.5*(p[0] + q[0]), 0.5*(p[1] + q[1])

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        lineSeg = LineSeg((0,0),(2,5))
        if havePlt:
            fig = plt.figure()
            lineSeg.plot()
            plt.show()

    if 1:
        arcSeg = ArcSeg((0,0),2,math.pi/2.0,0.0)
        if havePlt:
            fig = plt.figure()
            arcSeg.plot()
            plt.axis('equal')
            plt.show()

