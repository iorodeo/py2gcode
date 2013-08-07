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

    def __init__(self,feedrate=None, units='in',comment=True):
        super(GenericStart,self).__init__()
        self.add(Space())
        self.add(Comment('Generic Start'))
        self.add(CancelCutterCompensation(),comment=comment)
        self.add(CancelToolLengthOffset(),comment=comment)
        self.add(CancelCannedCycle(),comment=comment)
        self.add(CoordinateSystem(1),comment=comment)
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
                listOfCmd.append(LinearFeed(**point))
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

    def __init__(self,point0,point1,step,num):
        super(FilledRectPathXY,self).__init__()
        self.point0 = point0
        self.point1 = point1
        self.step = step
        self.num = num
        self.checkStep()

    def checkStep(self):
        x0, y0 = self.point0
        x1, y1 = self.point1
        dx = abs(x1-x0)
        dy = abs(y1-y0)
        if dx < self.step or dy < self.step:
            raise ValueError, 'step size is too small'

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
            dy0 =  self.step
            dy1 = -self.step
            def yDoneTest(y0,y1):
                return y0 > y1
        else:
            dy0 = -self.step
            dy1 =  self.step
            def yDoneTest(y0,y1):
                return y0 <= y1

        pointList = []
        for i in range(self.num):
            p0 = (x0,y0)
            p1 = (x1,y1)
            rectPath = RectPathXY(p0,p1)
            pointList.extend(rectPath.pointList)
            x0 += dx0
            x1 += dx1
            y0 += dy0
            y1 += dy1
            if xDoneTest(x0,x1):
                break
            if yDoneTest(y0,y1):
                break
        return pointList


class FilledRectWithCornerCutPathXY(LinearFeedPath):

    def __init__(self,point0,point1,step,num,cutLen):
        self.point0 = point0
        self.point1 = point1
        self.step = step
        self.num = num
        self.cutLen = cutLen

    @property
    def pointList(self):
        filledPath = FilledRectPathXY(
                self.point0, 
                self.point1,
                self.step, 
                self.num
                )
        cornerCutPath = RectWithCornerCutPathXY(
                self.point0, 
                self.point1, 
                self.cutLen
                )
        pointList = cornerCutPath.pointList + filledPath.pointList[5:]
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

    if 1:
        p = ( 1.0,  1.0)
        q = (-1.0, -1.0)
        step = 0.1
        num = 5
        cutLen = 0.25

        prog.add(Comment('FilledRectWithCornerCutPathXY'))
        prog.add(FilledRectWithCornerCutPathXY(p,q,step,num,cutLen))
        prog.add(Space())

    
    prog.add(Space())
    prog.add(End(),comment=True)

    #print(prog)
    prog.write('test.ngc')


