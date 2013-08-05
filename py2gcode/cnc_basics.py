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


class RectPathXY(GCodeProg):
    """
    Rectangle in xy plane made of LinearFeeds which is defined by points q and
    p. Note, prior to rectangle tool is moved from current position to start
    point p via a LinearFeed.  There is no move to safe height etc.
    """

    def __init__(self,p,q):
        super(RectPathXY,self).__init__()
        self.p = p
        self.q = q

    @property 
    def listOfCmds(self):
        x0, y0 = self.p
        x1, y1 = self.q 
        pointList = [(x0,y0), (x0,y1), (x1,y1), (x1,y0),(x0,y0)]
        listOfCmds = []
        for point in pointList:
            x,y = point
            listOfCmds.append(LinearFeed(x=x,y=y))
        return listOfCmds

    @listOfCmds.setter
    def listOfCmds(self,value):
        pass



class FilledRectPathXY(GCodeProg):
    """
    Rectangular path made up of LinearFeeds which is defined points p and q. 
    """

    def __init__(self,p,q,step):
        super(FilledRectPathXY,self).__init__()
        self.p = p
        self.q = q
        self.step = step
        self.checkStep()

    def checkStep(self):
        x0, y0 = self.p
        x1, y1 = self.q
        dx = abs(x1-x0)
        dy = abs(y1-y0)
        if dx < self.step or dy < self.step:
            raise ValueError, 'step size is too small'

    @property
    def listOfCmds(self):
        x0, y0 = self.p
        x1, y1 = self.q
        xMid = 0.5*(x0 + x1)
        yMid = 0.5*(y0 + y1)
        listOfCmds = []
        dx = abs(x0 - xMid)
        dy = abs(y0 - xMid)
        while dx > 0 and dy > 0:
            p = xMid + dx, yMid + dy
            q = xMid - dx, yMid - dy
            r = RectPathXY(p,q)
            listOfCmds.extend(r.listOfCmds)
            dx -= self.step
            dy -= self.step
        return listOfCmds

    @listOfCmds.setter
    def listOfCmds(self,value):
        pass


if __name__ == '__main__':

    # Test program
    prog = GCodeProg()

    prog.add(GenericStart())
    prog.add(Space())

    prog.add(FeedRate(10.0))
    prog.add(Space())

    p = ( 1.0,  1.0)
    q = (-1.0, -1.0)
    zSafe = 1.0
    zCut = 0.0

    prog.add(Comment('Move to start'))
    prog.add(RapidMotion(x=p[0],y=p[1],z=zSafe))
    prog.add(Space())

    prog.add(Comment('RectPathXY'))
    prog.add(RectPathXY(p,q))
    prog.add(Space())

    prog.add(Comment('FilledRectPathXY'))
    prog.add(FilledRectPathXY(p,q,0.05))
    prog.add(Space())
    
    prog.add(Space())
    prog.add(End(),comment=True)

    print(prog)
    prog.write('test.ngc')


