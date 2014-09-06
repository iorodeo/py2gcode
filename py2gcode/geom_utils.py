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

class SegmentBase(object):

    def __init__(self): 
        self.segType = None 

class LineSegment(SegmentBase):

    def __init__(self,startPt,endPt): 
        self.segType = 'line'
        self.startPt = startPt
        self.endPt = endPt

class ArcSegment(SegmentBase):
    
    def __init__(self,startPt,endPt,radius):
        self.segType = 'arc'
        self.startPt = startPt
        self.endPt = endPt
        self.radius = radius
        self.checkRadius()

    def checkRadius(self):
        if abs(self.radius) < dist2D(self.startPt, self.endPt):
            raise RuntimeError, 'radius must be >= dist between start and end pts'

def dist2D(p,q):
    return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

def midPoint2D(p,q): 
    return 0.5*(p[0] + q[0]), 0.5*(p[1] + q[1])

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    LineSegment((0,0),(0,5))
    ArcSegment((0,5),(5,5),6)

