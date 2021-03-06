"""

Copyright 2014 IO Rodeo Inc. 

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

def getEntityStartAndEndPts(entity):
    if entity.dxftype == 'LINE':
        startPt, endPt = entity.start[:2], entity.end[:2]
    elif entity.dxftype == 'ARC':
        startPt, endPt = getDxfArcStartAndEndPts(entity)
    elif entity.dxftype == 'CIRCLE':
        startPt, endPt = getDxfCircleStartAndEndPts(entity)
    else:
        raise RuntimeError('entity type not yet supported')
    return startPt, endPt

def getDxfArcStartAndEndPts(arc): 
    xc = arc.center[0]
    yc = arc.center[1]
    r = arc.radius
    try:
        angStart = (math.pi/180.0)*arc.start_angle
    except AttributeError:
        angStart = (math.pi/180.0)*arc.startangle
    try:
        angEnd = (math.pi/180.0)*arc.end_angle
    except AttributeError:
        angEnd = (math.pi/180.0)*arc.endangle
    if angEnd < angStart:
        angEnd += 2.0*math.pi 
    x0 = xc + r*math.cos(angStart)
    y0 = yc + r*math.sin(angStart)
    x1 = xc + r*math.cos(angEnd)
    y1 = yc + r*math.sin(angEnd)
    startPt = x0,y0
    endPt = x1,y1
    return startPt,endPt

def getDxfCircleStartAndEndPts(circle):
    xc = circle.center[0]
    yc = circle.center[1]
    r = circle.radius
    x0 = xc + r
    y0 = yc
    x1 = xc + r
    y1 = yc
    startPt = x0, y0
    endPt = x1, y1
    return startPt, endPt

