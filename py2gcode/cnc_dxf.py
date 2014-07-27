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
import gcode_cmd
import cnc_drill
import cnc_pocket
import dxfgrabber
import networkx
import numpy
import matplotlib.pyplot as plt

class DxfBase(gcode_cmd.GCodeProg):

    ALLOWED_TYPE_LIST = None
    DEFAULT_PARAM = {'dxfTypes': []}

    def __init__(self,param):
        self.param = dict(self.DEFAULT_PARAM)
        self.param.update(param)
        try:
            self.dwg = self.param['dwg']
        except KeyError:
            self.dwg = dxfgrabber.readfile(self.param['fileName'])
        self.makeListOfCmds()

    @property
    def layerNameList(self):
        try:
            layerNameList = self.param['layers']
        except KeyError:
            layerNameList = [layer.name for layer in self.dwg.layers]
        return layerNameList

    @property
    def entityList(self):
        entityList = [x for x in self.dwg.entities if x.layer in self.layerNameList]
        entityList = [x for x in entityList if x.dxftype in self.param['dxfTypes']] 
        entityList = [x for x in entityList if x.dxftype in self.ALLOWED_TYPE_LIST]
        return entityList


class DxfDrill(DxfBase):

    ALLOWED_TYPE_LIST = ['POINT', 'CIRCLE', 'ARC']
    DEFAULT_PARAM = {'dxfTypes': ['CIRCLE']}

    def __init__(self,param):
        super(DxfDrill,self).__init__(param)

    @property
    def drillClass(self):
        if 'stepZ' in self.param:
            drill = cnc_drill.PeckDrill
        else:
            drill = cnc_drill.SimpleDrill
        return drill 

    def makeListOfCmds(self):
        self.listOfCmds = []
        for entity in self.entityList:
            drillParam = dict(self.param)
            if entity.dxftype == 'POINT':
                centerPt = entity.point
            else:
                centerPt = entity.center 
            drillParam['centerX'] = centerPt[0]
            drillParam['centerY'] = centerPt[1]
            drill = self.drillClass(drillParam)
            self.listOfCmds.extend(drill.listOfCmds)


class DxfCircPocket(DxfBase):

    ALLOWED_TYPE_LIST = ['CIRCLE']
    DEFAULT_PARAM = {'dxfTypes': ['CIRCLE']}

    def __init__(self,param):
        super(DxfCircPocket,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        for entity in self.entityList:
            pocketParam = dict(self.param)
            pocketParam['centerX'] = entity.center[0]
            pocketParam['centerY'] = entity.center[1]
            pocketParam['radius'] = entity.radius
            pocket = cnc_pocket.CircPocketXY(pocketParam)
            self.listOfCmds.extend(pocket.listOfCmds)


class DxfBoundary(DxfBase):

    ALLOWED_TYPE_LIST = ['LINE','ARC']
    DEFAULT_PARAM = {
            'dxfTypes'    :  ['LINE','ARC'],
            'convertArcs' :  True,
            'ptEquivTol'  :  1.0e-6,
            #'maxArcLen'   :  1.0e-2,
            'maxArcLen'   :  1.0e-1,
            }

    def __init__(self,param):
        super(DxfBoundary,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        if self.param['convertArcs']:
            lineList = self.getLineList()
            for line in lineList:
                p0, p1 = line
                x0, y0 = p0
                x1, y1 = p1
                plt.plot([x0,x1],[y0,y1],'b')
            plt.show()
        else:
            raise ValueError, 'case convertArc = False not implemented yet'

    def getLineList(self):
        lineList = []
        for entity in self.entityList:
            if entity.dxftype  == 'LINE':
                line = entity.start[:2], entity.end[:2]
                lineList.append(line)
            else:
                arcLineList = self.convertArcToLineList(entity)
                lineList.extend(arcLineList)
        return lineList

    def convertArcToLineList(self,arc):
        xc = arc.center[0]
        yc = arc.center[1]
        r = arc.radius
        angStart = (math.pi/180.0)*arc.startangle
        angEnd = (math.pi/180.0)*arc.endangle
        # Get array of steps from start to end angle
        if angEnd < angStart:
            angEnd += 2.0*math.pi 
        totalAng = abs(angEnd - angStart)
        maxStepAng = self.param['maxArcLen']/arc.radius
        numPts = int(math.ceil(totalAng/maxStepAng))
        angStepArray = numpy.linspace(angStart, angEnd, numPts)
        # Create line segments
        lineList = []
        for ang0, ang1 in zip(angStepArray[:-1], angStepArray[1:]):
            x0 = xc + r*math.cos(ang0)
            y0 = yc + r*math.sin(ang0)
            x1 = xc + r*math.cos(ang1)
            y1 = yc + r*math.sin(ang1)
            lineSeg = ((x0,y0), (x1,y1))
            lineList.append(lineSeg)
        return lineList








# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import os

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.FeedRate(120.0))

    dxfDir = os.path.join(os.curdir,'test_dxf')

    if 0:
        fileName = os.path.join(dxfDir,'drill_test.dxf')
        param = { 
                'fileName'    : fileName,
                'startZ'      : 0.02,
                'stopZ'       : -0.5,
                'safeZ'       : 0.5,
                'stepZ'       : 0.05,
                'startDwell'  : 2.0,
                }
        drill = DxfDrill(param)
        prog.add(drill)


    if 0:
        fileName = os.path.join(dxfDir,'drill_test.dxf')
        param = { 
                'fileName'    : fileName,
                'layers'      : ['layer1'],
                'dxfTypes'    : ['CIRCLE'],
                'startZ'      : 0.02,
                'stopZ'       : -0.5,
                'safeZ'       : 0.5,
                'stepZ'       : 0.05,
                'startDwell'  : 2.0,
                }
        drill = DxfDrill(param)
        prog.add(drill)

    if 0:
        fileName = os.path.join(dxfDir,'circ_pocket_test.dxf')
        param = {
                'fileName'       : fileName,
                'depth'          : 0.4,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.1,
                'overlapFinish'  : 0.1,
                'maxCutDepth'    : 0.2,
                'toolDiam'       : 0.25,
                'direction'      : 'ccw',
                'startDwell'     : 2.0,
                }
        pocket = DxfCircPocket(param)
        prog.add(pocket)

    if 0:
        fileName = os.path.join(dxfDir,'circ_pocket_test.dxf')
        param = {
                'fileName'       : fileName,
                'layers'         : ['layer1'],
                'depth'          : 0.4,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.1,
                'overlapFinish'  : 0.1,
                'maxCutDepth'    : 0.2,
                'toolDiam'       : 0.25,
                'direction'      : 'ccw',
                'startDwell'     : 2.0,
                }
        pocket = DxfCircPocket(param)
        prog.add(pocket)

    if 1:
        #fileName = os.path.join(dxfDir,'boundary_test0.dxf')
        fileName = os.path.join(dxfDir,'boundary_test1.dxf')
        #fileName = os.path.join(dxfDir,'boundary_test2.dxf')
        param = {
                'fileName'       : fileName,
                'depth'       : 0.03,
                'startZ'      : 0.0,
                'safeZ'       : 0.15,
                'toolDiam'    : 0.25,
                'direction'   : 'ccw',
                'cutterComp'  : 'right',
                'maxCutDepth' : 0.03,
                'startDwell'  : 2.0,
                }
        boundary = DxfBoundary(param)
        prog.add(boundary)

    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')
