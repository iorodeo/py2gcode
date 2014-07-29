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
import cnc_boundary
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
            'maxArcLen'   :  0.5e-1,
            #'maxArcLen'   :  1.0e-2,
            'startCond'   : 'minX',
            }

    def __init__(self,param):
        super(DxfBoundary,self).__init__(param)

    def makeCmdsForLineString(self,graph):
        listOfCmds = []
        if self.param['cutterComp'] is not None:
            errorMsg = 'cutterComp must be None for line string graphs'
            raise ValueError, errorMsg

        # Get start and end  node based on startCond.
        endNodeList = [n for n in graph if graph.degree(n) == 1]
        if self.param['startCond'] in ('minX', 'maX'):
            endCoordAndNodeList = [(graph.node[n]['coord'][0],n)  for n in endNodeList]
        elif self.param['startCond'] in ('minY', 'maxY'):
            endCoordAndNodeList = [(graph.node[n]['coord'][1],n)  for n in endNodeList]
        else:
            raise ValueError, 'unknown startCond {0}'.format(self.param['startCond'])
        endCoordAndNodeList.sort()
        if 'min' in  self.param['startCond']:
            startNode = endCoordAndNodeList[0][1]
            endNode = endCoordAndNodeList[1][1]
        else:
            startNode = endCoordAndNodeList[1][1]
            endNode = endCoordAndNodeList[0][1]

        # Get path from start to end node (there is only one)
        simplePathList = networkx.all_simple_paths(graph, startNode,endNode)
        startToEndPath = simplePathList.next()

        # Get list of line segments (line or arc) from start to end
        segList = []
        for node0, node1 in zip(startToEndPath[:-1],startToEndPath[1:]):
            startCoord = graph.node[node0]['coord']
            endCoord = graph.node[node1]['coord']
            edgeEntity = graph[node0][node1]['entity']
            if edgeEntity.dxftype == 'LINE':
                segList.append((startCoord, endCoord))
            else: 
                if self.param['convertArcs']:
                    arcSegList = self.convertArcToLineList(edgeEntity)
                    if arcSegList[0][0] != startCoord:
                        arcSegList = [(y,x) for x,y in arcSegList[::-1]]
                    segList.extend(arcSegList)
                else:
                    raise ValueError, 'convertArcs=False not supported yet'
        
        if self.param['convertArcs']:
            pointList = [p[0] for p in segList]
            pointList.append(segList[-1][1])
            boundaryParam = dict(self.param)
            boundaryParam['pointList'] = pointList 
            boundaryParam['closed'] = False
            boundary = cnc_boundary.LineSegBoundaryXY(boundaryParam)
            listOfCmds = boundary.listOfCmds
        else:
            raise ValueError, 'convertArcs=False not supported yet'
        return listOfCmds

    def makeCmdsForClosedLoop(self,graph):
        print('makeCmdsForClosedLoop')
        listOfCmds = []
        return listOfCmds


    def makeListOfCmds(self):
        self.listOfCmds = []
        # Get entity graph and find connected components
        graph, ptToNodeDict = getEntityGraph(self.entityList,self.param['ptEquivTol'])
        connectedCompSubGraphs = networkx.connected_component_subgraphs(graph)
        # Remove any trivial edges - sometimes happens due to drawing errors
        for edge in graph.edges():
            if edge[0] == edge[1]:
                graph.remove_edge(*edge)
        for i, subGraph in enumerate(connectedCompSubGraphs):
            nodeDegreeList = [subGraph.degree(n) for n in subGraph]
            maxNodeDegree = max(nodeDegreeList)
            minNodeDegree = min(nodeDegreeList)
            if maxNodeDegree > 2:
                # Graph is complicated - treat each entity as separate task 
                for edge in subGraph.edges():
                    edgeGraph = subGraph.subgraph(edge)
                    listOfCmds = self.makeCmdsForLineString(edgeGraph)
                    self.listOfCmds.extend(listOfCmds)
            elif maxNodeDegree == 2 and minNodeDegree == 2:
                # Graph is closed loop
                listOfCmds = self.makeCmdsForClosedLoop(subGraph)
                self.listOfCmds.extend(listOfCmds)
            elif minNodeDegree == 1:
                # Graph is line string
                listOfCmds = self.makeCmdsForLineString(subGraph)
                self.listOfCmds.extend(listOfCmds)
            else:
                errorMsg = 'sub-graph has nodes with degree 0'
                raise ValueError, errorMsg
    
    def getEntityLineList(self):
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


# Utility functions
# -----------------------------------------------------------------------------
def getEntityGraph(entityList, ptEquivTol=1.0e-6):
    ptToNodeDict = getPtToNodeDict(entityList,ptEquivTol)
    graph = networkx.Graph()
    for entity in entityList:
        startPt, endPt = getEntityStartAndEndPts(entity)
        startNode = ptToNodeDict[startPt]
        graph.add_node(startNode,coord=startPt)
        endNode = ptToNodeDict[endPt]
        graph.add_node(endNode,coord=endPt)
        graph.add_edge(startNode, endNode, entity=entity)
    return graph, ptToNodeDict

def getPtToNodeDict(entityList, ptEquivTol=1.0e-6):
    ptList = []
    for entity in entityList:
        startPt, endPt = getEntityStartAndEndPts(entity)
        ptList.extend([startPt, endPt])
    ptToNodeDict = {}
    nodeCnt = 0
    for i, p in enumerate(ptList):
        found = False
        for q in ptList[:i]:
            if dist(p,q) < ptEquivTol:
                found = True
                ptToNodeDict[p] = ptToNodeDict[q] 
                break
        if not found:
            ptToNodeDict[p] = nodeCnt
            nodeCnt += 1
    return ptToNodeDict

def getDxfArcStartAndEndPts(arc): 
    xc = arc.center[0]
    yc = arc.center[1]
    r = arc.radius
    angStart = (math.pi/180.0)*arc.startangle
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


def getEntityStartAndEndPts(entity):
    if entity.dxftype == 'LINE':
        startPt, endPt = entity.start[:2], entity.end[:2]
    elif entity.dxftype == 'ARC':
        startPt, endPt = getDxfArcStartAndEndPts(entity)
    else:
        raise ValueError, 'entity type not yet supported'
    return startPt, endPt


def dist(p,q):
    return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)


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
        #fileName = os.path.join(dxfDir,'boundary_test1.dxf')
        #fileName = os.path.join(dxfDir,'boundary_test2.dxf')
        fileName = os.path.join(dxfDir,'boundary_test3.dxf')
        param = {
                'fileName'       : fileName,
                'depth'       : 0.03,
                'startZ'      : 0.0,
                'safeZ'       : 0.15,
                'toolDiam'    : 0.25,
                'direction'   : 'ccw',
                'cutterComp'  : None,
                'maxCutDepth' : 0.03,
                'startDwell'  : 2.0,
                }
        boundary = DxfBoundary(param)
        prog.add(boundary)

    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')
