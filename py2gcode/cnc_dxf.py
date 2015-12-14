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
import shapely.geometry.polygon as polygon
import matplotlib.pyplot as plt

from geom_utils import dist2D
from graph_utils import getEntityGraph
from dxf_utils import getEntityStartAndEndPts

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
    DEFAULT_PARAM = {
            'dxfTypes'  : ['CIRCLE'],
            'startCond' : 'minX',
            }

    def __init__(self,param):
        super(DxfDrill,self).__init__(param)

    @property
    def drillClass(self):
        if 'stepZ' in self.param:
            drill = cnc_drill.PeckDrill
        else:
            drill = cnc_drill.SimpleDrill
        return drill 

    def getCenterPt(self,entity):
        if entity.dxftype == 'POINT':
            centerPt = entity.point
        else:
            centerPt = entity.center 
        return centerPt

    def makeListOfCmds(self):
        self.listOfCmds = []
        # ------------------------------------------------------------------------
        # Note: for better efficiency it might be worth while sorting the entities
        # based on some criteria .....distance, etc.
        # -------------------------------------------------------------------------
        for entity in self.entityList:
            drillParam = dict(self.param)
            centerPt = self.getCenterPt(entity)
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
            if 'thickness' in pocketParam:
                pocket = cnc_pocket.CircAnnulusPocketXY(pocketParam)
            else:
                pocket = cnc_pocket.CircPocketXY(pocketParam)
            self.listOfCmds.extend(pocket.listOfCmds)


class DxfRectPocketFromExtent(DxfBase):

    ALLOWED_TYPE_LIST = ['LINE','POINT']
    DEFAULT_PARAM = {
            'dxfTypes'    : ['LINE','POINT'],
            'ptEquivTol'  :  1.0e-5,
            'components'  : True,
            }

    def __init__(self,param):
        super(DxfRectPocketFromExtent,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        if self.param['components']:
            # Get entity graph and find connected components
            graph, ptToNodeDict = getEntityGraph(self.entityList,self.param['ptEquivTol'])
            connectedCompSubGraphs = networkx.connected_component_subgraphs(graph)
            # Create list of commands for each connected component individually
            for i, subGraph in enumerate(connectedCompSubGraphs):
                entityList = [subGraph[n][m]['entity'] for n, m in subGraph.edges()]
                self.listOfCmds.extend(self.makeListOfCmdsForEntityList(entityList))
        else:
            self.listOfCmds.extend(self.makeListOfCmdsForEntityList(self.entityList))

    def makeListOfCmdsForEntityList(self,entityList):
        """
        Generates rectangular pocket from extent of entities in the given list.
        """
        coordList = []
        for entity in entityList:
            if entity.dxftype == 'LINE':
                coordList.append(entity.start[:2])
                coordList.append(entity.end[:2])
            elif entity.dxftype == 'POINT':
                coordList.append(entity.point[:2])
            else:
                raise RuntimeError('dxftype {0} not supported yet'.format(entity.dxftype))

        # Get x and y coordinates max and min values
        xCoordList = [p[0] for p in coordList]
        yCoordList = [p[1] for p in coordList]
        xMax = max(xCoordList)
        xMin = min(xCoordList)
        yMax = max(yCoordList)
        yMin = min(yCoordList)

        # Calculate center, width and height
        centerX = 0.5*(xMax + xMin)
        centerY = 0.5*(yMax + yMin)
        width = xMax - xMin
        height = yMax - yMin

        # Create list of commands
        pocketParam = dict(self.param)
        pocketParam['centerX'] = centerX
        pocketParam['centerY'] = centerY
        pocketParam['width'] = width
        pocketParam['height'] = height
        if 'thickness' in pocketParam:
            pocket = cnc_pocket.RectAnnulusPocketXY(pocketParam)
        else:
            pocket = cnc_pocket.RectPocketXY(pocketParam)
        return pocket.listOfCmds 



class DxfCircBoundary(DxfBase):

    ALLOWED_TYPE_LIST = ['CIRCLE']
    DEFAULT_PARAM = {'dxfTypes': ['CIRCLE']}

    def __init__(self,param):
        super(DxfCircBoundary,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        for entity in self.entityList:
            bndryParam = dict(self.param)
            bndryParam['centerX'] = entity.center[0]
            bndryParam['centerY'] = entity.center[1]
            bndryParam['radius'] = entity.radius
            bndry = cnc_boundary.CircBoundaryXY(bndryParam)
            self.listOfCmds.extend(bndry.listOfCmds)


class DxfBoundary(DxfBase):

    ALLOWED_TYPE_LIST = ['LINE','ARC']
    DEFAULT_PARAM = {
            'dxfTypes'    :  ['LINE','ARC'],
            'convertArcs' :  True,
            'ptEquivTol'  :  1.0e-5,
            'maxArcLen'   :  1.0e-2,
            'startCond'   : 'minX',
            }

    def __init__(self,param):
        super(DxfBoundary,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []
        # Get entity graph and find connected components
        graph, ptToNodeDict = getEntityGraph(self.entityList,self.param['ptEquivTol'])
        connectedCompSubGraphs = networkx.connected_component_subgraphs(graph)
        # Create list of commands for each connected component individually
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
                raise RuntimeError(errorMsg)
            

    def makeCmdsForLineString(self,graph):
        if self.param['cutterComp'] is not None:
            errorMsg = 'cutterComp must be None for line string graphs'
            raise RuntimeError(errorMsg)

        # Get start and end  node based on startCond.
        endNodeList = [n for n in graph if graph.degree(n) == 1]
        if self.param['startCond'] in ('minX', 'maX'):
            endCoordAndNodeList = [(graph.node[n]['coord'][0],n)  for n in endNodeList]
        elif self.param['startCond'] in ('minY', 'maxY'):
            endCoordAndNodeList = [(graph.node[n]['coord'][1],n)  for n in endNodeList]
        else:
            raise ValueError('unknown startCond {0}'.format(self.param['startCond']))
        endCoordAndNodeList.sort()
        if 'min' in  self.param['startCond']:
            startNode = endCoordAndNodeList[0][1]
            endNode = endCoordAndNodeList[1][1]
        else:
            startNode = endCoordAndNodeList[1][1]
            endNode = endCoordAndNodeList[0][1]

        # Get path from start to end node (there is only one)
        simplePathGen = networkx.all_simple_paths(graph, startNode,endNode)
        startToEndPath = simplePathGen.next()

        # Get list of segments (line or arc) along path and create cnc commands
        segList = self.getSegListFromPath(startToEndPath, graph)
        param = dict(self.param)
        param['closed'] = False 
        listOfCmds = self.makeListOfCmdsFromSegList(segList,param)
        return listOfCmds

    def makeCmdsForClosedLoop(self,graph):
        # Get start and end nodes based on startCond
        if self.param['startCond'] in ('minX', 'maxX'):
            coordAndNodeList = [(graph.node[n]['coord'][0], n) for n in graph]
        elif self.param['startCond'] in ('minY', 'maxY'):
            coordAndNodeList = [(graph.node[n]['coord'][1], n) for n in graph]
        else:
            raise ValueError('unknown startCond {0}'.format(self.param['startCond']))
        coordAndNodeList.sort()
        if 'min' in self.param['startCond']:
            startNode = coordAndNodeList[0][1]
        else:
            startNode = coordAndNodeList[-1][1]
        endNode = graph.neighbors(startNode)[0]

        # Get path around graph
        simplePathList = [p for p in networkx.all_simple_paths(graph,startNode,endNode)]
        lenAndSimplePathList = [(len(p),p) for p in simplePathList]
        closedPath = max(lenAndSimplePathList)[1]
        closedPath.append(startNode)
        closedPathCoord = [graph.node[n]['coord'] for n in closedPath]


        ## ==============================================
        ## DEBUG
        ## ==============================================
        #xvals = [x for x,y in closedPathCoord]
        #yvals = [y for x,y in closedPathCoord]
        #plt.plot(xvals,yvals)
        #plt.show()
        ## ==============================================

        lineString = polygon.LineString(closedPathCoord)
        # Test for self instersections and if none orient closed loop for cutting direction
        if not lineString.is_simple:
            if self.param['cutterComp'] is not None:
                raise RuntimeError('cutterComp is not allowed for non-simple closed loops')
            cutterComp = None
        else:
            linearRing = polygon.LinearRing(closedPathCoord)
            cwTest = self.param['direction'] == 'cw' and linearRing.is_ccw
            ccwTest = self.param['direction'] == 'ccw' and not linearRing.is_ccw
            if cwTest or ccwTest:
                closedPath.reverse()
                closedPathCoord.reverse()

            cutterComp = self.param['cutterComp']
            if cutterComp in ('inside', 'outside'):
                cutterCompTable = {
                        ('inside',  'ccw') : 'left',
                        ('inside',  'cw')  : 'right',
                        ('outside', 'ccw') : 'right',
                        ('outside', 'cw')  : 'left',
                        }
                cutterComp = cutterCompTable[(cutterComp,self.param['direction'])]

        # Get list of segments (line or arc) along path and create cnc commands
        segList = self.getSegListFromPath(closedPath,graph)
        param = dict(self.param)
        param['closed'] = True 
        param['cutterComp'] = cutterComp
        listOfCmds = self.makeListOfCmdsFromSegList(segList,param)
        return listOfCmds

    def makeListOfCmdsFromSegList(self,segList,param):
        listOfCmds = []
        if self.param['convertArcs']:
            pointList = [p[0] for p in segList]
            pointList.append(segList[-1][1])
            param['pointList'] = pointList 
            boundary = cnc_boundary.LineSegBoundaryXY(param)
            listOfCmds = boundary.listOfCmds
        else:
            raise RuntimeError('convertArcs=False not supported yet')

        #xList = [p[0] for p in pointList]
        #yList = [p[1] for p in pointList]
        #plt.plot(xList[:78],yList[:78],'.')
        #plt.axis('equal')
        #plt.show()

        return listOfCmds

    def getSegListFromPath(self, nodePath,  graph):
        segList = []
        for node0, node1 in zip(nodePath[:-1],nodePath[1:]):
            startCoord = graph.node[node0]['coord']
            endCoord = graph.node[node1]['coord']
            edgeEntity = graph[node0][node1]['entity']
            if edgeEntity.dxftype == 'LINE':
                segList.append((startCoord, endCoord))
            else: 
                if self.param['convertArcs']:
                    arcSegList = self.convertDxfArcToLineList(edgeEntity)
                    if dist2D(arcSegList[0][0],startCoord) > self.param['ptEquivTol']:
                        arcSegList = [(y,x) for x,y in arcSegList[::-1]]
                    segList.extend(arcSegList)
                else:
                    raise RuntimeError('convertArcs=False not supported yet')
        return segList


    def getEntityLineList(self):
        lineList = []
        for entity in self.entityList:
            if entity.dxftype  == 'LINE':
                line = entity.start[:2], entity.end[:2]
                lineList.append(line)
            else:
                arcLineList = self.convertDxfArcToLineList(entity)
                lineList.extend(arcLineList)
        return lineList
    

    def convertDxfArcToLineList(self,arc):
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



## Utility functions
## -----------------------------------------------------------------------------
#def getEntityGraph(entityList, ptEquivTol=1.0e-6):
#    ptToNodeDict = getPtToNodeDict(entityList,ptEquivTol)
#    graph = networkx.Graph()
#    for entity in entityList:
#        startPt, endPt = getEntityStartAndEndPts(entity)
#        startNode = ptToNodeDict[startPt]
#        graph.add_node(startNode,coord=startPt)
#        endNode = ptToNodeDict[endPt]
#        graph.add_node(endNode,coord=endPt)
#        graph.add_edge(startNode, endNode, entity=entity)
#    for edge in graph.edges():
#        # Remove any trivial edges - perhaps due to drawing errors?
#        if edge[0] == edge[1]:
#            graph.remove_edge(*edge)
#    return graph, ptToNodeDict
#
#def getPtToNodeDict(entityList, ptEquivTol=1.0e-6):
#    ptList = []
#    for entity in entityList:
#        startPt, endPt = getEntityStartAndEndPts(entity)
#        ptList.extend([startPt, endPt])
#    ptToNodeDict = {}
#    nodeCnt = 0
#    for i, p in enumerate(ptList):
#        found = False
#        for q in ptList[:i]:
#            if dist2D(p,q) < ptEquivTol:
#                found = True
#                ptToNodeDict[p] = ptToNodeDict[q] 
#                break
#        if not found:
#            ptToNodeDict[p] = nodeCnt
#            nodeCnt += 1
#    return ptToNodeDict

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
#        raise RuntimeError('entity type not yet supported')
#    return startPt, endPt




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

    if 1:
        #fileName = os.path.join(dxfDir, 'circ_boundary_test0.dxf')
        fileName = os.path.join(dxfDir, 'circ_boundary_test1.dxf')
        param = {
                'fileName'     : fileName,
                'layers'       : ['layer1', 'layer2'],
                'depth'        : 0.2,
                'startZ'       : 0.0,
                'safeZ'        : 0.15,
                'toolDiam'     : 0.25,
                'cutterComp'   : 'inside',
                'direction'    : 'ccw',
                'maxCutDepth'  : 0.03,
                'startDwell'   : 2.0,
                }
        boundary = DxfCircBoundary(param)
        prog.add(boundary)

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
        fileName = os.path.join(dxfDir,'rect_extent_test0.dxf')
        param = {
                'fileName'       : fileName,
                'components'     : False,
                'depth'          : 2*0.04,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.3,
                'overlapFinish'  : 0.5,
                'maxCutDepth'    : 0.04,
                'toolDiam'       : 0.25,
                'cornerCut'      : False,
                'direction'      : 'ccw',
                'startDwell'     : 2.0,
                }
        pocket = DxfRectPocketFromExtent(param)
        prog.add(pocket)


    if 0:
        fileName = os.path.join(dxfDir,'rect_extent_test1.dxf')
        param = {
                'fileName'       : fileName,
                'components'     : True,
                'depth'          : 2*0.04,
                'startZ'         : 0.0,
                'safeZ'          : 0.5,
                'overlap'        : 0.3,
                'overlapFinish'  : 0.5,
                'maxCutDepth'    : 0.04,
                'toolDiam'       : 0.25,
                'cornerCut'      : False,
                'direction'      : 'ccw',
                'startDwell'     : 2.0,
                }
        pocket = DxfRectPocketFromExtent(param)
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



    if 0:
        #fileName = os.path.join(dxfDir,'boundary_test0.dxf')
        fileName = os.path.join(dxfDir,'boundary_test1.dxf')
        #fileName = os.path.join(dxfDir,'boundary_test2.dxf')
        #fileName = os.path.join(dxfDir,'boundary_test3.dxf')
        #fileName = os.path.join(dxfDir,'boundary_test4.dxf')
        param = {
                'fileName'       : fileName,
                'depth'       : 0.03,
                'startZ'      : 0.0,
                'safeZ'       : 0.2,
                'toolDiam'    : 0.25,
                'direction'   : 'ccw',
                'cutterComp'  : None,
                'maxCutDepth' : 0.03,
                'startDwell'  : 2.0,
                'startCond'   : 'minX',
                }
        boundary = DxfBoundary(param)
        prog.add(boundary)

    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')
