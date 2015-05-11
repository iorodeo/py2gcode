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
import cnc_path
import math
import gcode_cmd
import dxfgrabber
import networkx
import shapely.geometry.polygon as polygon

from graph_utils import getEntityGraph
from dxf_utils import getEntityStartAndEndPts

class LaserCutBase(gcode_cmd.GCodeProg): 

    DEFAULT_PARAM = {}
    LASER_DIO_PIN = 1
    LASER_HOME_XY = (35,23)

    def __init__(self,param):
        self.param = dict(self.DEFAULT_PARAM)
        self.param['laserDIOPin'] = self.LASER_DIO_PIN
        self.param['laserHomeXY'] = self.LASER_HOME_XY
        self.param.update(param)
        try:
            self.dwg = self.param['dwg']
        except KeyError:
            self.dwg = dxfgrabber.readfile(self.param['fileName'])
        self.makeListOfCmds()

    def addStartComment(self):
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = 'Begin {0}'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        for k,v in self.param.iteritems():
            vStr = str(v)
            if len(vStr) > 50:
                vStr = 'too big' 
            self.listOfCmds.append(gcode_cmd.Comment('{0}: {1}'.format(k,vStr)))

    def addEndComment(self):
        self.listOfCmds.append(gcode_cmd.Space())
        commentStr = 'End {0}'.format(self.__class__.__name__)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.Comment('-'*60))
        self.listOfCmds.append(gcode_cmd.Space())

    def addRapidMoveToPos(self,**kwarg):
        self.listOfCmds.append(gcode_cmd.Space())
        try:
            pointName = kwarg.pop('comment')
        except KeyError:
            pointName = '{0}'.format(kwarg)
        commentStr = '{0}: rapid move to {1}'.format(self.__class__.__name__, pointName)
        self.listOfCmds.append(gcode_cmd.Comment(commentStr))
        self.listOfCmds.append(gcode_cmd.RapidMotion(**kwarg))

    def addRapidMoveToHome(self):
        homeX = self.param['laserHomeXY'][0]
        homeY = self.param['laserHomeXY'][1]
        self.addRapidMoveToPos(x=homeX, y=homeY)

    def addLaserSetup(self):
        self.listOfCmds.extend(self.getLaserSetupCmds())

    def addLaserShutdown(self):
        self.listOfCmds.extend(self.getLaserShutdownCmds())

    def addLaserOn(self,synchronized=False):
        self.listOfCmds.extend(self.getLaserOnCmds(synchronized=synchronized))

    def addLaserOff(self,synchronized=False):
        self.listOfCmds.extend(self.getLaserOffCmds(synchronized=synchronized))

    def getLaserSetupCmds(self):
        listOfCmds = []
        listOfCmds.append(gcode_cmd.Space())
        listOfCmds.append(gcode_cmd.Comment('Setup laser'))
        listOfCmds.append(gcode_cmd.FeedRate(self.param['feedRate']))
        listOfCmds.extend(self.getLaserOffCmds(comment=False))
        listOfCmds.append(gcode_cmd.SpindleSpeed(self.param['laserPower']))
        listOfCmds.append(gcode_cmd.StartSpindleCW())
        return listOfCmds

    def getLaserShutdownCmds(self):
        listOfCmds = []
        listOfCmds.append(gcode_cmd.Space())
        listOfCmds.append(gcode_cmd.Comment('Shutdown laser'))
        listOfCmds.extend(self.getLaserOffCmds(comment=False))
        listOfCmds.append(gcode_cmd.StopSpindle())
        listOfCmds.append(gcode_cmd.SpindleSpeed(0))
        return listOfCmds

    def getLaserOnCmds(self,synchronized=False,comment=True):
        listOfCmds = []
        if comment:
            listOfCmds.append(gcode_cmd.Space())
            listOfCmds.append(gcode_cmd.Comment('Laser on'))
        listOfCmds.append(gcode_cmd.DigitalOutput(self.param['laserDIOPin'], 1, synchronized))
        return listOfCmds

    def getLaserOffCmds(self,synchronized=False,comment=True):
        listOfCmds = []
        if comment:
            listOfCmds.append(gcode_cmd.Space())
            listOfCmds.append(gcode_cmd.Comment('Laser off'))
        listOfCmds.append(gcode_cmd.DigitalOutput(self.param['laserDIOPin'], 0, synchronized))
        return listOfCmds


    @property
    def layerNameList(self):
        try:
            layerNameList = self.param['layers']
        except KeyError:
            layerNameList = [layer.name for layer in self.dwg.layers]
        return layerNameList

    @property
    def entityList(self):
        print('1')
        entityList = [x for x in self.dwg.entities if x.layer in self.layerNameList]
        print('2')
        entityList = [x for x in entityList if x.dxftype in self.param['dxfTypes']] 
        print('3')
        entityList = [x for x in entityList if x.dxftype in self.ALLOWED_TYPE_LIST]
        print('4')
        return entityList


class VectorCut(LaserCutBase):

    ALLOWED_TYPE_LIST = ['LINE', 'ARC']
    DEFAULT_PARAM = {
            'dxfTypes'    :  ['LINE'],
            'laserPower'  :  300,
            'feedRate'    :  20,
            'convertArcs' :  True,
            'ptEquivTol'  :  1.0e-5,
            'maxArcLen'   :  1.0e-2,
            'startCond'   : 'minX',
            }


    def __init__(self,param):
        super(VectorCut,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []

        self.addLaserSetup()

        # Get entity graph and find connected components
        print('Getting entity graph')
        graph, ptToNodeDict = getEntityGraph(self.entityList,self.param['ptEquivTol'])
        print('Finding connected components')
        connectedCompSubGraphs = networkx.connected_component_subgraphs(graph)
        # Create list of commands for each connected component individually
        for i, subGraph in enumerate(connectedCompSubGraphs):
            print('subGraph: {0}'.format(i))
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

        self.addLaserShutdown()
        self.addRapidMoveToHome()
            

    def makeCmdsForLineString(self,graph):
        print('makeCmdsForLineString')

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
        print('makeCmdsForClosedLoop')
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

        # Test for self instersections and if none orient closed loop for cutting direction
        lineString = polygon.LineString(closedPathCoord)
        if lineString.is_simple:
            linearRing = polygon.LinearRing(closedPathCoord)
            cwTest = self.param['direction'] == 'cw' and linearRing.is_ccw
            ccwTest = self.param['direction'] == 'ccw' and not linearRing.is_ccw
            if cwTest or ccwTest:
                closedPath.reverse()
                closedPathCoord.reverse()

        # Get list of segments (line or arc) along path and create cnc commands
        segList = self.getSegListFromPath(closedPath,graph)
        param = dict(self.param)
        param['closed'] = True 
        listOfCmds = self.makeListOfCmdsFromSegList(segList,param)
        return listOfCmds

    def makeListOfCmdsFromSegList(self,segList,param):
        listOfCmds = []
        if self.param['convertArcs']:
            pointList = [p[0] for p in segList]
            pointList.append(segList[-1][1])
            param['pointList'] = pointList 
            path = LaserLineSegPath(param)
            listOfCmds = path.listOfCmds
        else:
            raise RuntimeError('convertArcs=False not supported yet')
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

        
class LaserLineSegPath(LaserCutBase):

    DEFAULT_PARAM = {'ptEquivTol'  :  1.0e-5}
    
    def __init__(self,param):
        super(LaserLineSegPath,self).__init__(param)

    def makeListOfCmds(self):
        self.listOfCmds = []

        lineSegPath = cnc_path.LineSegPath(
                self.param['pointList'],
                closed=self.param['closed'],
                plane='xy',
                helix=None
                )

        # Routine begin - move to safe height, then to start x,y and then to start z
        self.addStartComment()

        x0, y0 = lineSegPath.getStartPoint()[:2]
        self.addRapidMoveToPos(x=x0,y=y0,comment='start x,y')

        self.listOfCmds.append((gcode_cmd.PathBlendMode(p=0.001,q=0.001)))
        self.addLaserOn(synchronized=True)
        self.listOfCmds.extend(lineSegPath.listOfCmds) 
        self.addLaserOff()
        self.listOfCmds.append(gcode_cmd.ExactPathMode())

        self.addEndComment()



# --------------------------------------------------------------------------------
if __name__ == '__main__':

    import os

    prog = gcode_cmd.GCodeProg()
    prog.add(gcode_cmd.GenericStart())
    prog.add(gcode_cmd.Space())

    dxfDir = os.path.join(os.curdir,'test_dxf')

    fileName = os.path.join(dxfDir, '3mm_black_colorimeter_array.dxf')
    param = {
            'fileName'    :  fileName,
            'layers'      :  ['vector'],
            'dxfTypes'    :  ['LINE'],
            'laserPower'  :  690,
            'feedRate'    :  8,
            'convertArcs' :  True,
            'startCond'   : 'minX',
            'direction'   : 'ccw',
            'ptEquivTol'  :  0.4e-3,
            }

    vectorCut = VectorCut(param)
    prog.add(vectorCut)

    prog.add(gcode_cmd.Space())
    prog.add(gcode_cmd.End(),comment=True)
    print(prog)
    prog.write('test.ngc')





