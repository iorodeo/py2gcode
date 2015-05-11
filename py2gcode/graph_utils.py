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
import networkx
import dxf_utils
import geom_utils

def getEntityGraph(entityList, ptEquivTol=1.0e-6):
    print(' create point to node dict')
    ptToNodeDict = getPtToNodeDict(entityList,ptEquivTol)
    print(' create graph')
    graph = networkx.Graph()
    for entity in entityList:
        startPt, endPt = dxf_utils.getEntityStartAndEndPts(entity)
        startNode = ptToNodeDict[startPt]
        graph.add_node(startNode,coord=startPt)
        endNode = ptToNodeDict[endPt]
        graph.add_node(endNode,coord=endPt)
        graph.add_edge(startNode, endNode, entity=entity)
    for edge in graph.edges():
        # Remove any trivial edges - perhaps due to drawing errors?
        if edge[0] == edge[1]:
            graph.remove_edge(*edge)
    return graph, ptToNodeDict

def getPtToNodeDict(entityList, ptEquivTol=1.0e-6):
    ptList = []
    for entity in entityList:
        startPt, endPt = dxf_utils.getEntityStartAndEndPts(entity)
        ptList.extend([startPt, endPt])
    ptToNodeDict = {}
    nodeCnt = 0
    for i, p in enumerate(ptList):
        print(' {0}/{1}'.format(i,len(ptList)))
        found = False
        for q in ptList[:i]:
            if geom_utils.dist2D(p,q) < ptEquivTol:
                found = True
                ptToNodeDict[p] = ptToNodeDict[q] 
                break
        if not found:
            ptToNodeDict[p] = nodeCnt
            nodeCnt += 1
    return ptToNodeDict
