from __future__ import print_function
import math

def lineSegmentIntersect(segA, segB, tol=1e-9):
    """
    Tests whether or not two line segments intersect.
    """
    if not boundingBoxIntersect(segA,segB):
        return False
    test0 = pointOnLine(segB[0], segA, tol=tol)
    test1 = pointOnLine(segB[1], segA, tol=tol)
    test2 = pointRightOfLine(segB[0], segA)
    test3 = pointRightOfLine(segB[1], segA)
    return test0 or test1 or (test2 ^ test3)
    
def boundingBoxIntersect(segA, segB):
    """
    Checks if line segment bounding boxes intersect
    """
    xMinA = min([segA[0][0], segA[1][0]])
    xMaxA = max([segA[0][0], segA[1][0]])
    yMinA = min([segA[0][1], segA[1][1]])
    yMaxA = max([segA[0][1], segA[1][1]])
    xMinB = min([segB[0][0], segB[1][0]])
    xMaxB = max([segB[0][0], segB[1][0]])
    yMinB = min([segB[0][1], segB[1][1]])
    yMaxB = max([segB[0][1], segB[1][1]])
    test0 = xMinA <= xMaxB
    test1 = xMaxA >= xMinB
    test2 = yMinA <= yMaxB
    test3 = yMaxA >= yMinB
    return test0 and test1 and test2 and test3

def pointOnLine(pt, seg, tol=1e-9):
    """
    Tests whether or not point is on line segment
    """
    p = seg[1][0] - seg[0][0], seg[1][1] - seg[0][1]
    q = pt[0] - seg[0][0], pt[1] - seg[0][1]
    r = crossProd(p,q)
    if r < tol:
        return True
    else:
        return False

def pointRightOfLine(pt, seg):
    """
    Tests whether or not point is to the right of the line
    """
    p = seg[1][0] - seg[0][0], seg[1][1] - seg[0][1]
    q = pt[0] - seg[0][0], pt[1] - seg[0][1]
    r = crossProd(p,q)
    if r < 0:
        True
    else:
        False
            
def crossProd(v,w):
    """
    Calculates cross product of two vectors
    """
    return v[0]*w[1] - w[0]*v[1]

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import pylab

    segA = ((1,1), (2,2))
    segB = ((0,0), (1.5,1.50001))
    test= lineSegmentIntersect(segA,segB)
    print(test)
    pylab.plot([segA[0][0], segA[1][0]], [segA[0][1], segA[1][1]],'b')
    pylab.plot([segB[0][0], segB[1][0]], [segB[0][1], segB[1][1]],'r')
    pylab.show()







    




