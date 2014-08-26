import math

def dist2D(p,q):
    return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

def midPoint2D(p,q): 
    return 0.5*(p[0] + q[0]), 0.5*(p[1] + q[1])

