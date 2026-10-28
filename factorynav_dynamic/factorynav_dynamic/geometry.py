"""Conservative footprint clearances in metres, independent of LiDAR range."""
import math

ROBOT_BOUNDS=(-.22,.16,-.20,.20)


def local(point, pose):
    dx=point[0]-pose[0]; dy=point[1]-pose[1]; c=math.cos(pose[2]); s=math.sin(pose[2])
    return c*dx+s*dy,-s*dx+c*dy


def point_box(point,bounds):
    x,y=point; xmin,xmax,ymin,ymax=bounds
    dx=max(xmin-x,0,x-xmax); dy=max(ymin-y,0,y-ymax)
    if dx or dy: return math.hypot(dx,dy)
    return -min(x-xmin,xmax-x,y-ymin,ymax-y)


def circle_clearance(robot, center, radius):
    return point_box(local(center,robot),ROBOT_BOUNDS)-radius


def vertices(pose,bounds):
    xmin,xmax,ymin,ymax=bounds; c=math.cos(pose[2]); s=math.sin(pose[2])
    return [(pose[0]+c*x-s*y,pose[1]+s*x+c*y) for x,y in [(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)]]


def segment_distance(point,a,b):
    vx=b[0]-a[0]; vy=b[1]-a[1]; denominator=vx*vx+vy*vy
    t=max(0,min(1,((point[0]-a[0])*vx+(point[1]-a[1])*vy)/denominator)) if denominator else 0
    return math.hypot(point[0]-a[0]-t*vx,point[1]-a[1]-t*vy)


def box_clearance(robot, other, bounds):
    first=vertices(robot,ROBOT_BOUNDS); second=vertices(other,bounds); overlap=True; penetration=math.inf
    for polygon in (first,second):
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            axis=(-(b[1]-a[1]),b[0]-a[0]); norm=math.hypot(*axis); axis=(axis[0]/norm,axis[1]/norm)
            one=[p[0]*axis[0]+p[1]*axis[1] for p in first]; two=[p[0]*axis[0]+p[1]*axis[1] for p in second]
            depth=min(max(one),max(two))-max(min(one),min(two))
            if depth<0: overlap=False
            penetration=min(penetration,depth)
    if overlap: return -penetration
    return min(segment_distance(p,a,b) for points,polygon in ((first,second),(second,first)) for p in points for a,b in zip(polygon,polygon[1:]+polygon[:1]))
