import math
import numpy as np

def clusters(points,threshold=.35):
    result=[]; group=[]
    for point in points:
        if point is None:
            if len(group)>=3: result.append(group)
            group=[]; continue
        if group and math.dist(group[-1],point)>threshold:
            if len(group)>=3: result.append(group)
            group=[]
        group.append(point)
    if len(group)>=3: result.append(group)
    if len(result)>1 and math.dist(result[0][0],result[-1][-1])<threshold:
        result[0]=result[-1]+result[0]; result.pop()
    return [g for g in result if max(math.dist(g[0],p) for p in g)<1.8]


def observed_shape(group):
    points=np.asarray(group); mean=points.mean(axis=0); centered=points-mean
    matrix=np.column_stack((2*centered,np.ones(len(group))))
    if len(group)>=6 and np.linalg.cond(matrix)<100:
        fit=np.linalg.lstsq(matrix,np.sum(centered*centered,axis=1),rcond=None)[0]
        radius=math.sqrt(max(0,fit[2]+fit[0]*fit[0]+fit[1]*fit[1])); center=mean+fit[:2]
        error=np.sqrt(np.mean((np.linalg.norm(points-center,axis=1)-radius)**2))
        if .15<radius<.5 and error<.015: return (float(center[0]),float(center[1])),float(radius),'circle_fit'
    radius=min(.9,max(.35,float(np.max(np.linalg.norm(centered,axis=1)))+.35))
    return (float(mean[0]),float(mean[1])),radius,'partial_surface_bound'


def observed_model(group,sensor,long_side=1.0,short_side=.7):
    center,radius,kind=observed_shape(group)
    if kind=='circle_fit': return center,radius,kind,0.0,0.0,0.0
    points=np.asarray(group)
    # Minimize distance to rectangle boundaries, rather than averaging noisy
    # adjacent-ray directions. The latter is unstable on a close flat face.
    def residual(candidate):
        c=math.cos(candidate); s=math.sin(candidate)
        u=c*points[:,0]+s*points[:,1]; v=-s*points[:,0]+c*points[:,1]
        du=np.minimum(u-u.min(),u.max()-u); dv=np.minimum(v-v.min(),v.max()-v)
        return float(np.mean(np.minimum(du,dv)))
    angle=min((math.radians(a) for a in range(90)),key=residual)
    angle=min((angle+math.radians(a/10) for a in range(-10,11)),key=residual)
    axis=np.array([math.cos(angle),math.sin(angle)]); normal=np.array([-axis[1],axis[0]])
    along=points@axis; across=points@normal
    if np.ptp(along)<np.ptp(across):
        axis,normal=normal,-axis; along=points@axis; across=points@normal
    visible=long_side if np.ptp(along)>(long_side+short_side)/2 else short_side
    depth=short_side if visible==long_side else long_side
    midpoint=axis*(along.min()+along.max())/2
    if np.ptp(across)>.8*depth:
        midpoint+=normal*(across.min()+across.max())/2
    else:
        face=float(np.median(across)); away=1 if np.dot(normal,np.asarray(center)-np.asarray(sensor))>=0 else -1
        midpoint+=normal*(face+away*depth/2)
    return (float(midpoint[0]),float(midpoint[1])),math.hypot(visible/2,depth/2),'box_prior',visible/2,depth/2,math.atan2(axis[1],axis[0])
