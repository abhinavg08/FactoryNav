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
    points=np.asarray(group); differences=np.diff(points,axis=0)
    differences=differences[np.linalg.norm(differences,axis=1)>.008]
    if len(differences):
        angles=np.arctan2(differences[:,1],differences[:,0]); angle=.25*math.atan2(np.sin(4*angles).mean(),np.cos(4*angles).mean())
    else: angle=0.0
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
