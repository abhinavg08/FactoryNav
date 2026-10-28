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
