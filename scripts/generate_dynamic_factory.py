#!/usr/bin/env python3
"""Generate an additional factory; never modify the original factory/map."""
from collections import deque
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml
from generate_factory import box, child

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'factorynav_bringup'


def connected(free, start, goal, origin, resolution):
    def cell(p): return (int((p[1]-origin[1])/resolution), int((p[0]-origin[0])/resolution))
    begin, end = cell(start), cell(goal)
    queue = deque([begin]); seen = {begin}
    while queue:
        row, col = queue.popleft()
        if (row,col) == end: return True
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
            q=(row+dr,col+dc)
            if 0 <= q[0] < free.shape[0] and 0 <= q[1] < free.shape[1] and free[q] and q not in seen:
                seen.add(q); queue.append(q)
    return False


def main():
    scene_path=PACKAGE/'config/dynamic_scene.json'
    scene=json.loads(scene_path.read_text())
    tree=ET.parse(PACKAGE/'worlds/factory.sdf'); world=tree.getroot().find('world')
    for model in list(world.findall('model')): world.remove(model)
    box(world, {'name':'factory_floor','x':0,'y':0,'width':25,'depth':20,'height':0.1,'color':[0.72,0.75,0.76,1]})
    world.find("model[@name='factory_floor']/pose").text='0 0 -0.05 0 0 0'
    child(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    world.find('gui/plugin/camera_pose').text='-17 -18 21 0 0.7 0.8'
    for item in scene['obstacles']: box(world,item)
    for station in scene['stations']:
        box(world,{**station,'width':1.2,'depth':1.2},collision=False,height=0.004)
    for yy in (-1.8,1.8):
        box(world,{'name':f'walking_zone_{yy}','x':0,'y':yy,'width':19,'depth':0.04,'color':[0.95,0.85,0.2,1]},False,height=0.002)
    ET.indent(tree,space='  ')
    world_path=PACKAGE/'worlds/dynamic_factory.sdf'
    world_path.write_bytes(ET.tostring(tree.getroot(),encoding='utf-8',xml_declaration=True)+b'\n')
    res=scene['resolution']; ox,oy=scene['origin']; w,h=[round(v/res) for v in scene['size']]
    xx,yy=np.meshgrid(ox+(np.arange(w)+.5)*res,oy+(np.arange(h)+.5)*res)
    image=np.full((h,w),205,np.uint8)
    image[(abs(xx)<scene['bounds'][0])&(abs(yy)<scene['bounds'][1])]=254
    for item in scene['obstacles']:
        image[(abs(xx-item['x'])<=item['width']/2)&(abs(yy-item['y'])<=item['depth']/2)]=0
    map_path=PACKAGE/'maps/dynamic_factory.pgm'
    map_path.write_bytes(f'P5\n{w} {h}\n255\n'.encode()+image[::-1].tobytes())
    (PACKAGE/'maps/dynamic_factory.yaml').write_text(yaml.safe_dump({'image':'dynamic_factory.pgm','mode':'trinary','resolution':res,'origin':[ox,oy,0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
    # A conservative 0.31 m disk contains the entire configured rectangular footprint.
    clearance=distance_transform_edt(image==254)*res
    free=clearance>0.31
    stations={s['name']:(s['x'],s['y']) for s in scene['stations']}
    checks={name:connected(free,stations['loading'],point,scene['origin'],res) for name,point in stations.items()}
    alternate=free.copy(); alternate[(xx>-9.3)&(xx<-6.3)&(abs(yy)<.5)]=False
    report={'static_map_checks':checks,'loading_assembly_alternative_after_outer_corridor_block':connected(alternate,stations['loading'],stations['assembly'],scene['origin'],res),'conservative_connectivity_radius_m':.31,'note':'Geometric checks, not executed navigation results.'}
    if not all(checks.values()) or not report['loading_assembly_alternative_after_outer_corridor_block']:
        raise RuntimeError(report)
    report['sha256']={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in (scene_path,world_path,map_path)}
    (ROOT/'docs/dynamic_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(13,10))
    ax.imshow(image,origin='lower',extent=[ox,ox+w*res,oy,oy+h*res],cmap='gray',vmin=0,vmax=255)
    for s in scene['stations']:
        ax.scatter(s['x'],s['y'],s=110,color=s['color'][:3],edgecolors='black')
        ax.annotate(s['label'],(s['x'],s['y']),xytext=(0,12),textcoords='offset points',ha='center',fontsize=10,bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
    for route,color,label in [([(-8,-6.5),(8,-6.5)],'#15803d','Southern alternative'),([(-8,-6.5),(-8,0),(8,0),(9.3,-2.5),(9.3,-6.5),(8,-6.5)],'#dc2626','Main corridor alternative')]:
        ax.plot(*zip(*route),'--',color=color,label=label,alpha=.7)
    ax.annotate('0.95 m dead-end bay',(-9.92,3.2),(-6.5,1),arrowprops={'arrowstyle':'->'},bbox={'facecolor':'white','alpha':.85})
    ax.set(xlabel='World x (m)',ylabel='World y (m)',title='Dynamic Factory — static geometry and geometric route alternatives')
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.065),ncol=2); ax.set_aspect('equal'); fig.tight_layout()
    (ROOT/'media').mkdir(exist_ok=True); fig.savefig(ROOT/'media/dynamic_factory_map.png',dpi=150); plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
