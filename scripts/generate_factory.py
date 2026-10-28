#!/usr/bin/env python3
"""Generate collision geometry and its occupancy map from one scene definition."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'factorynav_bringup'


def child(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def geometry(parent, width, depth, height):
    child(child(child(parent, 'geometry'), 'box'), 'size', f'{width} {depth} {height}')


def box(world, item, collision=True, height=None):
    height = item['height'] if height is None else height
    model = child(world, 'model', name=item['name'])
    child(model, 'static', 'true')
    child(model, 'pose', f"{item['x']} {item['y']} {height / 2} 0 0 0")
    link = child(model, 'link', name='body')
    if collision:
        geometry(child(link, 'collision', name='collision'), item['width'], item['depth'], height)
    visual = child(link, 'visual', name='visual')
    geometry(visual, item['width'], item['depth'], height)
    material = child(visual, 'material')
    color = ' '.join(map(str, item['color']))
    child(material, 'ambient', color)
    child(material, 'diffuse', color)
    if 'storage_rack' in item['name']:
        for level in (0.25, 0.65, 1.05):
            stripe = child(link, 'visual', name=f'shelf_{level}')
            child(stripe, 'pose', f'0 0 {level - height / 2} 0 0 0')
            geometry(stripe, item['width'] + 0.02, item['depth'] + 0.02, 0.045)
            shelf_material = child(stripe, 'material')
            child(shelf_material, 'ambient', '0.95 0.65 0.1 1')
            child(shelf_material, 'diffuse', '0.95 0.65 0.1 1')


def main():
    scene_path = PACKAGE / 'config' / 'factory_scene.json'
    scene = json.loads(scene_path.read_text(encoding='utf-8'))
    sdf = ET.Element('sdf', version='1.9')
    world = child(sdf, 'world', name='default')
    for library, name in (
        ('physics', 'Physics'), ('user-commands', 'UserCommands'),
        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')
    ):
        plugin = child(world, 'plugin', filename=f'gz-sim-{library}-system',
                       name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            child(plugin, 'render_engine', 'ogre2')
    physics = child(world, 'physics', name='factory_physics', type='ode')
    child(physics, 'max_step_size', '0.003')
    child(physics, 'real_time_factor', '1')
    light = child(world, 'light', name='factory_sun', type='directional')
    child(light, 'pose', '0 0 10 0 0 0')
    child(light, 'diffuse', '0.9 0.9 0.9 1')
    child(light, 'specular', '0.2 0.2 0.2 1')
    child(light, 'direction', '-0.3 -0.2 -1')
    child(light, 'cast_shadows', 'false')
    attenuation = child(light, 'attenuation')
    child(attenuation, 'range', '1000')
    child(attenuation, 'constant', '0.9')
    child(attenuation, 'linear', '0.01')
    child(attenuation, 'quadratic', '0.001')
    render_scene = child(world, 'scene')
    child(render_scene, 'ambient', '0.7 0.7 0.7 1')
    child(render_scene, 'background', '0.12 0.15 0.19 1')
    child(render_scene, 'shadows', 'false')
    gui = child(world, 'gui', fullscreen='false')
    view = child(gui, 'plugin', filename='MinimalScene', name='3D View')
    view_options = child(view, 'gz-gui')
    child(view_options, 'property', 'false', type='bool', key='showTitleBar')
    child(view_options, 'property', 'docked', type='string', key='state')
    child(view, 'engine', 'ogre2')
    child(view, 'scene', 'scene')
    child(view, 'ambient_light', '0.7 0.7 0.7')
    child(view, 'background_color', '0.12 0.15 0.19')
    child(view, 'camera_pose', '-10 -10 12 0 0.7 0.7854')
    for filename, name in [('GzSceneManager', 'Scene Manager'),
                           ('InteractiveViewControl', 'Interactive view control')]:
        options = child(child(gui, 'plugin', filename=filename, name=name), 'gz-gui')
        for key, kind, value in [('width', 'double', '5'), ('height', 'double', '5'),
                                 ('showTitleBar', 'bool', 'false'), ('state', 'string', 'floating')]:
            child(options, 'property', value, type=kind, key=key)
    controls = child(gui, 'plugin', filename='WorldControl', name='World control')
    options = child(controls, 'gz-gui')
    child(options, 'property', 'false', type='bool', key='showTitleBar')
    child(options, 'property', 'floating', type='string', key='state')
    anchors = child(options, 'anchors', target='3D View')
    child(anchors, 'line', own='left', target='left')
    child(anchors, 'line', own='bottom', target='bottom')
    child(controls, 'play_pause', 'true')
    child(controls, 'start_paused', 'false')
    child(controls, 'use_event', 'true')
    box(world, {'name': 'factory_floor', 'x': 0, 'y': 0, 'width': 20,
                'depth': 20, 'height': 0.1, 'color': [0.72, 0.75, 0.76, 1]})
    # The top of the floor must be z=0 so the upstream robot spawn height works.
    world.find("model[@name='factory_floor']/pose").text = '0 0 -0.05 0 0 0'
    for item in scene['obstacles']:
        box(world, item)
    for station in scene['stations']:
        box(world, {**station, 'width': 1.2, 'depth': 1.2}, collision=False, height=0.004)
    for lane_y in (-0.8, 0.8):
        box(world, {'name': f'aisle_marking_{lane_y}', 'x': 0, 'y': lane_y,
                    'width': 10.3, 'depth': 0.035, 'color': [0.95, 0.85, 0.18, 1]},
            collision=False, height=0.002)
    world_path = PACKAGE / 'worlds' / 'factory.sdf'
    world_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf, space='  ')
    world_path.write_bytes(ET.tostring(sdf, encoding='utf-8', xml_declaration=True) + b'\n')

    resolution = scene['resolution']
    origin_x, origin_y = scene['map_origin']
    width, height = [round(value / resolution) for value in scene['map_size']]
    xx, yy = np.meshgrid(origin_x + (np.arange(width) + 0.5) * resolution,
                         origin_y + (np.arange(height) + 0.5) * resolution)
    image = np.full((height, width), 205, dtype=np.uint8)
    image[(np.abs(xx) < 6) & (np.abs(yy) < 5)] = 254
    for item in scene['obstacles']:
        occupied = ((np.abs(xx - item['x']) <= item['width'] / 2) &
                    (np.abs(yy - item['y']) <= item['depth'] / 2))
        image[occupied] = 0
    maps = PACKAGE / 'maps'
    maps.mkdir(parents=True, exist_ok=True)
    map_path = maps / 'factory.pgm'
    map_path.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + image[::-1].tobytes())
    (maps / 'factory.yaml').write_text(yaml.safe_dump({
        'image': 'factory.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8', newline='\n')
    manifest = {path.relative_to(ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (scene_path, world_path, map_path)}
    (ROOT / 'docs' / 'geometry_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(f'Generated {world_path} and matching {width}x{height} occupancy map')


if __name__ == '__main__':
    main()
