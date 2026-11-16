#!/usr/bin/python3
"""Extrude a ROS trinary occupancy map into exact, merged collision boxes."""
import argparse
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
def tag(parent, element_name, value=None, **attrs):
    e = E.SubElement(parent, element_name, attrs)
    if value is not None:
        e.text = str(value)
    return e

def box(parent, name, pose, size, color='0.55 0.6 0.65 1'):
    link = tag(parent, 'link', name=name)
    tag(link, 'pose', pose)
    for kind in ('collision', 'visual'):
        part = tag(link, kind, name=kind)
        tag(tag(tag(part, 'geometry'), 'box'), 'size', size)
        if kind == 'visual':
            mat = tag(part, 'material')
            tag(mat, 'ambient', color)
            tag(mat, 'diffuse', color)
    return link

def rectangles(mask):
    """Merge equal horizontal runs across consecutive rows, without filling gaps."""
    active = {}
    for row in range(mask.shape[0] + 1):
        runs = []
        if row < mask.shape[0]:
            edges = np.diff(np.r_[False, mask[row], False].astype(np.int8))
            runs = list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))
        current = set(runs)
        for run in list(active):
            if run not in current:
                yield (*run, active.pop(run), row)
        for run in runs:
            active.setdefault(run, row)

def build(map_path, output, height, spawn, robot_path=None):
    cfg = yaml.safe_load(map_path.read_text())
    if cfg.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary maps are supported; convert scale/raw maps first.')
    res = float(cfg['resolution'])
    if res <= 0 or height <= 0:
        raise ValueError('resolution and height must be positive')
    ox, oy, yaw = map(float, cfg['origin'])
    pixels = np.asarray(Image.open(map_path.parent / cfg['image']).convert('L'))
    p = pixels / 255.0 if cfg.get('negate', 0) else 1.0 - pixels / 255.0
    occupied = p > float(cfg['occupied_thresh'])
    free = p < float(cfg['free_thresh'])
    # Check a conservative circumscribed disk for the 1.2 x 0.9 m vehicle.
    sx, sy, heading = spawn
    dx, dy = sx - ox, sy - oy
    lx, ly = math.cos(yaw)*dx + math.sin(yaw)*dy, -math.sin(yaw)*dx + math.cos(yaw)*dy
    col, row = int(math.floor(lx/res)), pixels.shape[0]-1-int(math.floor(ly/res))
    margin = int(math.ceil(0.80/res))
    if (row-margin < 0 or col-margin < 0 or row+margin >= free.shape[0]
            or col+margin >= free.shape[1]
            or not free[row-margin:row+margin+1, col-margin:col+margin+1].all()):
        raise ValueError('Spawn lacks 0.80 m known-free clearance. Pass --spawn X Y YAW at a clear location.')
    sdf = E.Element('sdf', version='1.8')
    world = tag(sdf, 'world', name='patrol')
    physics = tag(world, 'physics', name='physics', type='ignored')
    tag(physics, 'max_step_size', '0.002')
    tag(physics, 'real_time_factor', '1')
    for suffix, cls in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = tag(world, 'plugin', filename=f'ignition-gazebo-{suffix}-system', name=f'ignition::gazebo::systems::{cls}')
        if cls == 'Sensors':
            tag(plugin, 'render_engine', 'ogre2')
    light = tag(world, 'light', name='sun', type='directional')
    tag(light, 'pose', '0 0 10 0 0 0')
    tag(light, 'diffuse', '0.9 0.9 0.9 1')
    tag(light, 'direction', '-0.3 0.2 -1')
    ground = tag(world, 'model', name='ground')
    tag(ground, 'static', 'true')
    tag(ground, 'pose', f'{ox} {oy} 0 0 0 {yaw}')
    width, depth = pixels.shape[1]*res, pixels.shape[0]*res
    box(ground, 'floor', f'{width/2} {depth/2} -0.1 0 0 0', f'{width+4} {depth+4} 0.2', '0.8 0.8 0.75 1')
    walls = tag(world, 'model', name='map_obstacles')
    tag(walls, 'static', 'true')
    tag(walls, 'pose', f'{ox} {oy} 0 0 0 {yaw}')
    geometry_link = tag(walls, 'link', name='map_geometry')
    count = 0
    for x0, x1, r0, r1 in rectangles(occupied):
        pose = f'{(x0+x1)*res/2} {(pixels.shape[0]-(r0+r1)/2)*res} {height/2} 0 0 0'
        temporary = E.Element('temporary')
        wall = box(temporary, f'wall_{count}', pose, f'{(x1-x0)*res} {(r1-r0)*res} {height}')
        # One static link avoids thousands of unnecessary rigid bodies.
        for kind in ('collision', 'visual'):
            part = wall.find(kind)
            part.set('name', f'{kind}_{count}')
            tag(part, 'pose', pose)
            geometry_link.append(part)
        count += 1
    output.parent.mkdir(parents=True, exist_ok=True)
    if count > 100:
        # Bake static boxes into one triangle mesh to reduce physics/render overhead.
        mesh_path = output.with_suffix('.obj')
        with mesh_path.open('w') as mesh:
            mesh.write('# Occupied map cells, extruded and merged; units: metres\n')
            mesh.write('vn 0 0 -1\nvn 0 0 1\nvn 0 -1 0\nvn 1 0 0\nvn 0 1 0\nvn -1 0 0\n')
            vertex = 1
            for collision in geometry_link.findall('collision'):
                cx, cy, cz = map(float, collision.find('pose').text.split()[:3])
                bx, by, bz = map(float, collision.find('geometry/box/size').text.split())
                for vx, vy, vz in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                   (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]:
                    mesh.write(f'v {cx+vx*bx/2} {cy+vy*by/2} {cz+vz*bz/2}\n')
                for face_index, face in enumerate([(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
                             (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]):
                    mesh.write('f '+' '.join(f'{vertex+i}//{face_index//2+1}' for i in face)+'\n')
                vertex += 8
        geometry_link.clear()
        geometry_link.set('name', 'map_geometry')
        for kind in ('collision', 'visual'):
            part = tag(geometry_link, kind, name=f'map_{kind}')
            tag(tag(tag(part, 'geometry'), 'mesh'), 'uri', mesh_path.name)
            if kind == 'visual':
                material = tag(part, 'material')
                tag(material, 'ambient', '0.55 0.6 0.65 1')
                tag(material, 'diffuse', '0.55 0.6 0.65 1')
    robot_path = Path(robot_path or ROOT/'models/patrol_car.sdf').resolve()
    robot = E.parse(robot_path).getroot().find('model')
    # Rebase relative model resources from models/ to the generated world's
    # directory.  Absolute file:// paths made the previous worlds machine-only.
    for uri in robot.findall('.//uri'):
        value = (uri.text or '').strip()
        if value and '://' not in value:
            source = (robot_path.parent / value).resolve()
            uri.text = os.path.relpath(source, output.parent)
    robot.find('pose').text = f'{sx} {sy} 0 0 0 {heading}'
    world.append(robot)
    output.parent.mkdir(parents=True, exist_ok=True)
    E.indent(sdf)
    E.ElementTree(sdf).write(output, encoding='utf-8', xml_declaration=True)
    metadata = dict(map=os.path.relpath(map_path.resolve(), output.parent), world=output.name, spawn=list(spawn),
                    obstacle_boxes=count, resolution=res, wall_height=height,
                    note='2D extrusion; unknown cells are not reconstructed; not measured elevation.')
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--height', type=float, default=1.5)
    parser.add_argument('--spawn', type=float, nargs=3, default=[0, 0, 0], metavar=('X','Y','YAW'))
    args = parser.parse_args()
    build(args.map.resolve(), args.output.resolve(), args.height, args.spawn)
