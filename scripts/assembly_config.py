"""Frozen, confirmed assembly integration; editor pivots never redefine sensor TF."""
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from build_sensor_description import ROOT,world_transforms
from sensor_visuals import visual,pose_text
sys.path.insert(0,str(ROOT))
from assembly.model import validate,frames_for

APPLIED=ROOT/'models/assembled/assembly_snapshot.json'


def load_applied(path=APPLIED):
    path=Path(path).resolve();manifest=json.loads((path.parent/'asset_manifest.json').read_text())
    assets=copy.deepcopy(manifest['assets'])
    for asset in assets.values():
        asset['path']=str((path.parent/asset['mesh']).resolve())
        if not Path(asset['path']).is_file():raise FileNotFoundError(asset['path'])
    state=validate(json.loads(path.read_text()),assets)
    missing=[p['label'] for p in state['parts'] if p['category'] in ('bracket','attachment','sensor') and not p['confirmed']]
    if missing:raise ValueError('Applied assembly contains unconfirmed parts: '+', '.join(missing))
    return state,assets,frames_for(state)


def apply_visuals(tree,state,assets,mounting=None):
    transforms=world_transforms(mounting or frames_for(state))
    base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
    if base is None:raise ValueError('Missing patrol_car/base_link')
    for p in state['parts']:
        if p['category'] not in ('bracket','attachment'):continue
        name='assembled_'+p['id']
        if base.find(f"visual[@name='{name}']") is not None:raise ValueError('Duplicate assembly visual: '+name)
        asset=assets[p['asset']];offset=np.eye(4);offset[:3,3]=asset['offset_m']
        color=asset['color'].lstrip('#');rgba=' '.join(str(int(color[i:i+2],16)/255) for i in (0,2,4))+' 1'
        visual(base,name,transforms[p['id']]@offset,rgba,'mesh',uri=Path(asset['path']).as_uri(),scale=' '.join([str(asset['scale'])]*3))
    # Use the frozen ASCII-named body mesh as well as the supplementary meshes.
    body=base.find("visual[@name='base_link_visual']/geometry/mesh/uri")
    if body is not None:body.text=Path(assets['body']['path']).as_uri()
    return transforms


def attach_visuals(source,output,state,assets,mounting_path):
    tree=ET.parse(source);mapping=yaml.safe_load(Path(mounting_path).read_text());apply_visuals(tree,state,assets,mapping)
    ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)


def ultrasonic_poses(mounting,frames):
    """Range/FOV stay configured by the sonar driver; geometry follows assembly TF."""
    transforms=world_transforms(frames);mounting=copy.deepcopy(mounting)
    for sensor in mounting:
        x,y,z,roll,pitch,yaw=map(float,pose_text(transforms[sensor['frame']]).split())
        sensor.update(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw)
    return mounting
