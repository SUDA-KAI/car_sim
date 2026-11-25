"""Measured sensor visuals in SDF; all additions are visual only."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from build_sensor_description import ROOT,MESH,world_transforms


def pose_text(transform):
    r=transform[:3,:3]
    pitch=math.atan2(-r[2,0],math.hypot(r[0,0],r[1,0]))
    if abs(math.cos(pitch))>1e-8:
        roll=math.atan2(r[2,1],r[2,2]);yaw=math.atan2(r[1,0],r[0,0])
    else:
        roll=0.;yaw=math.atan2(-r[0,1],r[1,1])
    return ' '.join(str(float(v)) for v in [*transform[:3,3],roll,pitch,yaw])


def visual(base,name,transform,color,shape,**dimensions):
    node=ET.SubElement(base,'visual',name=name)
    ET.SubElement(node,'pose').text=pose_text(transform)
    geometry=ET.SubElement(node,'geometry');part=ET.SubElement(geometry,shape)
    for key,value in dimensions.items():ET.SubElement(part,key).text=str(value)
    material=ET.SubElement(node,'material')
    for key in ('ambient','diffuse'):ET.SubElement(material,key).text=color
    return node


def add_visuals(tree,mounting_path,axes=False,l2_mesh=None):
    mounting=yaml.safe_load(Path(mounting_path).read_text())
    transforms=world_transforms(mounting)
    base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
    if base is None:raise ValueError('Missing patrol_car/base_link')
    for frame,transform in transforms.items():
        if frame=='base_link':continue
        if frame=='unilidar_lidar':
            mesh=Path(l2_mesh) if l2_mesh else MESH
            if not mesh.exists():raise FileNotFoundError(mesh)
            node=visual(base,'measured_l2_mesh',transform,'.16 .18 .20 1','mesh',uri=mesh.as_uri(),scale='1 1 1')
        elif frame in ('camera_left_link','camera_right_link'):
            color='.7 .45 .16 1' if frame=='camera_left_link' else '.12 .42 .65 1'
            # Camera markers sit BEHIND the sensor plane (-X): mechanical camera +X
            # points forward, so anything at +X renders as a huge disc in the image
            # (Fortress cameras have no visibility_mask). Same trick keeps them out
            # of the neighbour camera's frustum; visibility_flags=2 hides them from
            # GPU lidars (mask bit 2), matching measured_l2_mesh handling.
            back=np.eye(4);back[0,3]=-.04
            box=visual(base,'measured_'+frame,transform@back,color,'box',size='.06 .05 .04')
            ET.SubElement(box,'visibility_flags').text='2'
            # Mechanical camera +X points forward. Cylinder local +Z is rotated to +X.
            local=np.eye(4);local[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]];local[0,3]=-.05
            lens=visual(base,'measured_'+frame+'_lens',transform@local,'.025 .025 .025 1','cylinder',radius=.013,length=.012)
            ET.SubElement(lens,'visibility_flags').text='2'
        elif frame=='imu_link':
            visual(base,'measured_external_imu',transform,'.48 .23 .72 1','box',size='.04 .03 .015')
        elif frame=='third_crossbar':
            visual(base,'measured_reference_centre',transform,'.7 .32 .85 1','sphere',radius=.013)
        elif frame in {f'ultrasonic_{i}' for i in range(1,5)} and base.find(f"visual[@name='{frame}_marker']") is None:
            visual(base,'measured_'+frame,transform,'0 .8 1 1','sphere',radius=.008)
        if axes and frame in ('unilidar_lidar','third_crossbar','imu_link','camera_left_link','camera_right_link'):
            for axis,color in [(0,'.95 .12 .08 1'),(1,'.08 .8 .18 1'),(2,'.08 .28 1 1')]:
                length=.18 if frame=='unilidar_lidar' else .09
                local=np.eye(4);local[axis,3]=length/2
                if axis==0:local[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
                elif axis==1:local[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]]
                visual(base,f'measured_{frame}_axis_{axis}',transform@local,color,'cylinder',radius=.0025,length=length)
                tip=np.eye(4);tip[axis,3]=length
                visual(base,f'measured_{frame}_axis_{axis}_tip',transform@tip,color,'sphere',radius=.006)
    return mounting,transforms


def attach_visuals(source,output,mounting_path,axes=False,l2_mesh=None):
    tree=ET.parse(source);result=add_visuals(tree,mounting_path,axes,l2_mesh)
    ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)
    return result


def resolve_resource_paths(path,source_directory):
    """Keep world-relative mesh resources valid after copying a world to /tmp."""
    tree=ET.parse(path)
    for uri in tree.findall('.//uri'):
        value=(uri.text or '').strip()
        if value and '://' not in value and not Path(value).is_absolute():
            uri.text=(Path(source_directory)/value).resolve().as_uri()
    ET.indent(tree,space='  ')
    tree.write(path,encoding='utf-8',xml_declaration=True)
