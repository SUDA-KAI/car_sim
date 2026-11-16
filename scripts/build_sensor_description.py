#!/usr/bin/python3
"""Build visual descriptions from measured TF; preserve the original car model."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'models/unitree_l2'
MESH=MODEL/'upstream/meshes/l2_lidar.stl'


def matrix(pose):
    x,y,z,r,p,a=(float(pose.get(k,0.)) for k in ('x','y','z','roll','pitch','yaw'))
    if not all(math.isfinite(v) for v in (x,y,z,r,p,a)):
        raise ValueError('Mounting transforms must be finite')
    cr,sr,cp,sp,ca,sa=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(a),math.sin(a)
    result=np.eye(4)
    result[:3,:3]=[[ca*cp,ca*sp*sr-sa*cr,ca*sp*cr+sa*sr],
                  [sa*cp,sa*sp*sr+ca*cr,sa*sp*cr-ca*sr],[-sp,cp*sr,cp*cr]]
    result[:3,3]=[x,y,z]
    return result


def world_transforms(mounting):
    result={'base_link':np.eye(4)}
    pending=set()
    def visit(frame):
        if frame in result:return result[frame]
        if frame in pending:raise ValueError(f'TF cycle at {frame}')
        if frame not in mounting:raise ValueError(f'Missing parent frame {frame}')
        pending.add(frame);pose=mounting[frame]
        result[frame]=visit(pose['parent'])@matrix(pose)
        pending.remove(frame)
        return result[frame]
    for frame in mounting:visit(frame)
    return result


def lidar_visual(link):
    visual=ET.SubElement(link,'visual',name='unitree_l2_visual')
    ET.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0')
    geometry=ET.SubElement(visual,'geometry')
    ET.SubElement(geometry,'mesh',filename=MESH.as_uri(),scale='1 1 1')
    material=ET.SubElement(visual,'material',name='l2_dark')
    ET.SubElement(material,'color',rgba='.10 .10 .10 1')


def add_mounts(robot,mounting):
    existing={link.attrib['name'] for link in robot.findall('link')}
    for frame,pose in mounting.items():
        if frame in existing:raise ValueError(f'Duplicate robot frame: {frame}')
        link=ET.SubElement(robot,'link',name=frame)
        if frame=='unilidar_lidar':lidar_visual(link)
        joint=ET.SubElement(robot,'joint',name=f'measured_mount_{frame}',type='fixed')
        ET.SubElement(joint,'parent',link=pose['parent']);ET.SubElement(joint,'child',link=frame)
        ET.SubElement(joint,'origin',xyz=' '.join(str(float(pose.get(k,0.))) for k in ('x','y','z')),
                      rpy=' '.join(str(float(pose.get(k,0.))) for k in ('roll','pitch','yaw')))


def save_urdf(path,root):
    root.insert(0,ET.Comment('Visual mounting description. No invented sensor mass/inertia or LiDAR simulation plugin.'))
    ET.indent(root,space='  ')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


def build(mounting_path):
    mounting=yaml.safe_load(mounting_path.read_text())
    if not isinstance(mounting,dict) or 'unilidar_lidar' not in mounting:
        raise ValueError('Mounting YAML must contain unilidar_lidar')
    transforms=world_transforms(mounting)
    if not MESH.exists():raise FileNotFoundError(MESH)
    standalone=ET.Element('robot',name='unitree_l2_visual')
    lidar_visual(ET.SubElement(standalone,'link',name='unilidar_lidar'))
    save_urdf(MODEL/'l2.urdf',standalone)
    chain=[];frame='unilidar_lidar'
    while frame!='base_link':
        chain.append(frame);frame=mounting[frame]['parent']
    minimal=ET.Element('robot',name='unitree_l2_measured_mount')
    ET.SubElement(minimal,'link',name='base_link')
    add_mounts(minimal,{frame:mounting[frame] for frame in reversed(chain)})
    save_urdf(MODEL/'l2_mounted.urdf',minimal)
    car=ET.parse(ROOT/'models/patrol_car.urdf').getroot()
    car.set('name','patrol_car_measured_sensors')
    add_mounts(car,mounting)
    save_urdf(ROOT/'models/patrol_car_with_sensors.urdf',car)
    raw=MESH.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+count*50:raise ValueError('Unexpected mesh format or incomplete download')
    dtype=np.dtype([('normal','<f4',(3,)),('points','<f4',(3,3)),('attribute','<u2')])
    points=np.frombuffer(raw,dtype=dtype,offset=84,count=count)['points'].reshape(-1,3)
    low,high=points.min(0),points.max(0)
    summary=dict(mounting_file=str(mounting_path.resolve()),
        mounting_sha256=hashlib.sha256(mounting_path.read_bytes()).hexdigest(),
        lidar_point_cloud_origin='Centre of bottom mounting surface, per Unitree SDK2',
        bottom_mount_to_cloud_translation_m=[0.,0.,0.],
        x_axis_definition='Opposite direction to the bottom cable outlet; yaw=0 in supplied TF assumes this points forward',
        lidar_in_base_matrix=transforms['unilidar_lidar'].tolist(),
        all_frames_in_base={name:transform.tolist() for name,transform in transforms.items()},
        mesh=dict(triangles=count,bounds_min_m=low.tolist(),bounds_max_m=high.tolist(),size_m=(high-low).tolist(),
                  note='Mesh has a 0.33 mm minimum Z; preserve upstream geometry, do not add device height to the point-cloud origin'),
        builtin_lidar_imu_position_in_cloud_m=[-.007698,-.014655,.00667],
        description_scope='Visual geometry and measured fixed frames. No added sensor dynamics or 3D scan simulation. Existing car/world/driver config unchanged.')
    (MODEL/'mounting_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(lidar_in_base=transforms['unilidar_lidar'][:3,3].tolist(),
        standalone=str(MODEL/'l2.urdf'),mounted=str(MODEL/'l2_mounted.urdf'),
        car_with_sensors=str(ROOT/'models/patrol_car_with_sensors.urdf'),mesh_size_m=(high-low).tolist()),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mounting',type=Path,default=ROOT/'sensor_mounting_tf.yaml')
    build(parser.parse_args().mounting)
