#!/usr/bin/python3
"""Build a uniform-density STL URDF and a drivable Ackermann URDF/SDF."""
import argparse,json,math,struct,subprocess
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from build_world import ROOT,tag
from chassis_config import load_chassis, inner_wheel_limit

def read_stl(path):
    with path.open('rb') as f:
        f.read(80);n=struct.unpack('<I',f.read(4))[0]
        data=np.fromfile(f,dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]),count=n)
    if len(data)!=n:raise ValueError('Invalid binary STL')
    return data['v'].astype(np.float64)

def mass_properties(tri,mass):
    _,ids=np.unique(np.round(tri.reshape(-1,3),8),axis=0,return_inverse=True)
    faces=ids.reshape(-1,3)
    directed=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    edges=np.sort(directed,axis=1);_,inverse,counts=np.unique(edges,axis=0,return_inverse=True,return_counts=True)
    signs=np.where(directed[:,0]<directed[:,1],1,-1)
    if np.any(counts!=2) or np.any(np.bincount(inverse,weights=signs)!=0):
        raise ValueError('Mesh must be closed with consistently oriented faces for solid inertia')
    volumes=np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2]))/6
    if volumes.sum()<0:volumes=-volumes
    volume=volumes.sum()
    if volume<=0:raise ValueError('Nonpositive solid volume')
    sums=tri.sum(axis=1)
    center=np.einsum('i,ij->j',volumes,sums)/(4*volume)
    second=np.einsum('i,ijk,ijl->kl',volumes,tri,tri)/20
    second+=np.einsum('i,ij,ik->jk',volumes,sums,sums)/20
    second*=mass/volume
    inertia=np.trace(second)*np.eye(3)-second-mass*shift(center)
    if np.linalg.eigvalsh(inertia).min()<=0:raise ValueError('Invalid inertia')
    return volume,center,inertia

def shift(pos):return np.dot(pos,pos)*np.eye(3)-np.outer(pos,pos)
def fmt(v):return ' '.join(f'{x:.12g}' for x in np.asarray(v).ravel())
def inertial(link,mass,center,inertia):
    part=tag(link,'inertial');tag(part,'origin',xyz=fmt(center),rpy='0 0 0');E.SubElement(part,'mass',{'value':f'{mass:.12g}'})
    tag(part,'inertia',**{k:f'{inertia[i,j]:.12g}' for k,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
def joint(robot,name,parent,child,pos,axis=None,limit=None):
    j=tag(robot,'joint',name=name,type='revolute' if limit else ('continuous' if axis else 'fixed'))
    tag(j,'parent',link=parent);tag(j,'child',link=child);tag(j,'origin',xyz=fmt(pos),rpy='0 0 0')
    if axis:tag(j,'axis',xyz=axis)
    if limit:tag(j,'limit',lower=str(-limit),upper=str(limit),effort='150',velocity='1.0')
    if axis:tag(j,'dynamics',damping='0.05',friction='0.0')
    return j

def write_xml(root,path):
    E.indent(root);E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def build(mass=100.):
    chassis=load_chassis()
    wheelbase=chassis['wheelbase_m']
    wheel_radius=chassis['wheel_radius_m']
    track=chassis['track_m']
    if mass<=3:raise ValueError('Mass must exceed auxiliary link mass')
    original=read_stl(ROOT/'车身.STL')
    tri=original[:,:,[0,2,1]]*np.array([1,-1,1])*.001
    tri+=np.array([-.6,.45,0])
    volume,center,inertia=mass_properties(tri,mass)
    meshes=ROOT/'models/meshes';meshes.mkdir(parents=True,exist_ok=True)
    mesh=meshes/'car_body_m.stl'
    with mesh.open('wb') as f:
        f.write(b'ROS metre units, X forward Y left Z up'.ljust(80,b' '));f.write(struct.pack('<I',len(tri)))
        for t in tri:
            normal=np.cross(t[1]-t[0],t[2]-t[0]);normal/=np.linalg.norm(normal) if np.linalg.norm(normal)>0 else 1
            f.write(struct.pack('<12fH',*normal,*t.ravel(),0))
    # Keep generated assets relocatable.  The SDF / URDF live in models/, so a
    # relative URI continues to work after copying the bundle to a server.
    mesh_uri='meshes/car_body_m.stl'
    # A standalone solid body, as requested; collision is a stable bounding box.
    solid=E.Element('robot',name='uniform_car_body');link=tag(solid,'link',name='base_link')
    inertial(link,mass,center,inertia)
    vis=tag(link,'visual');tag(tag(vis,'geometry'),'mesh',filename=mesh_uri)
    color=tag(vis,'material',name='body_blue');tag(color,'color',rgba='0.12 0.45 0.7 1')
    col=tag(link,'collision');tag(col,'origin',xyz='0 0 0.2475',rpy='0 0 0')
    tag(tag(col,'geometry'),'box',size='1.2 0.9 0.495')
    write_xml(solid,ROOT/'models/car_body.urdf')
    robot=E.Element('robot',name='patrol_car');base=tag(robot,'link',name='base_link')
    # Subtract auxiliary link spatial inertias so combined mass/inertia remain
    # exactly those of the original homogeneous 100 kg STL solid.
    auxiliaries=[]
    wheel_inertia=np.diag([.5*(3*wheel_radius**2+.1**2)/12,.5*wheel_radius**2/2,.5*(3*wheel_radius**2+.1**2)/12])
    for end,x in [('rear',-wheelbase/2),('front',wheelbase/2)]:
        for side,y in [('left',track/2),('right',-track/2)]:
            pos=np.array([x,y,wheel_radius]);auxiliaries.append((.5,pos,wheel_inertia))
            if end=='front':auxiliaries.append((.05,pos,np.eye(3)*.0001))
    base_mass=mass-sum(a[0] for a in auxiliaries)
    base_center=(mass*center-sum((m*p for m,p,_ in auxiliaries),np.zeros(3)))/base_mass
    base_inertia=inertia+mass*shift(center)-sum((i+m*shift(p) for m,p,i in auxiliaries),np.zeros((3,3)))-base_mass*shift(base_center)
    inertial(base,base_mass,base_center,base_inertia)
    base.append(E.fromstring(E.tostring(vis)))
    col=tag(base,'collision');tag(col,'origin',xyz='0 0 0.39',rpy='0 0 0');tag(tag(col,'geometry'),'box',size='1.2 0.7 0.21')
    for end,x in [('rear',-wheelbase/2),('front',wheelbase/2)]:
        for side,y in [('left',track/2),('right',-track/2)]:
            name=f'{end}_{side}';parent='base_link';pos=[x,y,wheel_radius]
            if end=='front':
                parent=name+'_knuckle';knuckle=tag(robot,'link',name=parent);inertial(knuckle,.05,[0,0,0],np.eye(3)*.0001)
                joint(robot,name+'_steer','base_link',parent,pos,'0 0 1',inner_wheel_limit(chassis));pos=[0,0,0]
            wheel=tag(robot,'link',name=name+'_wheel');inertial(wheel,.5,[0,0,0],wheel_inertia)
            col=tag(wheel,'collision');tag(col,'origin',xyz='0 0 0',rpy=f'{math.pi/2} 0 0')
            tag(tag(col,'geometry'),'cylinder',radius=str(wheel_radius),length='.1')
            joint(robot,name+'_roll',parent,name+'_wheel',pos,'0 1 0')
            gz=tag(robot,'gazebo',reference=name+'_wheel');tag(gz,'mu1',1.0);tag(gz,'mu2',1.0)
    laser=tag(robot,'link',name='laser_frame');joint(robot,'laser_mount','base_link','laser_frame',[0,0,.85])
    gazebo=tag(robot,'gazebo');plugin=tag(gazebo,'plugin',filename='ignition-gazebo-ackermann-steering-system',name='ignition::gazebo::systems::AckermannSteering')
    for key,value in [('left_joint','rear_left_roll'),('right_joint','rear_right_roll'),('left_steering_joint','front_left_steer'),('right_steering_joint','front_right_steer'),('wheel_base',wheelbase),('wheel_separation',track),('kingpin_width',track),('wheel_radius',wheel_radius),('steering_limit',math.radians(chassis['max_center_steer_deg'])),('topic','/sim/cmd_vel'),('odom_topic','/sim/odom'),('tf_topic','/sim/tf'),('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',30),('min_velocity',-chassis['max_reverse_mps']),('max_velocity',chassis['max_forward_mps'])]:tag(plugin,key,value)
    gazebo=tag(robot,'gazebo',reference='base_link')
    sensor=tag(gazebo,'sensor',name='lidar',type='gpu_lidar');tag(sensor,'pose','0 0 .85 0 0 0')
    tag(sensor,'topic','/sim/scan');tag(sensor,'update_rate',10);tag(sensor,'always_on','true');tag(sensor,'ignition_frame_id','laser_frame')
    lidar=tag(sensor,'lidar');scan=tag(lidar,'scan');h=tag(scan,'horizontal')
    for key,value in [('samples',720),('resolution',1),('min_angle',-math.pi),('max_angle',math.pi)]:tag(h,key,value)
    rng=tag(lidar,'range')
    for key,value in [('min',.15),('max',25),('resolution',.01)]:tag(rng,key,value)
    path=ROOT/'models/patrol_car.urdf';write_xml(robot,path)
    result=subprocess.run(['ign','sdf','-p',str(path)],capture_output=True,text=True,check=True)
    sdf=E.fromstring(result.stdout);model=sdf.find('model')
    if len(model.findall('link')) != 7:
        raise ValueError('URDF conversion did not retain all 7 physical links: '+result.stderr)
    if model.find('pose') is None:tag(model,'pose','0 0 0 0 0 0')
    write_xml(sdf,ROOT/'models/patrol_car.sdf')
    metadata=dict(source=str(ROOT/'车身.STL'),mass_kg=mass,volume_m3=float(volume),density_kg_m3=float(mass/volume),
        center_of_mass_m=center.tolist(),inertia_kg_m2=inertia.tolist(),dimensions_m=[1.2,.9,.495],
        stl_transform='millimetres -> metres; x=X/1000-0.6, y=-Z/1000+0.45, z=Y/1000',
        wheelbase_m=wheelbase,wheel_radius_m=wheel_radius,track_m=track,
        assumption='Original CAD wheels remain rigid in visual mesh; auxiliary steering/rolling cylinders are collision only. X+ is assumed forward. Aggregate dynamics match homogeneous STL.')
    (ROOT/'models/car_properties.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--mass',type=float,default=100.)
    build(parser.parse_args().mass)
