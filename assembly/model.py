"""Local assembly drafts, preserving source meshes and authoritative sensor TF."""
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import xml.etree.ElementTree as ET
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from build_sensor_description import world_transforms


def catalog():
    assets={}
    for key,label,path,scale,color in [
        ('body','车身',ROOT/'models/meshes/car_body_m.stl',1.,'#739cae'),
        ('l2','L2 雷达',ROOT/'models/unitree_l2/upstream/meshes/l2_lidar.stl',1.,'#334354'),
        ('l2_bracket','激光雷达 L2 支架',ROOT/'激光雷达L2支架.STL',.001,'#f4ab55'),
        ('camera_bracket','摄像头左支架',ROOT/'摄像头左支架.STL',.001,'#64c4bb'),
        ('camera_right_bracket','摄像头右支架',ROOT/'摄像头右支架.STL',.001,'#599ed0'),
        ('ultrasonic_left','超声波左支架',ROOT/'超声波雷达左.STL',.001,'#7ec88b'),
        ('ultrasonic_right','超声波右支架',ROOT/'超声波雷达右.STL',.001,'#d39d73'),
        ('ultrasonic_body','超声波探头本体',ROOT/'超声波雷达.STL',.001,'#465768'),
        ('main_bracket','主控总支架',ROOT/'主控总.STL',.001,'#af9ce0')]:
        b=path.read_bytes();n=struct.unpack_from('<I',b,80)[0]
        if len(b)!=84+n*50:raise ValueError(f'不支持的 STL：{path.name}')
        dt=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        v=np.frombuffer(b,dtype=dt,offset=84)['vertices'].reshape(-1,3).astype(float)*scale
        low=v.min(0);high=v.max(0)
        offset=(-(low+high)/2).tolist();offset[2]=-float(low[2])
        if key in ('body','l2'):offset=[0.,0.,0.]
        assets[key]=dict(id=key,label=label,path=str(path),url='/meshes/'+key,scale=scale,
            offset_m=offset,color=color,bounds_min_m=low.tolist(),bounds_max_m=high.tolist(),
            size_mm=((high-low)*1000).tolist(),triangles=n,sha256=hashlib.sha256(b).hexdigest())
    return assets


def defaults(assets):
    measured=yaml.safe_load((ROOT/'sensor_mounting_tf.yaml').read_text())
    parts=[dict(id='base_link',label='车身 · 固定基准',asset='body',parent=None,xyz_m=[0.,0.,0.],rpy_deg=[0.,0.,0.],visible=True,confirmed=True,notes='',category='body')]
    sensor_assets={'unilidar_lidar':('l2','L2 雷达'), 'camera_left_link':('camera','左相机 0 · 示意外形'),
                   'camera_right_link':('camera','右相机 1 · 示意外形'),'imu_link':('imu','独立 IMU · 示意外形')}
    sonar_positions={'ultrasonic_1':'后右','ultrasonic_2':'前右','ultrasonic_3':'前左','ultrasonic_4':'后左'}
    for key,p in measured.items():
        asset,label=sensor_assets.get(key,('frame','第三根横杠中心' if key=='third_crossbar' else key))
        if key in sonar_positions:label=f'超声波 {key[-1]} · {sonar_positions[key]}测距坐标'
        parts.append(dict(id=key,label=label,asset=asset,parent=p['parent'],xyz_m=[float(p.get(k,0)) for k in ('x','y','z')],
            rpy_deg=[math.degrees(float(p.get(k,0))) for k in ('roll','pitch','yaw')],visible=True,confirmed=True,notes='来自已确认的 sensor_mounting_tf.yaml',category='sensor' if key in sensor_assets else 'reference'))
    # Inventory positions are deliberately separated, never presented as measured mounts.
    for key,xyz in [('main_bracket',[.15,-.85,.05]),('l2_bracket',[-.3,-.85,.05]),('camera_bracket',[-.6,-.85,.05]),
                    ('camera_right_bracket',[-.6,-1.05,.05])]:
        parts.append(dict(id=key,label=assets[key]['label'],asset=key,parent='base_link',xyz_m=xyz,
            rpy_deg=[0.,0.,0.],visible=True,confirmed=False,notes='待指定安装位置。模型按毫米读取；局部原点为包围盒底面中心。',category='bracket'))
    for frame,position in sonar_positions.items():
        asset='ultrasonic_right' if frame in ('ultrasonic_1','ultrasonic_2') else 'ultrasonic_left'
        parts.append(dict(id=frame+'_model',label=f'超声波 {frame[-1]} · {position}支架',asset=asset,parent=frame,
            xyz_m=[0.,0.,0.],rpy_deg=[0.,0.,0.],visible=True,confirmed=False,category='attachment',
            notes='沿用已确认的测距坐标；外形底面中心暂置于该坐标，仅供装配调整。需对齐探头/孔位，不能把包围盒中心当成声学原点。'))
    for frame,position in sonar_positions.items():
        parts.append(dict(id=frame+'_body',label=f'超声波 {frame[-1]} · {position}本体',asset='ultrasonic_body',parent=frame+'_model',
            xyz_m=[0.,0.,.060],rpy_deg=[0.,0.,0.],visible=True,confirmed=False,category='attachment',
            notes='真实探头本体 STL，暂放对应支架上方，尚未确认安装。移动/旋转本体以对齐探头面及螺孔；测距坐标保持不变。'))
    return dict(version=1,revision=0,saved_at=None,notes='',parts=parts,markers=[],source_mounting_sha256=hashlib.sha256((ROOT/'sensor_mounting_tf.yaml').read_bytes()).hexdigest())


def frames_for(state):
    return {p['id']:dict(parent=p['parent'],**dict(zip(('x','y','z'),p['xyz_m'])),
              **dict(zip(('roll','pitch','yaw'),map(math.radians,p['rpy_deg'])))) for p in state['parts'] if p['id']!='base_link'}


EXTRA_PARTS={'camera_right_bracket',*[f'ultrasonic_{i}_{suffix}' for i in range(1,5) for suffix in ('model','body')]}


def upgrade(raw,assets):
    """Add only new inventory instances; never replace saved poses or annotations."""
    if not isinstance(raw,dict) or raw.get('version')!=1:return raw
    state=copy.deepcopy(raw)
    if not isinstance(state.get('parts'),list) or any(not isinstance(p,dict) for p in state['parts']):return state
    ids={p.get('id') for p in state['parts']}
    for p in defaults(assets)['parts']:
        if p['id'] in EXTRA_PARTS and p['id'] not in ids:state['parts'].append(p)
    return state


def validate(raw,assets):
    if not isinstance(raw,dict) or raw.get('version')!=1:raise ValueError('装配文件版本无效')
    state=copy.deepcopy(raw);base=defaults(assets)
    if type(state.get('revision')) is not int or not 0<=state['revision']<1e12:raise ValueError('保存版本无效')
    if not isinstance(state.get('notes'),str) or len(state['notes'])>10000:raise ValueError('装配说明过长')
    if not isinstance(state.get('parts'),list):raise ValueError('部件表无效')
    expected={p['id']:p for p in base['parts']}
    if len(state['parts'])!=len(expected) or {p.get('id') for p in state['parts']}!=set(expected):raise ValueError('部件集合不匹配')
    def vector(v,limit):
        if not isinstance(v,list) or len(v)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>limit for x in v):raise ValueError('位置或角度须为范围内的有限数值')
    for p in state['parts']:
        original=expected[p['id']]
        if p.get('asset')!=original['asset']:raise ValueError('部件模型不可替换')
        p['category']=original['category'];p['label']=original['label']
        vector(p.get('xyz_m'),10);vector(p.get('rpy_deg'),3600)
        pivot=p.get('pivot_percent')
        if pivot is not None:
            vector(pivot,100)
            if any(v<0 for v in pivot):raise ValueError('参考点百分比必须在 0～100% 之间')
            if p['asset']=='frame':raise ValueError('无实体参考坐标不能设置包围盒百分比')
        if type(p.get('visible')) is not bool or type(p.get('confirmed')) is not bool:raise ValueError('显示或确认状态无效')
        if not isinstance(p.get('notes'),str) or len(p['notes'])>4000:raise ValueError('部件备注过长')
        if p['id']=='base_link':
            if p['parent'] is not None or p['xyz_m']!=[0,0,0] or p['rpy_deg']!=[0,0,0]:raise ValueError('车身基准不能移动')
        elif p.get('parent') not in expected or p['parent']==p['id']:raise ValueError('父部件无效')
        if p['id']=='third_crossbar' and any(p[k]!=original[k] for k in ('parent','xyz_m','rpy_deg')):raise ValueError('第三根横杠原点保持已确认位置')
    world_transforms(frames_for(state)) # Reject cycles and missing parents.
    markers=state.get('markers')
    if not isinstance(markers,list) or len(markers)>150:raise ValueError('标记点过多')
    ids=set()
    for m in markers:
        if type(m.get('id')) is not int or not 1<=m['id']<=100000 or m['id'] in ids:raise ValueError('标记编号无效')
        ids.add(m['id'])
        if m.get('part') not in expected:raise ValueError('标记部件无效')
        vector(m.get('point_m'),10);vector(m.get('normal'),1.001)
        if not .99<=np.linalg.norm(m['normal'])<=1.01:raise ValueError('标记面法向无效')
        if not isinstance(m.get('label'),str) or len(m['label'])>300:raise ValueError('标记说明过长')
    return state


def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,delete=False) as file:
        temp=Path(file.name)
        try:
            json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n');file.flush();os.fsync(file.fileno())
        except BaseException:
            temp.unlink(missing_ok=True);raise
    os.replace(temp,path)


class ConflictError(ValueError):pass
class Store:
    def __init__(self,path,assets=None):self.path=Path(path);self.assets=assets or catalog();self.lock=threading.RLock()
    def load(self):
        with self.lock:
            if not self.path.exists():atomic_json(self.path,defaults(self.assets))
            original=json.loads(self.path.read_text());upgraded=upgrade(original,self.assets)
            state=validate(upgraded,self.assets)
            if upgraded!=original:
                atomic_json(self.path.parent/'backups'/f"before_extra_meshes_r{original['revision']:05d}.json",original)
                atomic_json(self.path,state)
            return state
    def save(self,raw):
        with self.lock:
            state=validate(upgrade(raw,self.assets),self.assets);old=self.load()
            if state['revision']!=old['revision']:raise ConflictError('另一页面已保存修改，请先下载当前草稿，再刷新合并')
            state['revision']+=1;state['saved_at']=datetime.datetime.now().astimezone().isoformat()
            backups=self.path.parent/'backups';atomic_json(backups/f"assembly_r{old['revision']:05d}.json",old)
            atomic_json(self.path,state);return state
    def export(self):
        with self.lock:
            state=self.load();folder=self.path.parent/'exports';folder.mkdir(exist_ok=True)
            robot=ET.parse(ROOT/'models/patrol_car.urdf').getroot();robot.set('name','patrol_car_assembly_draft')
            for mesh in robot.findall('.//mesh'):
                name=mesh.get('filename','')
                if name and '://' not in name and not Path(name).is_absolute():mesh.set('filename',(ROOT/'models'/name).resolve().as_uri())
            for p in state['parts']:
                if p['id']=='base_link':continue
                link=ET.SubElement(robot,'link',name=p['id'])
                joint=ET.SubElement(robot,'joint',name='assembly_'+p['id'],type='fixed')
                ET.SubElement(joint,'parent',link=p['parent']);ET.SubElement(joint,'child',link=p['id'])
                ET.SubElement(joint,'origin',xyz=' '.join(map(str,p['xyz_m'])),rpy=' '.join(str(math.radians(v)) for v in p['rpy_deg']))
                asset=p['asset']
                if asset in self.assets:
                    a=self.assets[asset];visual=ET.SubElement(link,'visual');ET.SubElement(visual,'origin',xyz=' '.join(map(str,a['offset_m'])),rpy='0 0 0')
                    ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename=Path(a['path']).as_uri(),scale=' '.join([str(a['scale'])]*3))
                elif asset in ('camera','imu'):
                    visual=ET.SubElement(link,'visual');ET.SubElement(ET.SubElement(visual,'geometry'),'box',size='.06 .05 .04' if asset=='camera' else '.04 .03 .015')
            robot.insert(0,ET.Comment('Assembly DRAFT. Bracket mass/collision not added. No automatic simulation or hardware update.'))
            ET.indent(robot,space='  ');ET.ElementTree(robot).write(folder/'patrol_car_assembly.urdf',encoding='utf-8',xml_declaration=True)
            (folder/'assembly_frames.yaml').write_text('# Draft assembly frames; not applied to hardware or simulation.\n'+yaml.safe_dump(frames_for(state),allow_unicode=True,sort_keys=False))
            atomic_json(folder/'assembly_snapshot.json',state)
            return dict(files=[str(folder/name) for name in ('patrol_car_assembly.urdf','assembly_frames.yaml','assembly_snapshot.json')],unconfirmed=[p['label'] for p in state['parts'] if not p['confirmed']])
