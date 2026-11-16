#!/usr/bin/python3
"""Build a portable Gazebo/Nav2 navigation simulation archive."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import yaml

ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parent


def validate_sync():
    chassis=yaml.safe_load((ROOT/'config/chassis_sim.yaml').read_text())
    teleop=yaml.safe_load((WORKSPACE/'src/robot_core/robot_bringup/config/joystick_teleop.yaml').read_text())['joystick_teleop']['ros__parameters']
    for sim,real in (('wheelbase_m','wheelbase'),('max_forward_mps','max_forward_speed'),
                     ('max_reverse_mps','max_reverse_speed')):
        if abs(chassis[sim]-teleop[real])>1e-6:
            raise ValueError(f'Simulation {sim} differs from commissioned {real}')
    model=json.loads((ROOT/'models/car_properties.json').read_text())
    if abs(model['wheelbase_m']-chassis['wheelbase_m'])>1e-6:
        raise ValueError('Generated vehicle model is stale; run scripts/build_car.py')
    real_mppi=yaml.safe_load((WORKSPACE/'src/robot_modules/navigation/robot_navigation/config/nav2_params.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    sim_mppi=yaml.safe_load((ROOT/'config/controller_mppi.yaml').read_text())
    for key in ('time_steps','model_dt','batch_size','vx_std','wz_std','temperature'):
        if sim_mppi[key]!=real_mppi[key]:
            raise ValueError(f'Simulation MPPI {key} differs from current real navigation config')


def copy_payload(stage):
    for folder in ('config','scripts','launch','calibration','maps','models'):
        folder_ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.png','*.log','*.pdf','*.md',
            *(['sensor_navigation_bundle'] if folder=='calibration' else []))
        shutil.copytree(ROOT/folder,stage/folder,ignore=folder_ignore)
    shutil.copytree(ROOT/'conversations',stage/'conversations')
    (stage/'reports').mkdir()
    shutil.copy2(WORKSPACE/'docs/reports/2026-11-06_simulation_plan/仿真模拟方案报告.md',
                 stage/'reports/仿真模拟方案报告.md')
    shutil.copy2(WORKSPACE/'docs/reports/2026-11-06_robot_status/报告.md',
                 stage/'reports/实车阶段问题清单.md')
    shutil.copy2(WORKSPACE/'docs/reports/2026-11-06_robot_status/status_snapshot.json',
                 stage/'reports/status_snapshot.json')
    for extension in ('.yaml','.pgm'):
        source=WORKSPACE/'src/robot_core/robot_bringup/maps'/f'map_20260817_154310{extension}'
        shutil.copy2(source,stage/'maps'/source.name)
    (stage/'assembly').mkdir()
    for file in (ROOT/'assembly').glob('*.py'):
        shutil.copy2(file,stage/'assembly'/file.name)
    settings=ROOT/'assembly/assembly_settings.json'
    if settings.is_file():shutil.copy2(settings,stage/'assembly'/settings.name)
    shutil.copytree(ROOT/'worlds',stage/'worlds',
        ignore=lambda directory,names: {'sensor_mounting_preview.sdf'} & set(names))
    (stage/'test_scenes').mkdir()
    for folder in ('worlds','maps','routes'):
        shutil.copytree(ROOT/'test_scenes'/folder,stage/'test_scenes'/folder)
    for file in (ROOT/'test_scenes').glob('*.yaml'):
        shutil.copy2(file,stage/'test_scenes'/file.name)
    for filename in ('run_sim.sh','run_server_nav.sh','Dockerfile.nav','SERVER_NAV_README.md','sensor_mounting_tf.yaml','车身.STL'):
        shutil.copy2(ROOT/filename,stage/filename)
    recording=WORKSPACE/'maps/recorded/20261106_161101_pointlio_map_cd4975'
    recording_copy=stage/'reference/latest_recording_20261106_161101'
    recording_copy.mkdir(parents=True,exist_ok=True)
    shutil.copy2(recording/'session.json',recording_copy/'session.json')
    shutil.copytree(recording/'pointlio_loop_closed',recording_copy/'pointlio_loop_closed',
                    ignore=shutil.ignore_patterns('map_3d.html'))
    session=json.loads((recording_copy/'session.json').read_text())
    session['rosbag']=None
    session['rosbag_included']=False
    (recording_copy/'session.json').write_text(json.dumps(session,ensure_ascii=False,indent=2)+'\n')
    edits=recording_copy/'pointlio_loop_closed/display_edits.json'
    edit_data=json.loads(edits.read_text())
    edit_data['source']='map.npz'
    edits.write_text(json.dumps(edit_data,ensure_ascii=False,indent=2)+'\n')
    (recording_copy/'README.txt').write_text(
        'Latest 2026-11-06 loop-closed Point-LIO map source and height-filter outputs.\n'
        'The 834 MB all-lidars rosbag and interactive HTML preview are omitted.\n'
        'These files are reference data, not the independent physical truth for Gazebo.\n')
    snapshots={
        'config/amcl_current.yaml':WORKSPACE/'src/robot_modules/navigation/robot_navigation/config/amcl.yaml',
        'reference/real_nav2_params.yaml':WORKSPACE/'src/robot_modules/navigation/robot_navigation/config/nav2_params.yaml',
        'reference/real_chassis.yaml':WORKSPACE/'src/robot_drivers/stm32_chassis/config/chassis.yaml',
        'reference/real_joystick.yaml':WORKSPACE/'src/robot_core/robot_bringup/config/joystick_teleop.yaml',
    }
    for target,source in snapshots.items():
        output=stage/target;output.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,output)
    sonar=json.loads((WORKSPACE/'test/ultrasonic_settings.json').read_text())
    sonar.pop('port',None);sonar.pop('baud',None)
    (stage/'config/ultrasonic_settings.json').write_text(json.dumps(sonar,ensure_ascii=False,indent=2)+'\n')


def rewrite_paths(stage):
    metadata_files=list((stage/'worlds').glob('*.json'))+list((stage/'test_scenes/worlds').glob('*.json'))
    for metadata in metadata_files:
        data=json.loads(metadata.read_text())
        for key in ('map','nav_params','route','world'):
            value=data.get(key)
            if not isinstance(value,str) or not value:continue
            path=Path(value)
            if path.is_absolute():
                try:relative=path.resolve().relative_to(ROOT)
                except ValueError:
                    alternative=stage/'maps'/path.name
                    if not alternative.is_file():raise ValueError(f'External {key}: {metadata}: {value}')
                    relative=alternative.relative_to(stage)
                data[key]=os.path.relpath(stage/relative,metadata.parent)
            if not (metadata.parent/data[key]).exists():
                raise FileNotFoundError(f'{metadata}: missing {key} {data[key]}')
        metadata.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    for sdf in stage.rglob('*.sdf'):
        tree=ET.parse(sdf);changed=False
        for uri in tree.findall('.//uri'):
            value=(uri.text or '').strip()
            if not value or value.startswith('model://'):continue
            parsed=urlparse(value)
            if parsed.scheme=='file':source=Path(unquote(parsed.path)).resolve()
            elif parsed.scheme:continue
            elif Path(value).is_absolute():source=Path(value).resolve()
            else:source=None
            if source is not None:
                try:relative=source.relative_to(ROOT)
                except ValueError:raise ValueError(f'External SDF asset: {sdf}: {value}')
                uri.text=os.path.relpath(stage/relative,sdf.parent);changed=True
            if not (sdf.parent/(uri.text or '')).exists():
                alternative=stage/'models/meshes'/Path(uri.text or '').name
                if alternative.is_file():
                    uri.text=os.path.relpath(alternative,sdf.parent);changed=True
                else:raise FileNotFoundError(f'SDF asset missing: {sdf}: {uri.text}')
        if changed:
            ET.indent(tree,space='  ')
            tree.write(sdf,encoding='utf-8',xml_declaration=True)
    for urdf in stage.rglob('*.urdf'):
        tree=ET.parse(urdf);changed=False
        for mesh in tree.findall('.//mesh'):
            value=mesh.get('filename','')
            parsed=urlparse(value)
            if parsed.scheme=='file':
                source=Path(unquote(parsed.path)).resolve()
                try:relative=source.relative_to(ROOT)
                except ValueError:raise ValueError(f'External URDF mesh: {urdf}: {value}')
                target=stage/relative
            elif parsed.scheme=='package' and parsed.netloc=='l2lidar_node':
                target=stage/'models/unitree_l2/upstream'/unquote(parsed.path).lstrip('/')
            elif parsed.scheme:
                raise ValueError(f'Unsupported URDF mesh: {urdf}: {value}')
            else:
                target=(urdf.parent/value).resolve()
            if not target.is_file():raise FileNotFoundError(f'URDF mesh missing: {urdf}: {value}')
            if parsed.scheme:
                mesh.set('filename',os.path.relpath(target,urdf.parent));changed=True
        if changed:
            ET.indent(tree,space='  ')
            tree.write(urdf,encoding='utf-8',xml_declaration=True)
    for map_yaml in (stage/'maps').glob('*.yaml'):
        data=yaml.safe_load(map_yaml.read_text())
        if isinstance(data,dict) and 'image' in data:
            data['image']=Path(data['image']).name
            if not (map_yaml.parent/data['image']).is_file():
                raise FileNotFoundError(f'Map image missing: {map_yaml}: {data["image"]}')
            map_yaml.write_text(yaml.safe_dump(data,sort_keys=False))
    for nav_yaml in list((stage/'config').glob('*.yaml'))+list((stage/'test_scenes').glob('*.yaml')):
        content=nav_yaml.read_text()
        content=content.replace(str(ROOT/'config')+'/', 'config/')
        nav_yaml.write_text(content)
    for metadata in stage.rglob('*.json'):
        content=metadata.read_text()
        content=content.replace(str(ROOT)+'/', '')
        content=content.replace(str(WORKSPACE/'src/robot_core/robot_bringup/maps')+'/', 'maps/')
        metadata.write_text(content)


def write_manifest(stage):
    files=[]
    for path in sorted(p for p in stage.rglob('*') if p.is_file()):
        files.append({'path':path.relative_to(stage).as_posix(),
                      'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                      'size':path.stat().st_size})
    manifest={'bundle':'gz_patrol_nav','version':'2026-11-06',
              'platform':'ROS 2 Humble + Gazebo Fortress',
              'localization':'AMCL with time-aligned L2 and front/rear D6; wheel odometry',
              'controller':'Nav2 MPPI Ackermann',
              'default_scene':'test_scenes/worlds/narrow_turns.sdf',
              'diagnostic_scene':'worlds/recording_20261106_163030.sdf',
              'notes':'Scene pose seeds AMCL only; new map has 0.75 m forced-free swept corridor and is not physical ground truth. Real glass reflections and tire slip are not modelled.',
              'files':files}
    (stage/'BUNDLE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')


def build(output):
    validate_sync()
    output=Path(output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='patrol_nav_bundle_') as folder:
        stage=Path(folder)
        copy_payload(stage)
        rewrite_paths(stage)
        write_manifest(stage)
        with tarfile.open(output,'w:gz') as archive:
            for path in sorted(p for p in stage.rglob('*') if p.is_file()):
                archive.add(path,arcname=path.relative_to(stage).as_posix())
    digest=hashlib.sha256(output.read_bytes()).hexdigest()
    checksum=output.with_name(output.name+'.sha256')
    checksum.write_text(f'{digest}  {output.name}\n')
    print(output)
    print(checksum)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',nargs='?',default=str(ROOT/'dist/gz_patrol_nav.tar.gz'))
    build(parser.parse_args().output)
