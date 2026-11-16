#!/usr/bin/python3
"""Apply the user's confirmed assembly to a frozen, reviewable model bundle."""
import argparse
import copy
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
import yaml
from build_sensor_description import ROOT
sys.path.insert(0,str(ROOT))
from assembly.model import Store,frames_for,atomic_json
from assembly_config import apply_visuals,ultrasonic_poses
from sensor_visuals import add_visuals
from ultrasonic_config import attach_sensors,load_mounting
from l2_sensor_config import attach_inertial_lidar


def build(config=ROOT/'assembly/assembly_settings.json'):
    store=Store(config);state=store.load()
    pending=[p['label'] for p in state['parts'] if p['category'] in ('bracket','attachment','sensor') and not p['confirmed']]
    if pending:raise ValueError('请先确认这些部件：'+', '.join(pending))
    target=ROOT/'models/assembled';assets=copy.deepcopy(store.assets);mapping=frames_for(state)
    with tempfile.TemporaryDirectory(prefix='.assembled_',dir=ROOT/'models') as folder:
        stage=Path(folder);(stage/'meshes').mkdir()
        for key,asset in assets.items():
            source=Path(asset['path']);relative=Path('meshes')/(key+'.stl');shutil.copy2(source,stage/relative)
            asset['source_relative']=str(source.relative_to(ROOT));asset['mesh']=str(relative)
            asset['path']=str(target/relative)
        manifest=dict(source_revision=state['revision'],source_saved_at=state['saved_at'],assets={key:{k:v for k,v in asset.items() if k not in ('path','url')} for key,asset in assets.items()})
        atomic_json(stage/'asset_manifest.json',manifest);atomic_json(stage/'assembly_snapshot.json',state)
        (stage/'sensor_mounting_tf.yaml').write_text('# Applied confirmed assembly, canonical physical frames; edit pivots retained in JSON.\n'+yaml.safe_dump(mapping,sort_keys=False,allow_unicode=True))
        # Export the captured snapshot, not a potentially edited live web file.
        exported=Store(stage/'assembly_snapshot.json',store.assets).export()
        robot=ET.parse(exported['files'][0]);robot.getroot().set('name','patrol_car_assembled')
        shutil.rmtree(stage/'exports')
        replacement={Path(store.assets[key]['path']).as_uri():Path(asset['path']).as_uri() for key,asset in assets.items()}
        for mesh in robot.findall('.//mesh'):
            name=mesh.get('filename','')
            if name in replacement:mesh.set('filename',replacement[name])
        for gazebo in robot.findall('gazebo'):
            for sensor in list(gazebo.findall('sensor')):
                if sensor.findtext('topic')=='/sim/scan':gazebo.remove(sensor)
        # The standalone URDF describes geometry / dynamics / TF; actual sensor
        # plugins are supplied by the assembled SDF / simulation launch.
        ET.indent(robot,space='  ');robot.write(stage/'patrol_car_assembled.urdf',encoding='utf-8',xml_declaration=True)
        sdf=ET.Element('sdf',version='1.8');world=ET.SubElement(sdf,'world',name='assembly_build')
        model=copy.deepcopy(ET.parse(ROOT/'models/patrol_car.sdf').find('model'));world.append(model)
        for uri in model.findall('.//uri'):
            value=(uri.text or '').strip()
            if value and '://' not in value:uri.text=(ROOT/'models'/value).resolve().as_uri()
        tree=ET.ElementTree(sdf);temporary=stage/'build_world.sdf'
        add_visuals(tree,stage/'sensor_mounting_tf.yaml');apply_visuals(tree,state,assets,mapping)
        tree.write(temporary,encoding='utf-8',xml_declaration=True)
        attach_sensors(temporary,temporary,ultrasonic_poses(load_mounting(),mapping))
        attach_inertial_lidar(temporary,temporary,stage/'sensor_mounting_tf.yaml')
        full=ET.parse(temporary)
        for uri in full.findall('.//uri'):
            if uri.text in replacement:uri.text=replacement[uri.text]
        model=full.find('world/model');output=ET.Element('sdf',version='1.8');output.append(model)
        ET.indent(output,space='  ');ET.ElementTree(output).write(stage/'patrol_car_assembled.sdf',encoding='utf-8',xml_declaration=True)
        temporary.unlink()
        summary=dict(source_config=str(Path(config).resolve()),revision=state['revision'],saved_at=state['saved_at'],
            applied_at=datetime.datetime.now().astimezone().isoformat(),parts=len(state['parts']),
            supplementary_visuals=sum(p['category'] in ('bracket','attachment') for p in state['parts']),
            snapshot_sha256=hashlib.sha256((stage/'assembly_snapshot.json').read_bytes()).hexdigest(),
            mass_and_collision='Original car dynamics unchanged; bracket/sensor meshes are visual-only.',
            sensor_frames='Canonical assembly frames, not editor pivot origins. Sonar acoustic frames remain separately measured.',
            build_command='./build_assembly.sh')
        atomic_json(stage/'application_report.json',summary)
        if target.exists():
            backup=ROOT/'assembly/backups'/('applied_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
            shutil.copytree(target,backup)
        target.mkdir(exist_ok=True)
        # Publish resources before the snapshot that activates this bundle.
        for item in stage.iterdir():
            if item.is_dir():shutil.copytree(item,target/item.name,dirs_exist_ok=True)
            elif item.name!='assembly_snapshot.json':shutil.copy2(item,target/item.name)
        atomic_json(target/'assembly_snapshot.json',state)
    print(json.dumps(summary,ensure_ascii=False,indent=2));return target


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,default=ROOT/'assembly/assembly_settings.json')
    build(parser.parse_args().config)
