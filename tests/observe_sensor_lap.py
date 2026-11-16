#!/usr/bin/python3
"""Private, read-only ground-truth evaluation. Never publishes into ROS."""
import json,math,os,re,subprocess,time,fcntl
from pathlib import Path
import xml.etree.ElementTree as ET
from urllib.request import urlopen
ROOT=Path(__file__).resolve().parents[1]
lock=open('/tmp/sensor_lap_evaluator.lock','w')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
audit=json.loads((ROOT/'artifacts/sensor_localization_current.json').read_text())
model=ET.parse(audit['simulator_world']).getroot().find(".//world/model[@name='patrol_car']")
spawn=list(map(float,model.findtext('pose').split()))
report=dict(role='evaluation only; no ROS publishers; spawn is never passed to task/estimator',
            reference_spawn=spawn,samples=[],observed_distance_m=0.,observed_turn_rad=0.,minimum_sonar_m=None)
previous=None;begin=time.monotonic()
while time.monotonic()-begin<1900:
    try:
        out=subprocess.run(['ign','topic','-e','-t','/sim/ground_truth','-n','1'],env=dict(os.environ,IGN_PARTITION='patrol_sensors'),capture_output=True,text=True,timeout=5,check=True).stdout
        def fields(pattern):
            block=re.search(pattern,out,re.S).group(1)
            return {key:float(value) for key,value in re.findall(r'\b([xyzw]):\s*([^\s]+)',block)}
        p=fields(r'pose\s*\{\s*position\s*\{([^}]*)\}');q=fields(r'orientation\s*\{([^}]*)\}')
        yaw=math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
        stamp=re.search(r'stamp\s*\{([^}]*)\}',out,re.S).group(1)
        sec=re.search(r'\bsec:\s*(\d+)',stamp);nsec=re.search(r'nsec:\s*(\d+)',stamp)
        sample=dict(x=p.get('x',0),y=p.get('y',0),yaw=yaw,sim_time=(int(sec[1]) if sec else 0)+(int(nsec[1])*1e-9 if nsec else 0))
        if previous:
            report['observed_distance_m']+=math.hypot(sample['x']-previous['x'],sample['y']-previous['y'])
            report['observed_turn_rad']+=math.atan2(math.sin(yaw-previous['yaw']),math.cos(yaw-previous['yaw']))
        previous=sample;report['samples'].append(sample)
        report['true_distance_to_spawn_m']=math.hypot(sample['x']-spawn[0],sample['y']-spawn[1])
        report['true_heading_error_rad']=math.atan2(math.sin(yaw-spawn[5]),math.cos(yaw-spawn[5]))
        state=json.load(urlopen('http://127.0.0.1:8769/api/state',timeout=2))
        sample['localization_status']=state.get('localization',{});sample['odometry']=state.get('odometry',{});sample['task_pose']=state.get('localization',{}).get('patrol',{}).get('status',{}).get('pose')
        ranges=[item['distance'] for item in state['channels'] if item['distance'] is not None]
        if ranges:report['minimum_sonar_m']=min(ranges+[report['minimum_sonar_m']] if report['minimum_sonar_m'] is not None else ranges)
        task=json.loads((ROOT/'artifacts/sensor_corridor_lap.json').read_text());report['task_state']=task['state'];report['completed_laps']=task['completed_laps']
        report['wall_duration_s']=time.monotonic()-begin
        if task['state'] in ('succeeded','failed','canceled'):
            report['passed']=task['state']=='succeeded' and task['completed_laps']==1 and report['observed_distance_m']>35 and abs(report['observed_turn_rad'])>5.5 and report['true_distance_to_spawn_m']<.6
            (ROOT/'artifacts/sensor_lap_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True);break
        (ROOT/'artifacts/sensor_lap_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    except Exception as error:
        print(str(error),flush=True)
        try:
            task=json.loads((ROOT/'artifacts/sensor_corridor_lap.json').read_text())
            if task['state'] in ('succeeded','failed','canceled'):
                report['task_state']=task['state'];report['passed']=False;report['evaluation_error']=str(error)
                (ROOT/'artifacts/sensor_lap_evaluation.json').write_text(json.dumps(report,indent=2)+'\n');break
        except (OSError,ValueError):pass
    time.sleep(3)
