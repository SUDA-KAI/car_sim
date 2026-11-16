#!/usr/bin/python3
"""Run isolated sensor simulation and one lap; own process groups are cleaned up."""
import fcntl
import json
import yaml
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    args=sys.argv[1:];attach='--attach' in args
    if attach:args.remove('--attach')
    env=dict(os.environ,ROS_DOMAIN_ID='43',SIM_ROS_DOMAIN_ID='43',IGN_PARTITION='patrol_sensors',SIM_IGN_PARTITION='patrol_sensors')
    lock=open('/tmp/gz_sensor_corridor_lap.lock','w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('一圈巡逻已在运行，请先结束原任务。')
    processes=[]
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        if not attach:
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():continue
                try:
                    command=(proc/'cmdline').read_bytes();variables=(proc/'environ').read_bytes().split(b'\0')
                    if b'ign gazebo server' in command and b'IGN_PARTITION=patrol_sensors' in variables:
                        raise SystemExit('传感器仿真已在运行；使用 ./run_sensor_lap.sh --attach 连接当前仿真，或先退出原仿真。')
                except (PermissionError,FileNotFoundError,ProcessLookupError):pass
            sim=subprocess.Popen([str(ROOT/'run_sensor_navigation.sh'),'sensor_slam_config:='+str(ROOT/'config/sensor_lap_slam.yaml'),*args],env=env,start_new_session=True)
            processes.append(sim)
        elif args:raise SystemExit('--attach 不接受启动参数；连接已有仿真时无需再次启动 Gazebo。')
        if attach:
            audit=json.loads((ROOT/'artifacts/sensor_localization_current.json').read_text())
            slam=yaml.safe_load(Path(audit.get('slam_config',ROOT/'config/sensor_slam.yaml')).read_text())['slam_toolbox']['ros__parameters']
            if slam['use_scan_matching'] or slam['do_loop_closing']:
                raise SystemExit('当前仿真开启二维匹配/回环。请结束旧入口，再运行 ./run_sensor_lap.sh 使用已调整的通道建图配置。')
        lap=subprocess.Popen(['/usr/bin/python3',str(ROOT/'scripts/sensor_corridor_lap.py')],env=env,start_new_session=True)
        processes.append(lap)
        while lap.poll() is None:
            if not attach and sim.poll() is not None:raise RuntimeError('Gazebo launch exited')
            time.sleep(.3)
        if lap.returncode:raise RuntimeError('巡逻未完成，请查看 artifacts/sensor_corridor_lap.json')
        print('一圈巡逻完成。网页 http://127.0.0.1:8769；Ctrl+C 关闭本次启动的仿真。',flush=True)
        if not attach:sim.wait()
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        for process in reversed(processes):
            # Gazebo forks server/gui; signal the entire group, including children
            # whose launch parent has already exited.
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and any(p.poll() is None for p in processes):time.sleep(.1)
        for process in processes:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()

if __name__=='__main__':main()
