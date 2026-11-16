#!/usr/bin/python3
"""Check closed-loop traversal against physical odometry, not just action success."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np


def analyze(rows, scene, patrol):
    if not rows:raise ValueError('No trajectory samples')
    a=lambda key:np.array([float(row[key]) for row in rows])
    x,y,t=a('x'),a('y'),a('sim_time')
    radius=scene['reference_geometry']['radius']
    left,right=scene['reference_geometry']['left_x'],scene['reference_geometry']['right_x']
    distance=float(np.hypot(np.diff(x),np.diff(y)).sum())
    nominal=2*(right-left)+2*math.pi*radius
    yaw=float(np.unwrap(a('yaw'))[-1]-np.unwrap(a('yaw'))[0])
    checks=dict(action_succeeded=patrol.get('result')=='success',
        right_bend_traversed=bool(x.max()>right+radius-.5),
        left_bend_traversed=bool(x.min()<left-radius+.5),
        upper_straight_reached=bool(y.max()>radius-.5),
        lower_straight_reached=bool(y.min()<-radius+.5),
        sufficient_travel=distance>.9*nominal,
        complete_heading_rotation=yaw>math.radians(330),
        no_nominal_body_overlap=bool(np.min(a('estimated_side_clearance_m'))>0),
        no_conservative_wheel_envelope_overlap=bool(np.min(a('wheel_sweep_clearance_bound_m'))>0))
    speed=a('true_vx');bend=(x>right+.5)|(x<left-.5)
    bend_speed=abs(speed[bend])
    return dict(passed=all(checks.values()),checks=checks,
        scene_radius_m=radius,clear_width_m=scene['clear_width_m'],samples=len(rows),
        sim_duration_s=float(t[-1]-t[0]),distance_m=distance,nominal_loop_length_m=nominal,
        heading_rotation_rad=yaw,min_nominal_body_clearance_m=float(np.min(a('estimated_side_clearance_m'))),
        min_conservative_wheel_clearance_m=float(np.min(a('wheel_sweep_clearance_bound_m'))),
        median_bend_speed_mps=float(np.median(bend_speed)) if bend_speed.size else None,
        bend_slow_sample_fraction=float(np.mean(bend_speed<.03)) if bend_speed.size else None,
        max_path_distance_m=max((float(row['plan_distance_m']) for row in rows if row.get('plan_distance_m')),default=None),
        note='Ideal ground-truth localization and geometric envelope checks, not a contact-sensor or real-car safety validation.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix',type=Path);parser.add_argument('--plot',action='store_true');args=parser.parse_args()
    prefix=args.prefix
    run=json.loads(prefix.with_suffix('.run.json').read_text())
    scene=run.get('scene_metadata') or json.loads(Path(run['world']).with_suffix('.json').read_text())
    patrol=json.loads(prefix.with_suffix('.patrol.json').read_text())
    rows=list(csv.DictReader(prefix.with_suffix('.csv').open()))
    report=analyze(rows,scene,patrol)
    report['run']=run;report['patrol']=patrol
    prefix.with_suffix('.turns_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from build_narrow_suite import capsule
        fig,axes=plt.subplots(1,3,figsize=(17,6),gridspec_kw={'width_ratios':[1.5,1,1]})
        r=scene['reference_geometry']['radius']
        for offset,style,color in [(-.85,'-','gray'),(.85,'-','gray'),(0,'--','green')]:
            line=capsule(r+offset);line=np.vstack([line,line[0]]);axes[0].plot(*line.T,style,color=color,linewidth=1)
        axes[0].plot([float(row['x']) for row in rows],[float(row['y']) for row in rows],label='Physical trajectory')
        axes[0].axis('equal');axes[0].set_title('5 m centreline radius / 1.7 m corridor');axes[0].set_xlabel('x (m)');axes[0].set_ylabel('y (m)');axes[0].legend()
        t=np.array([float(row['sim_time']) for row in rows]);t-=t[0]
        for key,label in [('true_vx','Actual'),('command_vx','Command')]:axes[1].plot(t,[float(row[key]) for row in rows],label=label,alpha=.8)
        axes[1].set_title('Forward speed');axes[1].set_ylabel('m/s');axes[1].legend()
        for key,label in [('estimated_side_clearance_m','Nominal body'),('wheel_sweep_clearance_bound_m','Conservative tire envelope')]:axes[2].plot(t,[float(row[key]) for row in rows],label=label)
        axes[2].axhline(0,color='red',linestyle='--');axes[2].set_title('Geometric wall clearance');axes[2].set_ylabel('m');axes[2].legend()
        for ax in axes[1:]:ax.set_xlabel('Simulation time (s)')
        for ax in axes:ax.grid(True,alpha=.3)
        fig.tight_layout();fig.savefig(prefix.with_suffix('.turns_analysis.png'),dpi=140)
    print(json.dumps({k:v for k,v in report.items() if k not in ('run','patrol')},indent=2))

if __name__=='__main__':main()
