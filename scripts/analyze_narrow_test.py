#!/usr/bin/python3
"""Measure narrow-corridor swept-envelope clearance from true trajectory CSV."""
import argparse,csv,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from narrow_geometry import center_offset,envelope_clearance,projected_box,reference_heading

def analyze(csv_path,scene_path):
    scene=json.loads(scene_path.read_text())
    with csv_path.open() as f:rows=list(csv.DictReader(f))
    if not rows:raise ValueError('No trajectory samples')
    col=lambda name:np.array([float(row[name]) for row in rows])
    x,y,yaw,t=col('x'),col('y'),col('yaw'),col('sim_time');t-=t[0]
    center_y=scene.get('corridor_center_y',0.);width=scene['clear_width_m']
    def quaternion(roll,pitch,yaw):
        cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)
    quaternions=[quaternion(math.radians(r),math.radians(p),a) for r,p,a in zip(col('roll_deg'),col('pitch_deg'),yaw)]
    nominal=np.array([envelope_clearance(scene,xx,yy,q) for xx,yy,q in zip(x,y,quaternions)])
    sweep=np.array([envelope_clearance(scene,xx,yy,q,scene['wheel_sweep_width_bound_m']) for xx,yy,q in zip(x,y,quaternions)])
    offset=np.array([center_offset(scene,xx,yy) for xx,yy in zip(x,y)])
    heading_errors=np.array([math.atan2(math.sin(a-reference_heading(scene,xx,yy)),math.cos(a-reference_heading(scene,xx,yy))) for xx,yy,a in zip(x,y,yaw)])
    straight=scene.get('reference_geometry',{}).get('type','straight')=='straight'
    steady=(x>=scene['spawn'][0]+2)&(x<=6)&(col('true_vx')>.05) if straight else (col('true_vx')>.05)
    signs=np.sign(col('command_wz'));signs[abs(col('command_wz'))<.005]=0;nonzero=signs[signs!=0]
    summary=dict(scene=scene['name'],clear_width_m=width,nominal_vehicle_width_m=scene['robot_width_m'],
                 max_abs_center_offset_m=float(np.max(abs(offset))),
                 max_abs_center_offset_forward_settled_m=float(np.max(abs(offset[steady]))) if straight and steady.any() else None,
                 min_nominal_body_side_clearance_m=float(nominal.min()),
                 min_wheel_sweep_clearance_bound_m=float(sweep.min()),
                 potential_nominal_body_overlap_samples=int(sum(nominal<=0)),
                 max_abs_heading_deg=float(np.max(abs(np.degrees(yaw)))),
                 max_abs_heading_error_deg=float(np.max(abs(np.degrees(heading_errors)))),
                 steering_command_sign_reversals=int(sum(nonzero[1:]!=nonzero[:-1])),
                 note='Projected 3D envelope clearance estimate against straight/stadium walls; wheel bound conservatively includes all steering angles up to 0.75 rad. Not a physical contact sensor or real-vehicle safety guarantee.')
    patrol=csv_path.with_suffix('.patrol.json')
    if patrol.exists():summary['patrol']=json.loads(patrol.read_text())
    out=csv_path.with_suffix('.clearance.json');out.write_text(json.dumps(summary,indent=2))
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    axes[0,0].plot(x,y,label='true centre')
    if straight:
        axes[0,0].axhline(center_y+width/2,color='black',lw=4);axes[0,0].axhline(center_y-width/2,color='black',lw=4)
        extent=(scene['robot_length_m']*abs(np.sin(yaw))+scene['robot_width_m']*abs(np.cos(yaw)))/2
        axes[0,0].plot(x,y+extent,alpha=.6,label='body envelope');axes[0,0].plot(x,y-extent,alpha=.6);axes[0,0].set_ylim(-1.05,1.05)
    else:
        from matplotlib.patches import Polygon
        geometry=scene['reference_geometry'];left,right,radius=geometry['left_x'],geometry['right_x'],geometry['radius']
        for radius_wall in [radius-width/2,radius+width/2]:
            path=[(right+radius_wall*math.cos(a),radius_wall*math.sin(a)) for a in np.linspace(-math.pi/2,math.pi/2,150)]+[(left+radius_wall*math.cos(a),radius_wall*math.sin(a)) for a in np.linspace(math.pi/2,3*math.pi/2,150)]
            path.append(path[0]);axes[0,0].plot(*np.array(path).T,color='black',lw=2)
        for i in np.linspace(0,len(x)-1,min(40,len(x)),dtype=int):
            axes[0,0].add_patch(Polygon(projected_box(x[i],y[i],quaternions[i],scene['robot_length_m'],scene['robot_width_m']),fill=False,edgecolor='orange',alpha=.6))
        axes[0,0].set_aspect('equal')
    axes[0,0].set_xlabel('x (m)');axes[0,0].set_ylabel('y (m)');axes[0,0].legend();axes[0,0].set_title('1.70 m clear corridor')
    axes[0,1].plot(t,offset);axes[0,1].set_ylabel('centreline offset (m)');axes[0,1].set_title('Lateral tracking')
    axes[1,0].plot(t,nominal,label='nominal body');axes[1,0].plot(t,sweep,label='conservative wheel sweep');axes[1,0].axhline(0,color='red');axes[1,0].set_ylabel('estimated side clearance (m)');axes[1,0].legend()
    axes[1,1].plot(t,col('command_wz'),label='command');axes[1,1].plot(t,col('true_wz'),label='physical yaw rate');axes[1,1].set_ylabel('rad/s');axes[1,1].legend()
    for ax in axes.ravel():ax.grid(True)
    for ax in [axes[0,1],axes[1,0],axes[1,1]]:ax.set_xlabel('elapsed simulation seconds')
    fig.suptitle(scene['name']);fig.tight_layout();fig.savefig(csv_path.with_suffix('.clearance.png'),dpi=140)
    print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('csv',type=Path);p.add_argument('--scene',type=Path,required=True);a=p.parse_args();analyze(a.csv,a.scene)
