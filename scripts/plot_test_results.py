#!/usr/bin/python3
"""Plot recorded test-scene CSV against independent physical ground truth."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser(description=__doc__);p.add_argument('csv',type=Path);a=p.parse_args()
with a.csv.open() as f:rows=list(csv.DictReader(f))
if not rows:raise SystemExit('No measurements in CSV')
col=lambda key:np.array([float(row[key]) if row[key] else np.nan for row in rows])
t=col('sim_time');t-=t[0]
fig,axes=plt.subplots(2,2,figsize=(12,8))
axes[0,0].plot(col('x'),col('y'));axes[0,0].set_aspect('equal',adjustable='datalim');axes[0,0].set_xlabel('world x (m)');axes[0,0].set_ylabel('world y (m)');axes[0,0].set_title('Physical trajectory')
axes[0,1].plot(t,col('z'));axes[0,1].set_ylabel('height (m)');axes[0,1].set_title('Physical height')
axes[1,0].plot(t,col('true_vx'),label='true body speed');axes[1,0].plot(t,col('wheel_vx'),label='wheel odometry');axes[1,0].plot(t,col('command_vx'),label='command',alpha=.5);axes[1,0].set_ylabel('m/s');axes[1,0].legend();axes[1,0].set_title('Traction / slip indicator')
axes[1,1].plot(t,col('pitch_deg'),label='pitch');axes[1,1].plot(t,col('roll_deg'),label='roll');axes[1,1].set_ylabel('degrees');axes[1,1].legend();axes[1,1].set_title('Body attitude')
for ax in axes.ravel():ax.grid(True)
for ax in [axes[0,1],axes[1,0],axes[1,1]]:ax.set_xlabel('elapsed simulation time (s)')
fig.suptitle(a.csv.stem);fig.tight_layout();target=a.csv.with_suffix('.png');fig.savefig(target,dpi=140);print(target)
