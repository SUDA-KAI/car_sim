#!/usr/bin/python3
"""Render the metre-scale STL for visual inspection."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from build_car import read_stl,ROOT
tri=read_stl(ROOT/'models/meshes/car_body_m.stl')
fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(111,projection='3d')
n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-15)
light=np.array([-.4,-.3,1.]);light/=np.linalg.norm(light)
shade=.3+.7*np.maximum(n@light,0)
colors=np.c_[shade[:,None]*np.array([.22,.62,.85]),np.ones(len(tri))]
ax.add_collection3d(Poly3DCollection(tri,facecolors=colors,edgecolor='none'))
v=tri.reshape(-1,3);lo=v.min(0);hi=v.max(0)
ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2]);ax.set_box_aspect(hi-lo)
ax.set_xlabel('X / m');ax.set_ylabel('Y / m');ax.set_zlabel('Z / m');ax.view_init(25,-55)
ax.set_title('STL body: 1.2 x 0.9 x 0.495 m | Uniform total mass: 100 kg')
ax.scatter([-.0171147],[0],[.2583498],color='red',s=60,label='Centre of mass');ax.legend()
fig.tight_layout();fig.savefig(ROOT/'models/car_preview.png',dpi=140)
