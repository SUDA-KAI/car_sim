#!/usr/bin/python3
"""Build 1.7 m curved, sloped and low-friction scenes, without shrinking the car."""
import copy,json,math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import yaml
from PIL import Image
from build_world import ROOT,tag
from build_test_scenes import BASE,new_world,block,add_robot,friction,write_wedge

WIDTH=1.7
SWEEP=2*(.4+.2*math.sin(.75)+.05*math.cos(.75))

def symmetric_robot(world,spawn):
    add_robot(world,spawn)
    plugin=world.find("model[@name='patrol_car']/plugin[@name='ignition::gazebo::systems::AckermannSteering']")
    for name,value in [('min_velocity',-.8),('max_velocity',.8),('min_acceleration',-.4),('max_acceleration',.4)]:plugin.find(name).text=str(value)

def write_mesh(path,vertices,faces):
    vertices=np.asarray(vertices,dtype=float)
    with path.open('w') as f:
        for v in vertices:f.write('v '+' '.join(map(str,v))+'\n')
        for face in faces:
            a,b,c=vertices[list(face)];n=np.cross(b-a,c-a);n/=np.linalg.norm(n)
            f.write('vn '+' '.join(map(str,n))+'\n')
        for i,face in enumerate(faces):f.write('f '+' '.join(f'{j+1}//{i+1}' for j in face)+'\n')

def capsule(radius):
    # Chord error < 0.4 mm at the outer boundary; cross-section is 1.7 m.
    return np.array([(6+radius*math.cos(a),radius*math.sin(a)) for a in np.linspace(-math.pi/2,math.pi/2,129)]+
                    [(-2+radius*math.cos(a),radius*math.sin(a)) for a in np.linspace(math.pi/2,3*math.pi/2,129)])

def solid_extrusion(poly,z0=-.05,z1=2.):
    n=len(poly);verts=[(*p,z) for z in (z0,z1) for p in poly];faces=[]
    for i in range(1,n-1):faces += [(0,i+1,i),(n,n+i,n+i+1)]
    for i in range(n):
        j=(i+1)%n;faces += [(i,j,n+j),(i,n+j,n+i)]
    return verts,faces

def ring_extrusion(inner,outer,z0=-.05,z1=2.):
    n=len(inner);verts=[(*p,z) for z in (z0,z1) for poly in (inner,outer) for p in poly];faces=[]
    for i in range(n):
        j=(i+1)%n
        faces += [(i,n+j,n+i),(i,j,n+j),(2*n+i,3*n+i,3*n+j),(2*n+i,3*n+j,2*n+j),
                  (n+i,n+j,3*n+j),(n+i,3*n+j,3*n+i),(i,2*n+i,2*n+j),(i,2*n+j,j)]
    return verts,faces

def mesh_obstacle(world,name,path,vertices,faces):
    write_mesh(path,vertices,faces)
    model=tag(world,'model',name=name);tag(model,'static','true');link=tag(model,'link',name='link')
    for typ in ('collision','visual'):
        elem=tag(link,typ,name=typ);tag(tag(tag(elem,'geometry'),'mesh'),'uri',path.resolve().as_uri())
        if typ=='collision':friction(elem,1.)
        else:
            mat=tag(elem,'material');tag(mat,'ambient','.55 .6 .65 1');tag(mat,'diffuse','.55 .6 .65 1')

def save(name,kind,world,sdf,grid,origin,res,spawn,points,extra):
    map_path=BASE/'maps'/f'{name}.yaml';Image.fromarray(grid).save(map_path.with_suffix('.pgm'))
    map_path.write_text(yaml.safe_dump(dict(image=f'{name}.pgm',resolution=res,origin=origin,negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))
    route_data=dict(frame_id='map',waypoints=[dict(x=float(x),y=float(y),yaw=float(a)) for x,y,a in points])
    if kind=='narrow_turns':
        # A closed request ending at the current pose succeeds immediately in
        # Nav2. Handoff on the straights, keeping each entire bend continuous.
        route_data.update(navigation_mode='through_poses',segment_end_indices=[len(points)//2+1,len(points)],route_timeout_s=450.)
    route=BASE/'routes'/f'{name}.yaml';route.write_text(yaml.safe_dump(route_data,sort_keys=False))
    params=yaml.safe_load((BASE/'nav2_narrow_17.yaml').read_text())
    if kind=='narrow_turns':
        params['controller_server']['ros__parameters']['FollowPath'].update(max_vel_x=.18,min_turning_radius=2.)
        params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']=2.
        # The closed stadium has a feasible forward route; avoid nominal shunts.
        params['planner_server']['ros__parameters']['GridBased']['motion_model_for_search']='DUBIN'
        params['velocity_smoother']['ros__parameters']['max_velocity']=[.18,0.,.15]
    config=BASE/f'nav2_{name}.yaml';config.write_text(yaml.safe_dump(params,sort_keys=False))
    symmetric_robot(world,spawn)
    path=BASE/'worlds'/f'{name}.sdf';E.indent(sdf);E.ElementTree(sdf).write(path,encoding='utf-8',xml_declaration=True)
    meta=dict(name=name,kind=kind,world=str(path.resolve()),map=str(map_path.resolve()),spawn=spawn,route=str(route.resolve()),
              nav_params=str(config.resolve()),has_ground_truth=True,localization='ground_truth',clear_width_m=WIDTH,
              robot_length_m=1.2,robot_width_m=.9,robot_height_m=.495,wheel_sweep_width_bound_m=SWEEP,**extra)
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2));return meta

def build_turns(radius=5.):
    if not math.isfinite(radius) or radius<=WIDTH/2:raise ValueError('Invalid centreline radius')
    name='narrow_turns';sdf,world=new_world(name)
    block(world,'floor',[2.,0.,-.1],[8+2*radius+5,2*radius+5,.2],1.,'.65 .68 .6 1')
    inner=capsule(radius-WIDTH/2);outside=capsule(radius+WIDTH/2);outer=capsule(radius+WIDTH/2+.2)
    mesh_obstacle(world,'inner_island',BASE/'worlds'/f'{name}_inner.obj',*solid_extrusion(inner))
    mesh_obstacle(world,'outer_wall',BASE/'worlds'/f'{name}_outer.obj',*ring_extrusion(outside,outer))
    res=.02;xmin,xmax,ymin,ymax=-2-radius-2.01,6+radius+2.01,-radius-2.01,radius+2.01
    xs=xmin+(np.arange(round((xmax-xmin)/res))+.5)*res
    ys=ymax-(np.arange(round((ymax-ymin)/res))+.5)*res
    dx=np.maximum(np.maximum(-2-xs,xs-6),0.)
    d=np.sqrt(dx[None,:]**2+ys[:,None]**2)
    grid=np.full(d.shape,205,dtype=np.uint8);grid[d<=radius-WIDTH/2]=0
    grid[(d>radius-WIDTH/2)&(d<radius+WIDTH/2)]=254
    grid[(d>=radius+WIDTH/2)&(d<=radius+WIDTH/2+.2)]=0
    # Sparse 45-degree goals allow shortest-path shortcuts towards the inner
    # wall. 10-degree tangent constraints keep each connection near centreline.
    points=[(2,-radius,0),(6,-radius,0)]
    points += [(6+radius*math.cos(a),radius*math.sin(a),a+math.pi/2)
               for a in np.linspace(-math.pi/2,math.pi/2,19)[1:]]
    points += [(2,radius,math.pi),(-2,radius,math.pi)]
    points += [(-2+radius*math.cos(a),radius*math.sin(a),math.atan2(math.sin(a+math.pi/2),math.cos(a+math.pi/2)))
               for a in np.linspace(math.pi/2,3*math.pi/2,19)[1:]]
    return save(name,'narrow_turns',world,sdf,grid,[xmin,ymin,0.],res,[-2.,-radius,0.],points,
        dict(reference_geometry=dict(type='stadium',left_x=-2.,right_x=6.,radius=radius),
             description=f'Closed 1.7 m stadium corridor. Straight length 8 m; centreline turn radius {radius:g} m; curved mesh gap error <1 mm. Not a sharp 90-degree corner.'))

def build_straight_terrain(name,kind,angle=0.,mu=1.):
    sdf,world=new_world(name);obstacles=[]
    if kind=='narrow_slip':
        for label,x,size,coefficient in [('before',-3.,6.,1.),('patch',3.5,7.,mu),('after',11.5,9.,1.)]:
            block(world,'friction_patch' if label=='patch' else f'floor_{label}',[x,0,-.1],[size,4.,.2],coefficient,'.2 .45 .7 1' if label=='patch' else '.6 .65 .6 1')
    else:block(world,'floor',[5,0,-.1],[22,4,.2],1.,'.6 .65 .6 1')
    for label,xyz,size in [('left_wall',[5,.95,1.25],[20,.2,2.5]),('right_wall',[5,-.95,1.25],[20,.2,2.5]),
                           ('west_cap',[-5.1,0,1.25],[.2,2.1,2.5]),('east_cap',[15.1,0,1.25],[.2,2.1,2.5])]:
        block(world,label,xyz,size);obstacles.append([xyz[0],xyz[1],size[0],size[1]])
    if kind=='narrow_ramp':
        rise=4*math.tan(math.radians(angle))
        for part,x0,x1,z0,z1 in [('up',0,4,0,rise),('down',7,11,rise,0)]:
            path=BASE/'worlds'/f'{name}_{part}.obj';write_wedge(path,x0,x1,z0,z1,half_width=WIDTH/2)
            model=tag(world,'model',name=f'ramp_{part}');tag(model,'static','true');link=tag(model,'link',name='link')
            for typ in ('collision','visual'):
                elem=tag(link,typ,name=typ);tag(tag(tag(elem,'geometry'),'mesh'),'uri',path.resolve().as_uri())
                if typ=='collision':friction(elem,1.)
                else:
                    mat=tag(elem,'material');tag(mat,'ambient','.7 .5 .2 1');tag(mat,'diffuse','.7 .5 .2 1')
        block(world,'platform',[5.5,0,rise/2],[3,WIDTH,rise],1.,'.7 .5 .2 1')
        points=[(2.,0.,0.),(5.5,0.,0.),(9.,0.,0.),(13.,0.,0.)]
    else:points=[(-.8,0.,0.),(3.5,0.,0.),(9.,0.,0.)]
    res=.02;xmin,xmax,ymin,ymax=-6.,16.,-2.01,2.01
    xs=xmin+(np.arange(round((xmax-xmin)/res))+.5)*res;ys=ymax-(np.arange(round((ymax-ymin)/res))+.5)*res
    grid=np.full((len(ys),len(xs)),205,dtype=np.uint8)
    grid[(abs(ys[:,None])<WIDTH/2)&(xs[None,:]>-5)&(xs[None,:]<15)]=254
    for x,y,sx,sy in obstacles:grid[(abs(xs[None,:]-x)<sx/2+1e-9)&(abs(ys[:,None]-y)<sy/2+1e-9)]=0
    return save(name,kind,world,sdf,grid,[xmin,ymin,0.],res,[-3.,0.,0.],points,
        dict(reference_geometry=dict(type='straight',center_y=0.),slope_degrees=angle,patch_friction=mu,
             description='Continuous 1.7 m corridor, including the ramp/platform/low-friction section. No wide bypass.'))

def build_all():
    scenes=[build_turns(),build_straight_terrain('narrow_ramp_6deg','narrow_ramp',6.),build_straight_terrain('narrow_ramp_12deg','narrow_ramp',12.),
            build_straight_terrain('narrow_slip_normal','narrow_slip',mu=1.),build_straight_terrain('narrow_slip_low','narrow_slip',mu=.05),build_straight_terrain('narrow_slip_ice','narrow_slip',mu=.005)]
    preview(scenes)
    return scenes

def preview(scenes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(15,8))
    for ax,meta in zip(axes.ravel(),scenes):
        cfg=yaml.safe_load(Path(meta['map']).read_text());im=Image.open(Path(meta['map']).with_suffix('.pgm'));ox,oy,_=cfg['origin'];r=cfg['resolution']
        ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+im.width*r,oy,oy+im.height*r])
        route=yaml.safe_load(Path(meta['route']).read_text())['waypoints'];xy=[meta['spawn'][:2]]+[[p['x'],p['y']] for p in route]
        if meta['kind']=='narrow_turns':
            centre=capsule(meta['reference_geometry']['radius']);ax.plot(*np.vstack([centre,centre[0]]).T,'g--',lw=1)
        else:
            ax.set_ylim(-1.25,1.25)
            if meta['kind']=='narrow_ramp':ax.add_patch(plt.Rectangle((0,-.85),11,1.7,color='orange',alpha=.3))
            else:ax.add_patch(plt.Rectangle((0,-.85),7,1.7,color='blue',alpha=.15))
        ax.plot(*np.array(xy).T,'r.-',lw=1);ax.set_title(meta['name']+' | clear width 1.70 m');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    fig.tight_layout();fig.savefig(BASE/'narrow_scene_overview.png',dpi=130)
if __name__=='__main__':
    raise SystemExit('Run scripts/build_test_scenes.py to regenerate the complete suite.')
