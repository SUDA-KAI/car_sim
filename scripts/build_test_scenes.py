#!/usr/bin/python3
"""Generate reproducible turning, slope and low-friction patrol test scenes."""
import copy,json,math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import yaml
from PIL import Image,ImageDraw
from build_world import ROOT,tag
BASE=ROOT/'test_scenes'
XMIN,YMIN,XMAX,YMAX=-6.,-7.,16.,7.
RES=.05

def friction(collision,mu):
    ode=tag(tag(tag(collision,'surface'),'friction'),'ode')
    tag(ode,'mu',mu);tag(ode,'mu2',mu)

def block(world,name,xyz,size,mu=1.,color='0.6 0.65 0.7 1',pitch=0.):
    model=tag(world,'model',name=name);tag(model,'static','true')
    tag(model,'pose',' '.join(map(str,[*xyz,0,pitch,0])))
    link=tag(model,'link',name='link')
    for kind in ('collision','visual'):
        part=tag(link,kind,name=kind);tag(tag(tag(part,'geometry'),'box'),'size',' '.join(map(str,size)))
        if kind=='collision':friction(part,mu)
        else:
            mat=tag(part,'material');tag(mat,'ambient',color);tag(mat,'diffuse',color)
    return model

def new_world(name):
    sdf=E.Element('sdf',version='1.8');world=tag(sdf,'world',name='patrol')
    p=tag(world,'physics',name='physics',type='ignored');tag(p,'max_step_size',.002);tag(p,'real_time_factor',1.)
    for key,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        plugin=tag(world,'plugin',filename=f'ignition-gazebo-{key}-system',name=f'ignition::gazebo::systems::{cls}')
        if cls=='Sensors':tag(plugin,'render_engine','ogre2')
    sun=tag(world,'light',name='sun',type='directional');tag(sun,'pose','0 0 15 0 0 0');tag(sun,'diffuse','.9 .9 .9 1');tag(sun,'direction','-.3 .2 -1')
    return sdf,world

def add_robot(world,spawn):
    robot=copy.deepcopy(E.parse(ROOT/'models/patrol_car.sdf').getroot().find('model'))
    robot.find('pose').text=' '.join(map(str,[spawn[0],spawn[1],.025,0,0,spawn[2]]))
    steering=robot.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
    # Keep wheel odometry independent from the true model pose.
    steering.find('odom_topic').text='/sim/wheel_odom'
    steering.find('tf_topic').text='/sim/wheel_tf'
    # Bound acceleration for repeatable traction comparisons.
    tag(steering,'min_acceleration',-.5);tag(steering,'max_acceleration',.4)
    truth=tag(robot,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
    for key,value in [('odom_frame','odom'),('robot_base_frame','base_link'),('dimensions',3),('odom_topic','/sim/ground_truth'),('tf_topic','/sim/truth_tf'),('odom_publish_frequency',30)]:tag(truth,key,value)
    world.append(robot)

def build_scene(name,kind,mu=1.,angle=0.):
    sdf,world=new_world(name);obstacles=[]
    if kind=='slip':
        # Adjacent, coplanar floor slabs: no high-friction floor underneath patch.
        block(world,'floor_before',[-3.,0,-.1],[6.,14.,.2],1.,'.55 .6 .55 1')
        block(world,'friction_patch',[3.5,0,-.1],[7.,14.,.2],mu,'.15 .45 .7 1')
        block(world,'floor_after',[11.5,0,-.1],[9.,14.,.2],1.,'.55 .6 .55 1')
    else:block(world,'floor',[5.,0,-.1],[22.,14.,.2],1.,'.65 .68 .6 1')
    # Boundary walls are reflected identically in the navigation occupancy map.
    for name_wall,xyz,size in [('south',[5,-6.8,1.5],[22,.3,3]),('north',[5,6.8,1.5],[22,.3,3]),
                               ('west',[-5.8,0,1.5],[.3,14,3]),('east',[15.8,0,1.5],[.3,14,3])]:
        block(world,name_wall,xyz,size);obstacles.append([xyz[0],xyz[1],size[0],size[1]])
    if kind=='turns':
        for i,(x,y,sx,sy) in enumerate([(2,0,2,2),(8,1,1.4,3)]):
            block(world,f'block_{i}',[x,y,.75],[sx,sy,1.5]);obstacles.append([x,y,sx,sy])
        spawn=[-2.,-3.,0.]
        points=[(5,-3,0),(12,-3,math.pi/2),(12,4,math.pi),(5,4,math.pi),(-2,4,-math.pi/2),(-2,-3,0)]
        reference='Wide loop with 90-degree turns; two inner obstacles; minimum turning radius 1.5 m.'
    elif kind=='ramp':
        rise=4*math.tan(math.radians(angle));thickness=.18
        # Closed wedge cross-section, exactly at z=0 at either foot, avoiding steps.
        vertices=[(0,-2,0),(0,2,0),(4,2,rise),(4,-2,rise),
                  (7,-2,rise),(7,2,rise),(11,2,0),(11,-2,0)]
        # Build separate up/down closed meshes, platform is a box.
        for part,x0,x1,z0,z1 in [('up',0,4,0,rise),('down',7,11,rise,0)]:
            path=BASE/'worlds'/f'{name}_{part}.obj'
            write_wedge(path,x0,x1,z0,z1)
            mesh_model=tag(world,'model',name=f'ramp_{part}');tag(mesh_model,'static','true');link=tag(mesh_model,'link',name='link')
            for typ in ('collision','visual'):
                elem=tag(link,typ,name=typ);tag(tag(tag(elem,'geometry'),'mesh'),'uri',path.resolve().as_uri())
                if typ=='collision':friction(elem,1.)
                else:
                    mat=tag(elem,'material');tag(mat,'ambient','.7 .5 .2 1');tag(mat,'diffuse','.7 .5 .2 1')
        block(world,'platform',[5.5,0,rise/2],[3.,4.,rise],1.,'.7 .5 .2 1')
        # Side barriers prevent the global planner from bypassing the ramp.
        for y in [-2.35,2.35]:
            block(world,f'rail_{y}',[5.5,y,1.5],[11.6,.3,3]);obstacles.append([5.5,y,11.6,.3])
        spawn=[-3.,0.,0.]
        points=[(2.,0.,0.),(5.5,0.,0.),(9.,0.,0.),(13.,0.,0.)]
        reference=f'{angle:g} deg up/down, run 4 m each, platform 3 m, rise {rise:.3f} m. Route follows traversable terrain.'
    else:
        spawn=[-3.,0.,0.]
        points=[(-.8,0,0),(3.5,0,0),(5.5,1.5,.6),(10,1.5,0)]
        reference=f'Coplanar x=[0,7] patch with mu=mu2={mu}; use slip_normal as A/B control. Not a calibrated tyre model.'
    add_robot(world,spawn)
    image=Image.new('L',(round((XMAX-XMIN)/RES),round((YMAX-YMIN)/RES)),254);draw=ImageDraw.Draw(image)
    for x,y,sx,sy in obstacles:
        # Inclusive rasterization conservatively covers rectangle extents.
        draw.rectangle([(int((x-sx/2-XMIN)/RES),int((YMAX-y-sy/2)/RES)),
                        (int((x+sx/2-XMIN)/RES),int((YMAX-y+sy/2)/RES))],fill=0)
    image.save(BASE/'maps'/f'{name}.pgm')
    map_path=BASE/'maps'/f'{name}.yaml';map_path.write_text(yaml.safe_dump(dict(image=f'{name}.pgm',resolution=RES,origin=[XMIN,YMIN,0.],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))
    route_path=BASE/'routes'/f'{name}.yaml';route_path.write_text(yaml.safe_dump(dict(frame_id='map',waypoints=[dict(x=float(x),y=float(y),yaw=float(yaw)) for x,y,yaw in points]),sort_keys=False))
    path=BASE/'worlds'/f'{name}.sdf';E.indent(sdf);E.ElementTree(sdf).write(path,encoding='utf-8',xml_declaration=True)
    meta=dict(map=str(map_path.resolve()),world=str(path.resolve()),spawn=spawn,route=str(route_path.resolve()),
              has_ground_truth=True,localization='ground_truth',nav_params=str((BASE/'nav2_tests.yaml').resolve()),
              name=name,kind=kind,slope_degrees=angle,patch_friction=mu,obstacles=obstacles,
              description=reference,baseline='Ideal ground-truth localization; static-map obstacles. Physical model, not field validation.')
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2))
    return meta

def build_narrow_scene(name,baseline=False):
    """1.700 m clear gap, conservative full 1.2 x 0.9 m body, same disturbance."""
    sdf,world=new_world(name)
    block(world,'floor',[4.,0.,-.1],[22.,4.,.2],1.,'.65 .68 .6 1')
    obstacles=[]
    # Inner faces at y=+-0.85 and x=-5,13: dimensions are physical, not wall centres.
    for wall,xyz,size in [('left_wall',[4.,.95,.75],[18.,.2,1.5]),
                          ('right_wall',[4.,-.95,.75],[18.,.2,1.5]),
                          ('west_cap',[-5.1,0.,.75],[.2,2.1,1.5]),
                          ('east_cap',[13.1,0.,.75],[.2,2.1,1.5])]:
        block(world,wall,xyz,size);obstacles.append([xyz[0],xyz[1],size[0],size[1]])
    spawn=[-3.,.1,math.radians(3.)]
    add_robot(world,spawn)
    if not baseline:
        plugin=world.find("model[@name='patrol_car']/plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        # Fortress applies these limits to BOTH linear and angular velocity.
        plugin.find('min_velocity').text='-0.8';plugin.find('max_velocity').text='0.8'
        plugin.find('min_acceleration').text='-0.4';plugin.find('max_acceleration').text='0.4'
    res=.02;xmin,xmax,ymin,ymax=-6.,14.,-2.01,2.01
    cols,rows=round((xmax-xmin)/res),round((ymax-ymin)/res)
    xx=xmin+(np.arange(cols)+.5)*res;yy=ymax-(np.arange(rows)+.5)*res
    grid=np.full((rows,cols),205,dtype=np.uint8)
    grid[(abs(yy[:,None])<.85)&(xx[None,:]>-5)&(xx[None,:]<13)]=254
    for x,y,sx,sy in obstacles:
        mask=(abs(xx[None,:]-x)<sx/2+1e-9)&(abs(yy[:,None]-y)<sy/2+1e-9)
        grid[mask]=0
    Image.fromarray(grid).save(BASE/'maps'/f'{name}.pgm')
    map_path=BASE/'maps'/f'{name}.yaml'
    map_path.write_text(yaml.safe_dump(dict(image=f'{name}.pgm',resolution=res,origin=[xmin,ymin,0.],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))
    route=BASE/'routes'/f'{name}.yaml'
    route.write_text(yaml.safe_dump(dict(frame_id='map',waypoints=[dict(x=7.,y=0.,yaw=0.),dict(x=-3.,y=0.,yaw=0.)]),sort_keys=False))
    params=copy.deepcopy(yaml.safe_load((BASE/'nav2_tests.yaml').read_text()))
    if not baseline:
        follow=params['controller_server']['ros__parameters']['FollowPath']
        follow.update(max_vel_x=.2,max_vel_x_backwards=.12,max_vel_theta=.15,acc_lim_x=.15,acc_lim_theta=.35,
                      max_global_plan_lookahead_dist=3.5,global_plan_viapoint_sep=.3,weight_viapoint=20.,
                      min_obstacle_dist=.18,weight_obstacle=100.,weight_optimaltime=1.)
        smoother=params['velocity_smoother']['ros__parameters']
        smoother.update(max_velocity=[.2,0.,.15],min_velocity=[-.12,0.,-.15],max_accel=[.15,0.,.35],max_decel=[-.2,0.,-.35])
        for costmap in ['local_costmap','global_costmap']:
            params[costmap][costmap]['ros__parameters']['resolution']=.02
            params[costmap][costmap]['ros__parameters']['footprint_padding']=.03
    nav=BASE/f'nav2_{name}.yaml';nav.write_text(yaml.safe_dump(params,sort_keys=False))
    path=BASE/'worlds'/f'{name}.sdf';E.indent(sdf);E.ElementTree(sdf).write(path,encoding='utf-8',xml_declaration=True)
    meta=dict(name=name,kind='narrow',world=str(path.resolve()),map=str(map_path.resolve()),spawn=spawn,
              route=str(route.resolve()),nav_params=str(nav.resolve()),has_ground_truth=True,localization='ground_truth',
              clear_width_m=1.7,corridor_center_y=0.,robot_length_m=1.2,robot_width_m=.9,
              wheel_sweep_width_bound_m=2*(.4+.2*math.sin(.75)+.05*math.cos(.75)),obstacles=obstacles,
              baseline='Original test settings' if baseline else 'Symmetric simulator limits and conservative TEB/smoother settings',
              description='1.7 m clear straight corridor; 10 m forward then reverse, no impossible U-turn. Spawn offset 0.10 m, heading offset 3 degrees.')
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2))
    return meta


def write_wedge(path,x0,x1,z0,z1,half_width=2.):
    # Bottom extends 10 cm below ground; slope meets the floor without a raised lip.
    v=np.array([[x0,-half_width,-.1],[x1,-half_width,-.1],[x1,half_width,-.1],[x0,half_width,-.1],
                [x0,-half_width,z0],[x1,-half_width,z1],[x1,half_width,z1],[x0,half_width,z0]])
    faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    with path.open('w') as f:
        for vertex in v:f.write('v '+' '.join(map(str,vertex))+'\n')
        normals=[]
        for face in faces:
            a,b,c=v[list(face)];n=np.cross(b-a,c-a);n/=np.linalg.norm(n);normals.append(n)
        for n in normals:f.write('vn '+' '.join(map(str,n))+'\n')
        for i,face in enumerate(faces):f.write('f '+' '.join(f'{j+1}//{i+1}' for j in face)+'\n')

def main():
    for directory in ['maps','worlds','routes','results']:(BASE/directory).mkdir(parents=True,exist_ok=True)
    params=yaml.safe_load((ROOT/'config/nav2_sim.yaml').read_text())
    for name in ['local_costmap','global_costmap']:
        p=params[name][name]['ros__parameters'];p['plugins']=['static_layer','inflation_layer']
        p['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_subscribe_transient_local=True)
        p.pop('obstacle_layer',None)
    follow=params['controller_server']['ros__parameters']['FollowPath'];follow['max_vel_x']=.45;follow['min_turning_radius']=1.5
    params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']=1.5
    params['velocity_smoother']['ros__parameters']['max_velocity']=[.45,0.,.35]
    (BASE/'nav2_tests.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    specs=[('turns','turns',1.,0.),('ramp_6deg','ramp',1.,6.),('ramp_12deg','ramp',1.,12.),
           ('slip_normal','slip',1.,0.),('slip_low','slip',.05,0.),('slip_ice','slip',.005,0.)]
    manifest=[build_scene(*spec) for spec in specs]
    manifest += [build_narrow_scene('narrow_17_baseline',baseline=True),build_narrow_scene('narrow_17')]
    from build_narrow_suite import build_all
    manifest += build_all()
    (BASE/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print('\n'.join(f"{m['name']}: {m['description']}" for m in manifest))
    preview(manifest)

def preview(manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,3,figsize=(15,17))
    for ax,m in zip(axes.ravel(),manifest):
        im=Image.open(BASE/'maps'/f"{m['name']}.pgm");cfg=yaml.safe_load(Path(m['map']).read_text());ox,oy,_=cfg['origin'];resolution=cfg['resolution'];ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+im.width*resolution,oy,oy+im.height*resolution])
        route=yaml.safe_load(Path(m['route']).read_text())['waypoints'];xy=[m['spawn'][:2]]+[[p['x'],p['y']] for p in route]
        ax.plot(*np.array(xy).T,'r.-');ax.set_title(m['name']);ax.set_aspect('equal')
        if m['kind'] in ('ramp','narrow_ramp'):ax.add_patch(plt.Rectangle((0,-m.get('clear_width_m',4)/2),11,m.get('clear_width_m',4),color='orange',alpha=.3))
        if m['kind'] in ('slip','narrow_slip'):ax.add_patch(plt.Rectangle((0,-6.5),7,13,color='blue',alpha=.15))
        ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
    axes.ravel()[-1].axis('off')
    fig.tight_layout();fig.savefig(BASE/'scene_overview.png',dpi=130)
    fig,ax=plt.subplots(figsize=(10,4))
    for degree in [6,12]:
        height=4*math.tan(math.radians(degree));ax.plot([-3,0,4,7,11,13],[0,0,height,height,0,0],label=f'{degree} deg, rise {height:.2f} m')
    ax.set_xlabel('x (m)');ax.set_ylabel('surface height (m)');ax.set_title('Ramp side profiles | 4 m up + 3 m platform + 4 m down');ax.legend();ax.grid(True);fig.tight_layout();fig.savefig(BASE/'ramp_profiles.png',dpi=140)
if __name__=='__main__':main()
