"""Convert dashboard mounting coordinates and add ideal fan range sensors."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

DEFAULT_SETTINGS = Path(__file__).resolve().parents[1] / 'config/ultrasonic_settings.json'


def load_mounting(path=DEFAULT_SETTINGS, height=0.49):
    data = json.loads(Path(path).read_text())
    if data.get('version') != 1 or data.get('origin') != 'rear_left' or not data.get('configured'):
        raise ValueError('Expected configured version-1 rear_left mounting settings')
    if len(data['sensors']) != 4:
        raise ValueError('Exactly four ultrasonic sensors are required')
    length, width = float(data['length_m']), float(data['width_m'])
    if not all(math.isfinite(v) and v > 0 for v in (length, width, height)):
        raise ValueError('Vehicle dimensions and sensor height must be finite and positive')
    mounting = []
    for index, raw in enumerate(data['sensors'], 1):
        values = [float(raw[k]) for k in ('longitudinal_m', 'right_m', 'heading_deg', 'fov_deg', 'max_range_m')]
        longitudinal, right, heading, fov, maximum = values
        if not all(math.isfinite(v) for v in values) or not 0 < fov <= 180 or not maximum > .03:
            raise ValueError(f'Invalid geometry for ultrasonic_{index}')
        mounting.append(dict(frame=f'ultrasonic_{index}', topic=f'/ultrasonic_{index}',
                             x=longitudinal-length/2, y=width/2-right, z=float(height),
                             yaw=-math.radians(heading), field_of_view=math.radians(fov),
                             min_range=.03, max_range=maximum))
    return mounting


def attach_sensors(world_path, output_path, mounting):
    tree = ET.parse(world_path)
    base = tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
    if base is None:
        raise ValueError('World must contain patrol_car/base_link')
    for sensor in mounting:
        name = sensor['frame']
        if base.find(f"sensor[@name='{name}']") is not None:
            raise ValueError(f'Duplicate sensor {name}')
        pose = ' '.join(map(str, [sensor['x'], sensor['y'], sensor['z'], sensor.get('roll',0), sensor.get('pitch',0), sensor['yaw']]))
        element = ET.SubElement(base, 'sensor', name=name, type='gpu_lidar')
        for key, value in [('pose',pose), ('topic',f'/sim/{name}/scan'), ('update_rate','10'),
                           ('always_on','true'), ('ignition_frame_id',name)]:
            ET.SubElement(element,key).text=value
        lidar=ET.SubElement(element,'lidar');scan=ET.SubElement(lidar,'scan')
        horizontal=ET.SubElement(scan,'horizontal')
        for key,value in [('samples',61),('resolution',1),('min_angle',-sensor['field_of_view']/2),('max_angle',sensor['field_of_view']/2)]:
            ET.SubElement(horizontal,key).text=str(value)
        limits=ET.SubElement(lidar,'range')
        for key,value in [('min',sensor['min_range']),('max',sensor['max_range']),('resolution',.001)]:
            ET.SubElement(limits,key).text=str(value)
        # Visual marker only: no extra inertia, collisions or self-occluding housing.
        visual=ET.SubElement(base,'visual',name=f'{name}_marker')
        ET.SubElement(visual,'pose').text=pose
        geometry=ET.SubElement(visual,'geometry');sphere=ET.SubElement(geometry,'sphere')
        ET.SubElement(sphere,'radius').text='.008'
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text='0 0.8 1 1'
        ET.SubElement(material,'diffuse').text='0 0.8 1 1'
    tree.write(output_path,encoding='utf-8',xml_declaration=True)


def ultrasonic_layer_params(params,mounting=None,mode='range'):
    params=copy.deepcopy(params)
    local=params['local_costmap']['local_costmap']['ros__parameters']
    plugins=local['plugins']
    if 'ultrasonic_layer' not in plugins:
        plugins.insert(plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins),'ultrasonic_layer')
    if mode=='range':
        local['ultrasonic_layer']=dict(plugin='nav2_costmap_2d::RangeSensorLayer',enabled=True,
            topics=[f'/ultrasonic_{i}/navigation_range' for i in range(1,5)],
            input_sensor_type='VARIABLE',clear_on_max_reading=True,
            clear_threshold=.2,mark_threshold=.8,inflate_cone=1.0,phi=1.2,
            no_readings_timeout=1.0)
        return params
    if mode!='arcs':raise ValueError('ultrasonic fusion must be range or arcs')
    layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,combination_method=1,
               footprint_clearing_enabled=True,min_obstacle_height=0.0,max_obstacle_height=2.5)
    names=[]
    mounting=mounting or load_mounting()
    for i in range(1,5):
        for kind,marking in [('obstacles',True),('clearing',False)]:
            name=f'ultrasonic_{i}_{kind}';names.append(name)
            layer[name]=dict(topic=f'/ultrasonic_{i}/{kind}',sensor_frame=f'ultrasonic_{i}',
                data_type='PointCloud2',marking=marking,clearing=True,obstacle_min_range=0.0,
                min_obstacle_height=0.0,max_obstacle_height=2.5,
                obstacle_max_range=mounting[i-1]['max_range']+.001,raytrace_min_range=0.0,
                raytrace_max_range=mounting[i-1]['max_range']+.03,
                observation_persistence=0.0,expected_update_rate=0.0)
    layer['observation_sources']=' '.join(names)
    local['ultrasonic_layer']=layer
    return params


def finite_navigation_range(distance,minimum,maximum):
    """Humble RangeSensorLayer rejects +/-inf for variable-range sensors."""
    if (math.isnan(distance) or not math.isfinite(minimum) or not math.isfinite(maximum)
            or not 0<=minimum<maximum):return None
    if math.isfinite(distance) and not minimum<=distance<=maximum:return None
    return max(minimum,min(maximum,distance))


def fan_distance(ranges, minimum, maximum):
    """Closest valid horizontal ray; NaNs are unknown, never free-space evidence."""
    if not ranges or any(math.isnan(v) for v in ranges):
        return None
    if any(v == -math.inf or (math.isfinite(v) and v < minimum) for v in ranges):
        return -math.inf
    finite=[v for v in ranges if math.isfinite(v) and v <= maximum]
    return min(finite) if finite else math.inf


def range_arc(distance, minimum, maximum, field_of_view, samples=31):
    """An echo has unknown bearing: conservatively mark the whole possible arc."""
    if (math.isnan(distance) or not all(math.isfinite(v) for v in (minimum,maximum,field_of_view))
            or not 0<=minimum<maximum or not 0<field_of_view<=math.pi or samples<2):
        return [],False
    marking=distance < maximum
    radius=max(minimum,min(maximum,distance))
    points=[(radius*math.cos(-field_of_view/2+field_of_view*i/(samples-1)),
             radius*math.sin(-field_of_view/2+field_of_view*i/(samples-1)),0.0)
            for i in range(samples)]
    return points,marking
