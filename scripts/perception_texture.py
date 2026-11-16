"""Paint existing scene visuals for stereo tracking; collisions and widths unchanged."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from build_sensor_description import ROOT


def material(visual,texture):
    previous=visual.find('material')
    if previous is not None:visual.remove(previous)
    m=ET.SubElement(visual,'material')
    for key in ('ambient','diffuse'):ET.SubElement(m,key).text='1 1 1 1'
    metal=ET.SubElement(ET.SubElement(m,'pbr'),'metal')
    ET.SubElement(metal,'albedo_map').text=texture.resolve().as_uri()
    ET.SubElement(metal,'roughness').text='0.9';ET.SubElement(metal,'metalness').text='0'


def paint_scene(source,output):
    """Deterministic nonsemantic texture, never a known-pose fiducial map."""
    directory=ROOT/'models/perception_textures';directory.mkdir(exist_ok=True)
    rng=np.random.default_rng(73119)
    texture=directory/'wall_features.png'
    if not texture.exists():
        cells=rng.integers(0,2,(48,768,1),dtype=np.uint8)*180+40
        rgb=np.repeat(cells,3,axis=2)
        Image.fromarray(rgb).resize((6144,384),Image.NEAREST).save(texture)
    floor_texture=directory/'floor_features.png'
    if not floor_texture.exists():
        cells=rng.integers(0,2,(384,384,1),dtype=np.uint8)*150+50
        Image.fromarray(np.repeat(cells,3,axis=2)).resize((3072,3072),Image.NEAREST).save(floor_texture)
    tree=ET.parse(source);world=tree.find('world')
    count=0
    for model in world.findall('model'):
        if model.get('name') in ('inner_island','outer_wall'):
            visual=model.find('link/visual');uri=visual.find('geometry/mesh/uri')
            path=Path(uri.text.removeprefix('file://'));lines=path.read_text().splitlines()
            vertices=[list(map(float,line.split()[1:])) for line in lines if line.startswith('v ')]
            verts=np.array(vertices)
            # Existing mesh has 258 points per capsule boundary, repeated for
            # bottom/top and inner/outer shells. Geometry is copied exactly.
            n=258;boundary=verts[:n,:2]
            lengths=np.r_[0,np.linalg.norm(np.diff(boundary,axis=0),axis=1).cumsum()]
            total=lengths[-1]+np.linalg.norm(boundary[-1]-boundary[0])
            uv=[f'vt {lengths[i%n]/total:.9f} {p[2]/2:.9f}' for i,p in enumerate(verts)]
            result=[line for line in lines if not line.startswith('f ')]+uv
            for line in lines:
                if line.startswith('f '):
                    result.append('f '+' '.join(f'{token.split("/")[0]}/{token.split("/")[0]}/{token.split("/")[-1]}' for token in line.split()[1:]))
            target=directory/(path.stem+'_textured.obj');target.write_text('\n'.join(result)+'\n')
            uri.text=target.resolve().as_uri();material(visual,texture);count+=1
        elif model.get('name')=='floor':
            for visual in model.findall('link/visual'):material(visual,floor_texture)
    ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)
    return count
