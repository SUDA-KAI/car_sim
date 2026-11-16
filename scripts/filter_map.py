#!/usr/bin/python3
"""Conservative occupancy cleanup in metres; preserve source and explicit unknown."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
import yaml
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]

def filter_occupancy(occupied, unknown, resolution, min_area=0.004, radius=0.04):
    count,labels,stats,_=cv2.connectedComponentsWithStats(occupied.astype(np.uint8),8)
    cutoff=max(1,int(round(min_area/resolution**2)))
    keep=np.array([False]+[stats[i,cv2.CC_STAT_AREA]>=cutoff for i in range(1,count)])
    retained=keep[labels]
    clean=retained.astype(np.uint8)
    r=max(0,int(round(radius/resolution)))
    if r:
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1))
        clean=cv2.morphologyEx(clean,cv2.MORPH_CLOSE,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
    # Closing adds at most radius-scale bridges, never erode retained thin walls.
    result=((clean>0)|retained)&~unknown
    return result,dict(components_before=count-1,components_removed=int(sum(~keep[1:])),
                       removed_cells=int((occupied&~result).sum()),added_cells=int((result&~occupied).sum()),
                       min_area_m2=min_area,radius_m=radius)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,default=ROOT.parent/'src/robot_core/robot_bringup/maps/map_20260817_154310.yaml')
    parser.add_argument('--output',type=Path,default=ROOT/'maps/map_20260817_154310_filtered.yaml')
    parser.add_argument('--radius',type=float,default=.04)
    parser.add_argument('--min-area',type=float,default=.004)
    a=parser.parse_args()
    if a.radius<0 or a.min_area<0:parser.error('radius/min-area must be nonnegative')
    c=yaml.safe_load(a.map.read_text()); im=np.asarray(Image.open(a.map.parent/c['image']).convert('L'))
    prob=im/255. if c.get('negate',0) else 1-im/255.
    occ=prob>c['occupied_thresh']
    # Source uses 205 for unobserved pixels, despite a free_thresh that mislabels them.
    unknown=(im==205)|((prob>=c['free_thresh'])&~occ)
    cleaned,stats=filter_occupancy(occ,unknown,float(c['resolution']),a.min_area,a.radius)
    output=np.full_like(im,254);output[unknown]=205;output[cleaned]=0
    a.output.parent.mkdir(parents=True,exist_ok=True)
    Image.fromarray(output).save(a.output.with_suffix('.pgm'))
    c.update(image=a.output.with_suffix('.pgm').name,negate=0,mode='trinary',free_thresh=.196,occupied_thresh=.65)
    a.output.write_text(yaml.safe_dump(c,sort_keys=False))
    stats.update(source=str(a.map.resolve()),output=str(a.output.resolve()),unknown_cells=int(unknown.sum()),
                 note='Gray 205 preserved as unknown; free_thresh normalized to 0.196. Closing, not Gaussian blur.')
    a.output.with_suffix('.json').write_text(json.dumps(stats,indent=2)); print(json.dumps(stats,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(16,6))
    axes[0].imshow(im,cmap='gray',vmin=0,vmax=255);axes[0].set_title('Original')
    axes[1].imshow(output,cmap='gray',vmin=0,vmax=255);axes[1].set_title('Filtered: 4 cm closing, small speckles removed')
    delta=np.zeros((*im.shape,3),dtype=np.uint8)+230
    delta[occ&~cleaned]=[0,100,255];delta[cleaned&~occ]=[255,50,0]
    axes[2].imshow(delta);axes[2].set_title('Blue: removed | Red: added')
    for ax in axes:ax.set_xlim(500,1500);ax.set_ylim(1850,700);ax.axis('off')
    fig.tight_layout();fig.savefig(a.output.with_suffix('.png'),dpi=130)
if __name__=='__main__':main()
