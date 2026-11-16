"""Planar corridor geometry and projected 3D vehicle-envelope bounds."""
import math

def point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];d=dx*dx+dy*dy
    u=0. if d==0 else max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d))
    return math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)

def hull(points):
    points=sorted(set(points))
    if len(points)<=2:return points
    cross=lambda o,a,b:(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    low=[];high=[]
    for p in points:
        while len(low)>1 and cross(low[-2],low[-1],p)<=1e-12:low.pop()
        low.append(p)
    for p in reversed(points):
        while len(high)>1 and cross(high[-2],high[-1],p)<=1e-12:high.pop()
        high.append(p)
    return low[:-1]+high[:-1]

def segment_distance(a,b,c,d):
    cross=lambda p,q,r:(q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    o=[cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)]
    if o[0]*o[1]<=0 and o[2]*o[3]<=0:
        if max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0]))+1e-12 and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))+1e-12:return 0.
    return min(point_segment_distance(a,c,d),point_segment_distance(b,c,d),point_segment_distance(c,a,b),point_segment_distance(d,a,b))

def projected_box(x,y,q,length,width,height=.495):
    # Box reference is the STL ground origin, not its centre of mass.
    qx,qy,qz,qw=q
    r00=1-2*(qy*qy+qz*qz);r01=2*(qx*qy-qz*qw);r02=2*(qx*qz+qy*qw)
    r10=2*(qx*qy+qz*qw);r11=1-2*(qx*qx+qz*qz);r12=2*(qy*qz-qx*qw)
    return hull([(x+r00*xx+r01*yy+r02*zz,y+r10*xx+r11*yy+r12*zz)
                 for xx in [-length/2,length/2] for yy in [-width/2,width/2] for zz in [0.,height]])

def envelope_clearance(scene,x,y,q,body_width=None):
    poly=projected_box(x,y,q,scene['robot_length_m'],body_width or scene['robot_width_m'],scene.get('robot_height_m',.495))
    geometry=scene.get('reference_geometry',{'type':'straight','center_y':scene.get('corridor_center_y',0.)})
    if geometry['type']=='straight':
        cy=geometry.get('center_y',0.);h=scene['clear_width_m']/2
        return min(cy+h-max(p[1] for p in poly),min(p[1] for p in poly)-(cy-h))
    a=(geometry['left_x'],0.);b=(geometry['right_x'],0.)
    max_r=max(point_segment_distance(p,a,b) for p in poly)
    def inside(p):
        return all((v[0]-u[0])*(p[1]-u[1])-(v[1]-u[1])*(p[0]-u[0])>=-1e-12 for u,v in zip(poly,poly[1:]+poly[:1]))
    min_r=0. if inside(a) or inside(b) else min(segment_distance(u,v,a,b) for u,v in zip(poly,poly[1:]+poly[:1]))
    inner=geometry['radius']-scene['clear_width_m']/2
    outer=geometry['radius']+scene['clear_width_m']/2
    return min(min_r-inner,outer-max_r)

def center_offset(scene,x,y):
    geometry=scene.get('reference_geometry',{'type':'straight','center_y':scene.get('corridor_center_y',0.)})
    if geometry['type']=='straight':return y-geometry.get('center_y',0.)
    return point_segment_distance((x,y),(geometry['left_x'],0.),(geometry['right_x'],0.))-geometry['radius']

def reference_heading(scene,x,y):
    geometry=scene.get('reference_geometry',{'type':'straight'})
    if geometry['type']=='straight':return 0.
    left,right=geometry['left_x'],geometry['right_x']
    if x>=right:return math.atan2(y,x-right)+math.pi/2
    if x<=left:return math.atan2(y,x-left)+math.pi/2
    return math.pi if y>=0 else 0.
