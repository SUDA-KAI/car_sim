"""Validate and group patrol poses without depending on ROS."""
def route_groups(route, mode):
    points=route['waypoints']
    if not points:raise ValueError('Empty patrol route')
    if mode=='waypoints':return [[point] for point in points]
    if mode!='through_poses':raise ValueError('Unknown patrol mode')
    ends=route.get('segment_end_indices',[len(points)])
    if not ends or ends[-1]!=len(points) or any(type(end) is not int for end in ends):
        raise ValueError('Segment ends must be integer indices ending at the route length')
    starts=[0]+ends[:-1]
    if any(end<=start or end>len(points) for start,end in zip(starts,ends)):
        raise ValueError('Segment ends must increase within the route')
    return [points[start:end] for start,end in zip(starts,ends)]
