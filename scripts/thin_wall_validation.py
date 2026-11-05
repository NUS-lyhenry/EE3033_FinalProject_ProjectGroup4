"""Independent raw-map continuity metrics and deterministic laser fixtures.

No map modification, closing, dilation, planner input, or mission truth injection.
The truth segments are used only to generate sensor rays and score output maps.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def segment_distances(points, segments):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    segments=np.asarray(segments,dtype=float).reshape(-1,2,2)
    if not len(points):return np.empty((0,len(segments)))
    start=segments[:,0];edge=segments[:,1]-start
    along=np.clip(np.sum((points[:,None]-start)*edge,axis=2)/np.sum(edge*edge,axis=1),0,1)
    return np.linalg.norm(points[:,None]-(start+along[:,:,None]*edge),axis=2)


def rotate(points, angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.asarray(points)@np.array([[c,s],[-s,c]])


def fixture(angle_degrees=0, sides='two', opening_width=.30):
    if sides not in ('one','two'):raise ValueError('Observation sides must be one or two')
    angle=math.radians(angle_degrees);half=opening_width/2
    boundary=[[[-2.,-1.5],[2.,-1.5]],[[2.,-1.5],[2.,1.5]],
        [[2.,1.5],[-2.,1.5]],[[-2.,1.5],[-2.,-1.5]]]
    partition=[[[-1.5,0.],[-half,0.]],[[half,0.],[1.5,0.]]]
    # Identical frozen observations for both threshold runs. Repeated static
    # scans exercise raster evidence; small yaw sweeps exercise angular bins.
    poses=[]
    for y in ([-.75] if sides=='one' else [-.75,.75]):
        for x in (-1.15,-.55,0.,.55,1.15):
            for yaw in (-.12,0.,.12):
                xy=rotate([[x,y]],angle)[0]
                poses.append([float(xy[0]),float(xy[1]),yaw+angle])
    return dict(case_id=f'wall_{angle_degrees}_{sides}',
        opening_width=opening_width,
        walls=rotate(np.asarray(boundary+partition).reshape(-1,2),angle).reshape(-1,2,2).tolist(),
        scored_walls=rotate(np.asarray(partition).reshape(-1,2),angle).reshape(-1,2,2).tolist(),
        opening=rotate([[0.,-.45],[0.,.45]],angle).tolist(),poses=poses)


def fixture_scans(case, beams=450, mask_radius=.20):
    # Use the same ray caster as the sensor model, without starting ROS or a
    # simulated chassis. The hashes prove both configurations saw the same data.
    from simulation_geometry import raycast
    from ee3033_project.scan_core import resample
    angles=-math.pi+np.arange(beams)*math.tau/beams
    scans=[]
    for pose in case['poses']:
        ranges=raycast(pose[:2],angles+pose[2],case['walls']).tolist()
        scans.append(resample(ranges,-math.pi,math.tau/beams,.08,8.,beams,mask_radius))
    encoded=json.dumps([[None if math.isnan(x) else x for x in r] for r in scans],separators=(',',':'))
    return scans,hashlib.sha256(encoded.encode()).hexdigest()


def longest_missing(flags, step):
    longest=current=0
    for supported in flags:
        current=0 if supported else current+1
        longest=max(longest,current)
    return longest*step


def evaluate_map(data, case, footprint_radius=.116801):
    resolution=float(data['resolution']);width=int(data['width']);height=int(data['height'])
    grid=np.asarray(data['cells']).reshape(height,width)
    origin=data['origin'];angle=float(origin[2]) if len(origin)>2 else 0.
    rows,cols=np.nonzero(grid>=50)
    occupied=rotate(np.column_stack(((cols+.5)*resolution,(rows+.5)*resolution)),angle)+origin[:2]
    walls=[]
    for a,b in np.asarray(case['scored_walls']):
        count=max(1,math.ceil(np.linalg.norm(b-a)/(resolution/2)))
        samples=np.linspace(a,b,count+1)
        distances=np.min(np.linalg.norm(samples[:,None]-occupied,axis=2),axis=1) if len(occupied) else np.full(count+1,np.inf)
        supported=distances<=1.5*resolution
        walls.append(dict(supported_fraction=float(np.mean(supported)),
            longest_missing_m=longest_missing(supported,float(np.linalg.norm(b-a)/count)),samples=len(samples)))
    ghost_distance=np.min(segment_distances(occupied,case['walls']),axis=1) if len(occupied) else np.array([])
    ghosts=int(np.count_nonzero(ghost_distance>2*resolution))
    # Require an actually observed, footprint-wide free corridor through the
    # true opening. An unknown gap or a cosmetically open drawing cannot pass.
    a,b=np.asarray(case['opening']);direction=b-a;length=np.linalg.norm(direction)
    normal=np.array([-direction[1],direction[0]])/length
    sample_grid=np.array([a+t*direction+d*normal for t in np.linspace(0,1,math.ceil(length/(resolution/4))+1)
        for d in np.linspace(-footprint_radius,footprint_radius,math.ceil(2*footprint_radius/(resolution/4))+1)])
    def corridor_free_fraction(offset):
        local=rotate(sample_grid+offset*normal-np.asarray(origin[:2]),-angle)
        cells=np.floor(local/resolution).astype(int)
        inside=(cells[:,0]>=0)&(cells[:,0]<width)&(cells[:,1]>=0)&(cells[:,1]<height)
        free=np.zeros(len(cells),dtype=bool)
        free[inside]=grid[cells[inside,1],cells[inside,0]]==0
        return float(np.mean(free))
    # Rasterized jambs need not be symmetric about the exact truth centerline.
    # Search only within the independently known physical doorway margin:
    # every candidate remains a full-footprint straight passage through it.
    margin=max(0.,float(case['opening_width'])/2-footprint_radius)
    offsets=[0.,*np.linspace(-margin,margin,max(2,math.ceil(2*margin/(resolution/8))+1))]
    fractions=[corridor_free_fraction(offset) for offset in offsets]
    best=int(np.argmax(fractions));opening_free=fractions[best]==1.
    passed=bool(walls and all(w['supported_fraction']>=.95 and w['longest_missing_m']<=2*resolution+1e-9 for w in walls)
        and ghosts==0 and opening_free)
    return dict(passed=passed,resolution=resolution,occupied_cells=len(occupied),walls=walls,
        ghost_cells=ghosts,ghost_distance_limit_m=2*resolution,opening_free=opening_free,
        opening_free_fraction=fractions[best],opening_lateral_offset_m=float(offsets[best]),
        opening_centerline_free_fraction=fractions[0],opening_width=float(case['opening_width']),
        footprint_radius=footprint_radius,
        support_distance_limit_m=1.5*resolution,maximum_missing_m=2*resolution)


def cases(path=None, opening_width=None):
    path=Path(path) if path else Path(__file__).resolve().parents[1]/'tests/fixtures/thin_wall_cases.json'
    config=json.loads(path.read_text())
    if opening_width is not None:config['opening_width']=opening_width
    return config,[fixture(angle,sides,config['opening_width'])
        for angle in config['angles_degrees'] for sides in config['observation_sides']]
