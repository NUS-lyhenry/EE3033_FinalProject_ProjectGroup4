"""Physical-only unicycle dynamics and continuous wall collision checks.

No map, navigation state, route or completion information is consumed here.
"""
import math
import numpy as np
from simulation_geometry import clearance

PROFILES = {
    'ideal': dict(command_delay=0., linear_deadband=0., angular_deadband=0., frozen=False),
    'delayed_deadband': dict(command_delay=.08, linear_deadband=.025, angular_deadband=.08, frozen=False),
    'frozen': dict(command_delay=0., linear_deadband=0., angular_deadband=0., frozen=True),
}


def motion_profile(value='ideal'):
    if isinstance(value,str):
        if value not in PROFILES:raise ValueError('Unknown simulated motion profile')
        return dict(PROFILES[value])
    if not isinstance(value,dict) or set(value)-set(PROFILES['ideal']):
        raise ValueError('Invalid simulated motion profile')
    result=dict(PROFILES['ideal'],**value)
    for key in ('command_delay','linear_deadband','angular_deadband'):
        v=result[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<=v<=2:
            raise ValueError('Invalid simulated '+key)
    if type(result['frozen']) is not bool:raise ValueError('frozen must be boolean')
    return result


def advance_motion(pose, command, duration, walls, radius):
    """Integrate all elapsed time; reject every swept-disc wall intersection.

    Small exact-arc substeps prevent crossing thin wall segments even when a
    delayed timer spans a long distance. Rotation of a circular body remains
    possible when a translational step would penetrate a wall.
    """
    if duration<0 or not all(math.isfinite(v) for v in (*pose,*command,duration,radius)) or radius<=0:
        raise ValueError('Invalid physical integration input')
    x,y,a=map(float,pose);v,w=command;rejected=False
    count=max(1,math.ceil(duration/.02),math.ceil(abs(v)*duration/.004),math.ceil(abs(w)*duration/.04))
    dt=duration/count
    for _ in range(count):
        b=a+w*dt
        if abs(w)<1e-10:nx,ny=x+v*math.cos(a)*dt,y+v*math.sin(a)*dt
        else:nx,ny=x+v/w*(math.sin(b)-math.sin(a)),y-v/w*(math.cos(b)-math.cos(a))
        # Check the whole connector conservatively, not only its endpoint.
        segment=math.hypot(nx-x,ny-y)
        samples=np.linspace([x,y],[nx,ny],max(2,math.ceil(segment/.002)+1))
        if v and any(clearance(point,walls)<radius for point in samples):
            rejected=True
        else:x,y=nx,ny
        a=b
    return [x,y,a],rejected
