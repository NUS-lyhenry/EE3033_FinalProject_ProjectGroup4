"""Map-backed, uncertainty-inflated clearance for bounded recovery reversing."""
import math
import numpy as np


def reverse_clearance(grid, resolution, origin, pose, covariance, distance=.24,
                      radius=.15, reserve=.04):
    """Return (allowed, reason, inflated radius); unknown space fails closed.

    The map is uninflated occupancy. Include every cell intersecting the swept
    disc, not just its centre, and a two-sigma localization envelope.
    """
    try:
        grid=np.asarray(grid)
        covariance=np.asarray(covariance,dtype=float)
        if (grid.ndim!=2 or covariance.shape!=(3,3) or resolution<=0
                or not np.isfinite([resolution,*origin,*pose,distance,radius,reserve]).all()
                or not np.isfinite(covariance).all() or distance<=0 or radius<=0 or reserve<0
                or not np.isfinite(grid).all() or np.any((grid < -1)|(grid > 100))):
            return False,'Invalid reverse geometry',None
        if not np.allclose(covariance,covariance.T) or np.linalg.eigvalsh(covariance).min() < -1e-9:
            return False,'Invalid localization covariance',None
        sigma=math.sqrt(max(0.,float(np.linalg.eigvalsh(covariance[:2,:2]).max())))
        yaw_sigma=math.sqrt(max(0.,covariance[2,2]))
        if sigma>.12 or yaw_sigma>.20:
            return False,'Localization uncertainty too high for reverse',None
        envelope=radius+reserve+2*sigma+distance*math.sin(min(math.pi/2,2*yaw_sigma))
        ox,oy,a=origin;c,s=math.cos(a),math.sin(a)
        # Half a diagonal accounts for occupied cells intersecting the disc.
        cell_radius=envelope+resolution/math.sqrt(2)
        for step in np.linspace(0,distance,max(2,math.ceil(distance/(resolution/2))+1)):
            x=pose[0]-step*math.cos(pose[2]);y=pose[1]-step*math.sin(pose[2])
            col=(c*(x-ox)+s*(y-oy))/resolution-.5
            row=(-s*(x-ox)+c*(y-oy))/resolution-.5
            rr=math.ceil(cell_radius/resolution)
            r0,r1=math.floor(row)-rr,math.ceil(row)+rr
            c0,c1=math.floor(col)-rr,math.ceil(col)+rr
            if min(r0,c0)<0 or r1>=grid.shape[0] or c1>=grid.shape[1]:
                return False,'Reverse envelope outside mapped area',envelope
            rows,cols=np.mgrid[r0:r1+1,c0:c1+1]
            mask=((rows-row)**2+(cols-col)**2)*resolution**2<=cell_radius**2
            values=grid[r0:r1+1,c0:c1+1][mask]
            if np.any(values<0):return False,'Unknown space behind robot',envelope
            if np.any(values>=65):return False,'Obstacle in reverse envelope',envelope
        return True,'Mapped reverse corridor clear',envelope
    except (ValueError,TypeError,IndexError):
        return False,'Invalid reverse geometry',None
