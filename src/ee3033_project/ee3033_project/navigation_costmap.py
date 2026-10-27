"""Nonblocking snapshots of Nav2's actual inflated navigation envelope."""
import math
import time
import numpy as np
from nav2_msgs.srv import GetCostmap
from .core import quaternion_yaw


class NavigationCostmap:
    def __init__(self,node):
        self.client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
        self.future=None
        self.stamp=self.requested=-math.inf
        self.map=None

    def poll(self,now):
        if self.future is not None:
            if not self.future.done():
                if now-self.requested<=2.:return
                self.future.cancel()
                self.future=None
            if self.future is None:
                return
            try:
                msg=self.future.result().map
                if msg.header.frame_id=='map' and msg.metadata.resolution>0:
                    self.map=msg;self.stamp=now
            except Exception:pass
            self.future=None
        if now-self.requested>=1. and self.client.service_is_ready():
            self.requested=now
            self.future=self.client.call_async(GetCostmap.Request())

    def allowed(self,x,y):
        msg=self.map
        if msg is None:return False if self.client.service_is_ready() else None
        if time.monotonic()-self.stamp>3.:return False
        m=msg.metadata;o=m.origin;a=quaternion_yaw(o.orientation)
        dx,dy=x-o.position.x,y-o.position.y
        col=math.floor((math.cos(a)*dx+math.sin(a)*dy)/m.resolution)
        row=math.floor((-math.sin(a)*dx+math.cos(a)*dy)/m.resolution)
        return bool(0<=row<m.size_y and 0<=col<m.size_x and msg.data[row*m.size_x+col]<253)

    def mask(self,shape,resolution,origin):
        msg=self.map
        if msg is None:return np.ones(shape,dtype=bool)
        if time.monotonic()-self.stamp>3.:return np.zeros(shape,dtype=bool)
        rows,cols=np.indices(shape);a=origin[2]
        x=origin[0]+np.cos(a)*(cols+.5)*resolution-np.sin(a)*(rows+.5)*resolution
        y=origin[1]+np.sin(a)*(cols+.5)*resolution+np.cos(a)*(rows+.5)*resolution
        m=msg.metadata;o=m.origin;b=quaternion_yaw(o.orientation)
        cc=np.floor((np.cos(b)*(x-o.position.x)+np.sin(b)*(y-o.position.y))/m.resolution).astype(int)
        rr=np.floor((-np.sin(b)*(x-o.position.x)+np.cos(b)*(y-o.position.y))/m.resolution).astype(int)
        inside=(rr>=0)&(cc>=0)&(rr<m.size_y)&(cc<m.size_x)
        values=np.asarray(msg.data).reshape(m.size_y,m.size_x)
        mask=np.zeros(shape,dtype=bool);mask[inside]=values[rr[inside],cc[inside]]<253
        return mask
