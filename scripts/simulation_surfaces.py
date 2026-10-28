"""Lightweight world-anchored surfaces for the simulated optical camera.

Tile dimensions are physical metres; this renderer supplies camera pixels only.
It does not publish occupancy, detections, positions or mission outcomes.
"""
import cv2
import numpy as np

TILE_SIZE_M = .30
GROUT_WIDTH_M = .0025
WALL_HEIGHT_M = .75


def surface_image(translation, rotation, intrinsics, walls, width=640, height=480):
    # Render at half resolution to keep CPU available for real YOLO and Nav2.
    h,w=height//2,width//2
    vv,uu=np.mgrid[:h,:w]
    pixels=np.stack([(uu+.5)*2,(vv+.5)*2,np.ones((h,w))],axis=-1)
    rays=(pixels@np.linalg.inv(intrinsics).T)@rotation.T
    origin=np.asarray(translation,dtype=float)
    sky=np.clip(201-vv/h*18,0,255)
    image=np.stack([sky,sky+3,sky+5],axis=-1)
    depth=np.full((h,w),np.inf)
    with np.errstate(divide='ignore',invalid='ignore'):
        floor_t=-origin[2]/rays[...,2]
    floor=(rays[...,2]<-1e-6)&(floor_t>0)&(floor_t<12.)
    safe_t=np.where(floor,floor_t,0.)
    xy=origin[:2]+rays[...,:2]*safe_t[...,None]
    # Light warm ceramic, 30 cm squares with 2.5 mm grey grout. Small coherent
    # variation makes motion visible while keeping an uncluttered detector view.
    fraction=np.mod(xy,TILE_SIZE_M)
    seam_distance=np.min(np.minimum(fraction,TILE_SIZE_M-fraction),axis=-1)
    footprint=np.clip(safe_t/float(intrinsics[0,0])*2,.001,.025)
    grout=np.clip((GROUT_WIDTH_M/2+footprint/2-seam_distance)/footprint,0,1)
    tile_index=np.floor(xy/TILE_SIZE_M)
    variation=2.*np.sin(tile_index[...,0]*12.31+tile_index[...,1]*7.17)
    vein=1.3*np.sin(xy[...,0]*48+np.sin(xy[...,1]*29))*np.sin(xy[...,1]*35)
    light=np.clip(1.-safe_t*.025,.76,1.)
    base=np.array([194.,202.,207.])+variation[...,None]+vein[...,None]
    floor_color=(base*(1-grout[...,None])+np.array([133.,140.,144.])*grout[...,None])*light[...,None]
    image[floor]=floor_color[floor];depth[floor]=floor_t[floor]
    for a,b in np.asarray(walls,dtype=float):
        edge=b-a;offset=a-origin[:2]
        denominator=rays[...,0]*edge[1]-rays[...,1]*edge[0]
        with np.errstate(divide='ignore',invalid='ignore'):
            t=(offset[0]*edge[1]-offset[1]*edge[0])/denominator
            along=(offset[0]*rays[...,1]-offset[1]*rays[...,0])/denominator
        z=origin[2]+t*rays[...,2]
        visible=(np.abs(denominator)>1e-8)&(t>0)&(t<depth)&(along>=0)&(along<=1)&(z>=0)&(z<=WALL_HEIGHT_M)
        # A tiny plaster pattern, directional wall shading and a darker skirting.
        u=np.where(visible,along*np.linalg.norm(edge),0.)
        zz=np.where(visible,z,0.)
        grain=1.7*np.sin(u*143+zz*37)*np.sin(zz*111-u*29)
        shade=.86+.10*abs(edge[0])/np.linalg.norm(edge)
        wall_color=(np.array([207.,215.,220.])+grain[...,None])*shade
        wall_color*=np.where(zz<.055,.78,1.)[...,None]
        image[visible]=wall_color[visible];depth[visible]=t[visible]
    return cv2.resize(np.clip(image,0,255).astype(np.uint8),(width,height),interpolation=cv2.INTER_LINEAR)
