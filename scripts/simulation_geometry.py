"""Sensor/kinematic environment only. No mission states, routes or completion logic."""
import math

import cv2
import numpy as np
from ee3033_project.calibration import mounting_rotation


WALLS = np.array([[[-1.2, -1.0], [1.4, -1.0]], [[1.4, -1.0], [1.4, 1.3]],
                  [[1.4, 1.3], [-1.2, 1.3]], [[-1.2, 1.3], [-1.2, -1.0]],
                  [[.55, .15], [.55, 1.3]]], dtype=float)
TARGET = np.array([.95, .85, 0.])
K = np.array([[450., 0., 320.], [0., 450., 240.], [0., 0., 1.]])
PITCH = math.radians(10)
CAMERA_OFFSET = np.array([.05, 0., .22])


def cross(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def raycast(origin, angles, walls=WALLS, limit=8.):
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=-1)
    edge = walls[:, 1]-walls[:, 0]
    offset = walls[:, 0]-np.asarray(origin)
    denominator = cross(directions[:, None, :], edge[None, :, :])
    with np.errstate(divide='ignore', invalid='ignore'):
        distance = cross(offset, edge)[None, :]/denominator
        position = cross(offset[None, :, :], directions[:, None, :])/denominator
    valid = (abs(denominator)>1e-9) & (distance>=0) & (position>=0) & (position<=1)
    return np.minimum(limit, np.min(np.where(valid, distance, np.inf), axis=1))


def clearance(xy):
    edge = WALLS[:, 1]-WALLS[:, 0]
    along = np.clip(np.sum((np.asarray(xy)-WALLS[:, 0])*edge, axis=1)/np.sum(edge*edge, axis=1), 0, 1)
    return float(np.min(np.linalg.norm(WALLS[:, 0]+along[:, None]*edge-xy, axis=1)))


def camera_pose(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    body = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    return np.array([pose[0], pose[1], 0.])+body@CAMERA_OFFSET, body@mounting_rotation(PITCH, 0)


def project(points, translation, rotation):
    optical = (np.asarray(points)-translation)@rotation
    if np.any(optical[:, 2] <= .03):
        return None
    pixels = optical@K.T
    return (pixels[:, :2]/pixels[:, 2:]).astype(np.float32)


def render_camera(pose, texture, present):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:] = (159, 170, 177)
    image[160:] = (140, 145, 146)
    translation, rotation = camera_pose(pose)
    # Simple vertical walls and floor, with optical geometry shared with CameraInfo/TF.
    for a, b in WALLS:
        quad = project([[*a, .75], [*b, .75], [*b, 0], [*a, 0]], translation, rotation)
        if quad is not None:
            cv2.fillConvexPoly(image, np.clip(quad, -3000, 3000).astype(np.int32), (190, 197, 200))
            cv2.polylines(image, [np.clip(quad, -3000, 3000).astype(np.int32)], True, (100, 115, 125), 1)
    delta = TARGET[:2]-translation[:2]
    distance = np.linalg.norm(delta)
    visible = raycast(translation[:2], [math.atan2(delta[1], delta[0])])[0] >= distance-.01
    if present and visible:
        right = np.array([delta[1], -delta[0], 0.])/max(distance, .001)
        left, right = TARGET-right*.09, TARGET+right*.09
        corners = np.array([left+[0, 0, .18], right+[0, 0, .18], right, left])
        quad = project(corners, translation, rotation)
        if quad is not None and np.max(quad[:, 0])>=0 and np.min(quad[:, 0])<640:
            h, w = texture.shape[:2]
            transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]]), quad)
            layer = cv2.warpPerspective(texture, transform, (640, 480))
            mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), transform, (640, 480))
            image[mask>0] = layer[mask>0]
    cv2.putText(image, 'SIMULATED SENSOR / REAL YOLO', (12, 465), cv2.FONT_HERSHEY_SIMPLEX, .45, (50, 60, 70), 1)
    return image
