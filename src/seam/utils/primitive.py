import os
import json
import sys
import math

from compas.geometry import Point, Vector, Circle, Rotation, Plane
from compas.geometry import Transformation

import logging
## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
#######################################

def polygon(plane_const, radius, vNum, closed=False):
    """
    plane_const is like [centre_Pt, [u_Venctor, v_Vector]]
    """
    centrePt = plane_const[0]
    vectors = plane_const[1]
    u = vectors[0]
    v = vectors[1]
    normal = u.cross(v)

    normal.unitized()
    print("normal", normal)
    angle_step = 2*(math.pi) / vNum
    firstPt = centrePt + u * radius
    polypts = []
    if closed:
        num = vNum + 1
    else:
        num = vNum
    for i in range(num):
        rotation = Rotation.from_axis_and_angle(normal, angle_step*(i))
        temp = firstPt
        point = temp.transformed(rotation)
        polypts.append(point)
    return polypts

def get_closest_point_from_pts_list(point, pts_list):
    distances = [point.distance_to_point(target) for target in pts_list]
    distances.sort()
    minimum = round(distances[0], 5)
    for i, targetPt in enumerate(pts_list):
        distance = round(point.distance_to_point(targetPt), 5)
        if distance == minimum:
            closestPt = targetPt
            index = i
            break
    return closestPt, minimum, index











