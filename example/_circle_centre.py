from typing import Dict, Union
import math
import numpy as np

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline, intersection_line_line


import seam.utils.utils as utils
import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################



def get_intersection_pts_two_polyline(poly_0, poly_1, touch=True):
    """
    search intersection points of two polylines with using compas Poly
    This intersection point is the position where these two polylines are touching each other.
    If "touch = False", can get intersection points that is not where they are touching.
    """
    pts = []
    for i in range(len(poly_0) - 1):
        line1 = poly_0[i:i + 2]
        for j in range(len(poly_1) - 1):
            line2 = poly_1[j:j + 2]
            interPts = intersection_line_line(line1, line2)
            ## coordinate should be rounded to check if these two points are the same point or not ##
            interPts_coord = []
            for interPt in interPts:
                interPt_coord = [round(interPt[0],2), round(interPt[1],2), round(interPt[2],2)]
                interPts_coord.append(interPt_coord)
            # print(interPts_coord)

            if touch:
                if interPts_coord[0] == interPts_coord[1] and interPts_coord[0] != None:
                    coord = interPts[0]
                    interPt = Point(coord[0], coord[1], coord[2])
                    # print("interPt :", interPt)
                    pts.append(interPt)
            else:
                count = 0
                for interPt in interPts:
                    if interPt.on_polyline(poly_0) or interPt.on_polyline(poly_1):
                        count += 1
                if count == len(interPts):
                    pts.extend(interPts)
    # print(len(pts))
    if len(pts) == 1:
        return pts[0]
    elif len(pts) == 0:
        return None
    else:
        return pts


def calculate_circle_centre_and_radius_from_three_pts(three_pts_list):
    """
    points is compas Point
    """
    A = three_pts_list[0]
    B = three_pts_list[1]
    C = three_pts_list[2]
    vBA = B - A
    vBC = C - B
    normVec = vBA.cross(vBC)
    midBA = (B + A) / 2
    midBC = (B + C) / 2
    vec_BA = vBA.cross(normVec)
    vec_BC = vBC.cross(normVec)
    poly_0 = Polyline([midBA + vec_BA*10, midBA - vec_BA*10])
    poly_1 = Polyline([midBC - vec_BC*10, midBC + vec_BC*10])
    centrePt = get_intersection_pts_two_polyline(poly_0, poly_1, touch=True)

    radius = round(centrePt.distance_to_point(B), 3)

    return centrePt, radius


three_pts_list = [Point(0,0,0),
                  Point(50,50,0),
                  Point(100,0,0)]

centrePt, radius = calculate_circle_centre_and_radius_from_three_pts(three_pts_list)
print(centrePt, radius)

normal = three_pts_list[1] - centrePt
normal.unitized()
print(normal)



#     A = three_pts_list[0]
#     B = three_pts_list[1]
#     C = three_pts_list[2]
#     AB = B - A
#     BC = C - B
#     CA = C - A
#     a = AB.length
#     b = BC.length
#     c = CA.length
#     radius = (a * b * c) / np.sqrt(2.0 * a ** 2 * b ** 2 +
#                               2.0 * b ** 2 * c ** 2 +
#                               2.0 * c ** 2 * a ** 2 -
#                               a ** 4 - b ** 4 - c ** 4)
#
#     b1 = a ** 2 * (b ** 2 + c ** 2 - a ** 2)
#     b2 = b ** 2 * (c ** 2 + a ** 2 - b ** 2)
#     b3 = c ** 2 * (a ** 2 + b ** 2 - c ** 2)
#
#     Px = (b1 * A.x) + (b2 * B.x) + (b3 * C.x)
#     Py = (b1 * A.y) + (b2 * B.y) + (b3 * C.y)
#     Pz = (b1 * A.z) + (b2 * B.z) + (b3 * C.z)
#
#     centrePt = Point(Px, Py, Pz)
#     return centrePt, radius
#
#
# three_pts_list = [Point(0,0,0),
#                   Point(50,50,0),
#                   Point(100,0,0)]
# centrePt, radius = calculate_circle_centre_and_radius_from_three_pts(three_pts_list)
#
# print(centrePt, radius)























