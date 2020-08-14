from typing import Dict, Union
import math
import numpy as np

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline, intersection_line_line
from compas_blender.geometry import BlenderGeometry, BlenderCurve

import seam.utils.utils as utils
from seam.Branch import skeleton_data, discrete_curve
import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################

start_end_pts = [Point(20,0,0), Point(0,20,0)]
distance = Point(20,0,0).distance_to_point(Point(0,20,0))
resolution = int(distance / 3)
print("resolution :", resolution)

vec1 = Vector(0,2,0)
vec1 = vec1.unitized()
vec2 = Vector(-1,-1,0)
vec2 = vec2.unitized()
start_end_vectors = [vec1, vec2]

bezier_pts = discrete_curve.bezier_curve_from_two_vectors(
    start_end_pts, start_end_vectors, resolutionNum=resolution, degree=0.6)
print("bezier_pts Num :", len(bezier_pts))

pts = utils.convert_compas_Points_list_to_Data(bezier_pts)
print(pts)

DATA_PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU\MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data/"
utils.save_json(pts, DATA_PATH, "bezier_pts.json")





