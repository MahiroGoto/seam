from typing import Dict, Union
import math
import numpy as np

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane
from compas_blender.geometry import BlenderGeometry, BlenderCurve

import seam.utils.utils as utils
from seam.Branch import branch_data, discrete_curve
import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################


pt0 = [0,0,0]
pt1 = [0,50,0]
pt2 = [40,55,0]
pt3 = [45,5,0]

control_points = [pt0, pt1, pt2, pt3]
number_of_curve_points = 5
curve_vertices = discrete_curve.bezier_curve(control_points, number_of_curve_points)