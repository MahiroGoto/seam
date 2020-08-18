import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane

from seam.utils import utils, primitive
from seam.Branch import discrete_curve, boundary_control
import igl

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################

def get_seams_pts_from_data(seams_pts_Data):
    seams_pts = []
    for seam_pts in seams_pts_Data:
        s_pts = utils.convert_data_pts_list_to_compas_pts(seam_pts, ROUND=False)
        seams_pts.append(s_pts)
    return seams_pts


def get_seam_vertex_indices(mesh, seam_pts):
    """
    useful for calculate geodesic distance between boundaries
    """
    v, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v]

    seam_ids = []
    for bPt in seam_pts:
        closestPt, minimun, index = primitive.get_closest_point_from_pts_list(bPt, vertices)
        if index not in seam_ids:
            seam_ids.append(index)
    return seam_ids


def get_seams_vertex_indices_first_and_second(mesh, seams_pts):
    seams_ids = []
    for seam_pts in seams_pts:
        seam_ids = get_seam_vertex_indices(mesh, seam_pts)
        seams_ids.append(seam_ids)
    return seams_ids















