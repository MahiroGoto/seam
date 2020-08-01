from typing import Dict, Union

import numpy as np

from compas.geometry import Vector, Point, Rotation

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


def create_branch_data_from_pts(pts_data_dict):

    vertices = pts_data_dict
    key_list = list(vertices.keys())

    Branch = {}
    for i in range(len(key_list)-1):
        keys_pair = [key_list[i], key_list[i+1]]
        v_start = vertices[keys_pair[0]]
        v_end = vertices[keys_pair[1]]
        # create vertex point with compas point
        v_start = Point(x=v_start[0], y=v_start[1], z=v_start[2])
        v_end = Point(x=v_end[0], y=v_end[1], z=v_end[2])
        # EDGE TANGENT VECTOR is not an unit vector, has a length
        E_Tangent = v_end - v_start
        Branch[i] = { "Edge_Tangent" : E_Tangent }

    logger.info("Branch Num "+str(len(Branch)))
    logger.info("Vertices Num "+str(len(vertices)))

    return Branch

