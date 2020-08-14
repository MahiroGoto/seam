from typing import Dict, Union
import numpy as np
import math

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline
from compas_blender.geometry import BlenderGeometry, BlenderCurve

import seam.utils.utils as utils
from seam.Branch import skeleton_data, discrete_curve, boundary_control
from seam.Branch.skeleton_data import Skeleton
import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

########################################################################################################################


DATA_PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/" \
            "MAS Thesis 2020/3_Prototype/17_demonstrate_work_flow/data_02/"

if __name__ == "__main__":

    ## load skeleton data from GH ##
    skeleton_topology = utils.load_from_Json(DATA_PATH, "_Skeleton.json")
    all_vertices_data = utils.load_from_Json(DATA_PATH, "_all_vertices_data.json")
    start_and_node_vertices = utils.load_from_Json(DATA_PATH, "_start_and_node_vertices.json")

    ## get skeleton and calculate information of it such as vectors and planes ##
    skeleton = Skeleton(skeleton_topology, all_vertices_data, start_and_node_vertices)

    branch_keys = skeleton.branch_keys
    palnes_const = skeleton.vPlanes_const
    planes_data = skeleton.vPlanes_data

    branch_vertex_vertices = skeleton.branch_vertex_vertices

    utils.save_json(branch_keys, DATA_PATH, "branch_keys.json")
    utils.save_json(planes_data, DATA_PATH, "planes_data.json")











































