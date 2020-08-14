from typing import Dict, Union
import numpy as np
import math

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane
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

########################################################################################################################


DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data_02/'

if __name__ == "__main__":
    vertices = utils.load_from_Json(DATA_PATH, "pts_data_02.json")

    Skeleton = skeleton_data.create_data_from_pts(vertices)
    print(Skeleton)

    vTangents, vNormals, vBinormals = skeleton_data.get_vectors_on_vertices(Skeleton)

