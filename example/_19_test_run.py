import math
import os

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh

import igl

from seam.utils import utils, primitive
from seam.boundary import seam_crv, boundary_crv, distance_calculation
from seam.Branch import discrete_curve, boundary_control

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################

DATA_PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/19_overlapping_connection/data_test/"
OBJ_INPUT_NAME = "test_mesh.obj"
OBJ_OUTPUT_NAME = ["test_mesh_splitted_00.obj.", "test_mesh_splitted_01.obj."]

if __name__ == "__main__":

    mesh = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_INPUT_NAME))
    print("Vertices : %d , Faces : %d " % (len(list(mesh.vertices())), len(list(mesh.faces()))))
    v_atrs, faces = mesh.to_vertices_and_faces()

    ## get compas point on seams and get seam vertices index from them ##
    seams_pts_Data = utils.load_from_Json(DATA_PATH, "seam_pts_data.json")
    seams_pts = []
    for seam_pts in seams_pts_Data:
        s_pts = utils.convert_data_pts_list_to_compas_pts(seam_pts, ROUND=False)
        seams_pts.append(s_pts)

    seam_ids_list = []
    for seam_pts in seams_pts:
        ids = seam_crv.get_seam_vertex_indices(mesh, seam_pts)
        seam_ids_list.append(ids)

    ## get differences from each seams and get curve points ##
    differences = boundary_crv.get_distance_differences_between_two_seams(mesh, seam_ids_list, time=0.5, ABS=False)
    print("differences :", differences)
    crv_pts = boundary_crv.get_curve_pts_from_distance_differences_on_Mesh(mesh, differences)

    ## split mesh ##
    mesh00, mesh01 = boundary_crv.split_mesh_with_single_differences_crv(mesh, differences)

    mesh00.to_obj(filepath=os.path.join(DATA_PATH, OBJ_OUTPUT_NAME[0]))
    mesh01.to_obj(filepath=os.path.join(DATA_PATH, OBJ_OUTPUT_NAME[1]))












































