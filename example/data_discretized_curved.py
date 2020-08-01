from typing import Dict, Union
import numpy as np
from compas.geometry import Vector, Point, Rotation
import seam.utils.utils as utils
import seam.

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################


DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data/'



if __name__ == "__main__":
    vertices = utils.load_from_Json(DATA_PATH, "data_pts.json")
    key_list = list(vertices.keys())

    # create a data structure of BRANCH #
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

    # builds Binormals from Edge_Tangents on each vertex #
    Binormals = {}
    indices = list(Branch.keys())
    print(indices)
    for j in range(len(indices)-1):
        print(j)
        branch = Branch[j]
        branch_ = Branch[j+1]
        tangent_vec = branch["Edge_Tangent"]
        tangent_vec_ = branch_["Edge_Tangent"]

        binormal_vec = tangent_vec.cross(tangent_vec_)
        binormal_vec.unitized()

        if j == 0:
            Binormals[j] = binormal_vec
        Binormals[j+1] = binormal_vec
        if j == len(indices)-2:
            Binormals[j+2] = binormal_vec

    logger.info("Binormals Num :"+str(len(Binormals)))

    # builds Normal can be calculated by tangent - previous tangent ##
    Normals = {}
    indices = list(Branch.keys())
    for m in range(len(indices)-1):
        index = indices[m]
        index_ = indices[m+1]
        edge_current = Branch[index]
        edge_next = Branch[index_]
        Edge_Tangent_current = edge_current["Edge_Tangent"]
        Edge_Tangent_next = edge_next["Edge_Tangent"]

        normal_vec = Edge_Tangent_current - Edge_Tangent_next
        normal_vec = normal_vec.unitized()

        if m == 0:
            start_edge = Branch[m]
            start_Edge_Tangent = start_edge["Edge_Tangent"]
            start_binormal = Binormals[m]
            normal_vec_start = start_Edge_Tangent.cross(start_binormal)
            normal_vec_start = normal_vec_start.unitized()

            Normals[m] = normal_vec_start

        Normals[m+1] = normal_vec

        if m == len(indices)-2:
            end_branch = Branch[m+1]
            end_Edge_Tangent = end_branch["Edge_Tangent"]
            end_binormal = Binormals[m+1]
            normal_vec_end = end_Edge_Tangent.cross(end_binormal)
            normal_vec_end = normal_vec_end.unitized()

            Normals[m+2] = normal_vec_end

    logger.info("Normals Num :"+str(len(Normals)))

    curvature = {}
    ## Tangent on vertex is cross product with Normal and Binormal ##
    for h in range(len(Normals)):
        normal = Normals[h]
        binorm = Binormals[h]

        verTangent =




