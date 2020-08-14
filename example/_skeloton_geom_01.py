from typing import Dict, Union
import numpy as np
import math

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane
from compas_blender.geometry import BlenderGeometry, BlenderCurve

import seam.utils.utils as utils
from seam.Branch import branch_data
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

    # create a data structure of BRANCH #
    Branch = branch_data.create_data_from_pts(vertices)

    # get three vectors on every vertices #
    vTangents, vNormals, vBinormals = branch_data.get_vectors_on_vertices(Branch)

    # check the direction #
    branch_indices = branch_data.get_separated_branch_indices_list(Branch)
    print('branch_indices :', branch_indices)

    #######################################
    # get vertex indices and edge indices #
    vIndices = vTangents.keys()
    eIndices = list(Branch.keys())
    for i, index in enumerate(eIndices):
        if type(index) != int:
            eIndices.pop(i)
    #######################################

    # # set vertices from branch data #
    # branch_origin = Branch["origin"]
    # vertices = []
    # vertices.append(branch_origin)
    # vertex = branch_origin
    # for eIndex in eIndices:
    #     branch = Branch[eIndex]
    #     Tangent_vec = branch["branch_Tangent"]
    #     vertex = vertex + Tangent_vec
    #     vertices.append(vertex)


    vertices = branch_data.get_vertices_from_Branch(Branch)




    # curve = bezier.curve.Curve.from_nodes(pts_bezier)

    # # set vertices from branch data #
    # branch_origin = Branch["origin"]
    # vertices = []
    # vertices.append(branch_origin)
    # vertex = branch_origin
    # for eIndex in eIndices:
    #     branch = Branch[eIndex]
    #     Tangent_vec = branch["Edge_Tangent"]
    #     vertex = vertex + Tangent_vec
    #     vertices.append(vertex)
    #
    # # create a seam curve #
    # seams = []
    # for vIndex in vIndices:
    #     vTangent = vTangents[vIndex]
    #     vNormal = vNormals[vIndex]
    #     vBinormal = vBinormals[vIndex]
    #
    #     vertex = vertices[vIndex]
    #     cPlane = Plane(vertex, vTangent)
    #
    #     circle = Circle(cPlane, radius=40)
    #









