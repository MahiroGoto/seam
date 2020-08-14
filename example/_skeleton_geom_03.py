from typing import Dict, Union
import numpy as np
import math

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline
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



"""
this file is an exploration for the construct the bezier curves from boundaries
"""

# DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data_02/'
DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/' \
            'MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data_02/'


if __name__ == "__main__":
    vertices = utils.load_from_Json(DATA_PATH, "pts_data_02.json")

    # create a data structure of BRANCH #
    Skeleton = skeleton_data.create_data_from_pts(vertices)

    # get three vectors on every vertices #
    vTangents, vNormals, vBinormals = skeleton_data.get_vectors_on_vertices(Skeleton)

    # create the planes on each vertex #
    vIndices, eIndices = skeleton_data.get_indices_of_vertices_and_edges(Skeleton)
    vertices = skeleton_data.get_vertices_from_Skeleton(Skeleton)
    vPlanes = skeleton_data.get_planes_on_each_vertex(Skeleton)

    ## sending the plane datas to Rhino ##
    ## x direction of the plane is vNormal ##
    planes_data = skeleton_data.get_planes_data_point_and_two_vectors(Skeleton)
    print("planes_data :", planes_data)
    utils.save_json(planes_data, DATA_PATH, "planes_data.json")

    Bound_pts_data_list = utils.load_from_Json(DATA_PATH, "Bound_pts_list.json")

    bPts_list = {}
    for vInd in vIndices:
        Bound_pts_data = Bound_pts_data_list[str(vInd)]
        bPt_list = []
        for boundPt_data in Bound_pts_data:
            bPt = Point(boundPt_data[0], boundPt_data[1], boundPt_data[2])
            bPt_list.append(bPt)
        bPts_list[vInd] = bPt_list

    print(vIndices)
    control_string = zip(vIndices[:-1], vIndices[1:])
    connecting_bezier_dict = {}
    for vInd, vInd_ in control_string:
        bPts = bPts_list[vInd]
        bPts_ = bPts_list[vInd_]
        vTangent_pair = [vTangents[vInd], vTangents[vInd_]]
        bezier_in_one_level = []
        for i in range(len(bPts)):
            bPts_pair = [bPts[i], bPts_[i]]
            bezier_pts = discrete_curve.bezier_curve_from_two_vectors(bPts_pair, vTangent_pair, resolutionNum=7, degree=0.3)
            bezier = utils.convert_compas_Points_list_to_Data(bezier_pts)
            bezier_in_one_level.append(bezier)
        connecting_bezier_dict[vInd] = bezier_in_one_level
    print(connecting_bezier_dict)
    utils.save_json(connecting_bezier_dict, DATA_PATH, "connecting_bezier_dict.json")














