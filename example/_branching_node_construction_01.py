from typing import Dict, Union
import numpy as np
import math

import splipy
from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline
from compas_blender.geometry import BlenderGeometry, BlenderCurve

import seam.utils.utils as utils
from seam.Branch import skeleton_data, discrete_curve, boundary_control
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


DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/' \
            '3_Prototype/000_Exploration/07_branch_data_structure/data_node_construct/'

if __name__ == "__main__":
    node_pts_list_data = utils.load_from_Json(DATA_PATH, "node_pts_list_data.json")

    connected_Pts_list = boundary_control.get_point_pairs_at_branching_node(node_pts_list_data)

    ## the list of node pts ##
    node_pts_list = []
    for node_pts_data in node_pts_list_data:
        node_pts = utils.convert_data_pts_list_to_compas_pts(node_pts_data, ROUND=True)
        node_pts_list.append(node_pts)

    ## the list of the direction (tangent) vector on the node ##
    node_vectors_data = utils.load_from_Json(DATA_PATH, "node_vectors_data.json")
    node_vectors = utils.convert_data_vectors_list_to_compas_vectors(node_vectors_data, ROUND=True)

    ## get bezeir points list from two connected points ##
    bezier_list = []
    for connected_pts in connected_Pts_list:
        ## get the vector of the point ##
        vector_pair = []
        for pt in connected_pts:
            for i, node_pts in enumerate(node_pts_list):
                closest, min_dis = utils.select_closest_pt_from_list(pt, node_pts)
                if min_dis < 1:
                    node_vec = node_vectors[i]
                    vector_pair.append(node_vec)
        dis = connected_pts[0].distance_to_point(connected_pts[1])
        add_value = dis/500
        bezier = discrete_curve.bezier_curve_from_two_vectors(connected_pts, vector_pair, degree=0.3+add_value, resolutionNum=5, is_node=True)
        bezier_list.append(bezier)

    ##########################################################################
    ## convert bezier list into data for visualization in RHINO and send it ##
    bezier_data_list = []
    for bezier in bezier_list:
        bezier_data = utils.convert_compas_Points_list_to_Data(bezier)
        bezier_data_list.append(bezier_data)
    utils.save_json(bezier_data_list, DATA_PATH, "bezier_data_list.json")
    ##########################################################################

    ## to get the boundaries ##
    boundary_list = []
    for i, node_pts in enumerate(node_pts_list):
        print(i)
        boundary_pts = []
        for pt in node_pts:
            ## seach the bezier that this point is belonging to ##
            for bezier in bezier_list:
                cl_pt, distance = utils.select_closest_pt_from_list(pt, bezier)
                if distance < 1:
                    rail_bezier = bezier
                    aim_point = discrete_curve.create_point_around_centre_of_polyline_with_pts_list(rail_bezier, pt, 3)
                    boundary_pts.append(aim_point)
                    break
        boundary_list.append(boundary_pts)

    # ############################################################################
    # ## convert boundary_list into data for visualization in RHINO and send it ##
    # boundary_list_data = []
    # for boundary in boundary_list:
    #     boundary_data = utils.convert_compas_Points_list_to_Data(boundary)
    #     boundary_list_data.append(boundary_data)
    # utils.save_json(boundary_list_data, DATA_PATH, "boundary_list_data.json")
    # ##########################################################################

    ## modifying the boundaries by adding points of bezier ##
    ## complimenting the curve by adding points ##
    print("boundary_list Num;", len(boundary_list))

    updated_boundary_list = []
    for boundary_pts in boundary_list:
        updated_boundary = discrete_curve.compliment_polyline_pts_with_bezier(boundary_pts)
        print("updated_boundary :", updated_boundary)
        updated_boundary_list.append(updated_boundary)
    ## move boundary points to make them closer ##
    final_updated_boundary_list = boundary_control.modify_boundaries_pts_position(updated_boundary_list)


    ############################################################################
    ## convert boundary_list into data for visualization in RHINO and send it ##
    final_updated_boundary_list_data = []
    for boundary in final_updated_boundary_list:
        boundary_data = utils.convert_compas_Points_list_to_Data(boundary)
        final_updated_boundary_list_data.append(boundary_data)
    utils.save_json(final_updated_boundary_list_data, DATA_PATH, "final_updated_boundary_list_data.json")
    ##########################################################################



























