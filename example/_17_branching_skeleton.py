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
            "MAS Thesis 2020/3_Prototype/17_demonstrate_work_flow/data/"

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


    ## load boundaries points to build the bezier curves ##
    bound_pts_dict = utils.load_from_Json(DATA_PATH, "_bound_pts_dict.json")

    ## create the connecting curve ##
    vTangent_dict = skeleton.vTangent_vectors
    bezier_dict = {}
    node_boundPts = {}

    for bKey in branch_keys:
        bezier_in_one_branch = []
        bound_pts = bound_pts_dict[str(bKey)]
        vTangents = vTangent_dict[bKey]

        ## last boundary is in the branching node ##
        last = bound_pts[-2]
        node_boundPts[bKey] = last

        pair_string = zip(vTangents[:-2], vTangents[1:-1], bound_pts[:-2], bound_pts[1:-1])
        ## in one level ##
        for i, pair in enumerate(pair_string):
            tVec00 = pair[0]
            tVec01 = pair[1]

            pts00 = pair[2]
            pts00 = utils.convert_data_pts_list_to_compas_pts(pts00)
            pts01 = pair[3]
            pts01 = utils.convert_data_pts_list_to_compas_pts(pts01)
            ## get pairs of two pts between boundaries ##
            pt_pairs, pair_lines = boundary_control.create_pair_point_list_between_boundaries(pts00, pts01)

            ## build bezier curve in between pair points ##
            bezier_in_one_level = []
            for pt_pair in pt_pairs:
                bezier = discrete_curve.bezier_curve_from_two_vectors(pt_pair, [tVec00, tVec01],
                                                                      resolutionNum=10, degree=0.25)
                bezier_data = utils.convert_compas_Points_list_to_Data(bezier)
                bezier_in_one_level.append(bezier_data)
            bezier_in_one_branch.append(bezier_in_one_level)

        bezier_dict[bKey] = bezier_in_one_branch

    # print("bezier_dict :", bezier_dict)

    bezier_branch00 = bezier_dict[branch_keys[0]]
    bezier_branch01 = bezier_dict[branch_keys[1]]
    bezier_branch02 = bezier_dict[branch_keys[2]]

    utils.save_json(bezier_branch00, DATA_PATH, "bezier_branch00.json")
    utils.save_json(bezier_branch01, DATA_PATH, "bezier_branch01.json")
    utils.save_json(bezier_branch02, DATA_PATH, "bezier_branch02.json")


    ###########################
    ## create branching node ##
    node_bounds = [node_boundPts[bKey] for bKey in branch_keys]
    print("node_bounds :", node_bounds)
    connected_Pts_list = boundary_control.get_point_pairs_at_branching_node(node_bounds)
    # print("connected_Pts_list :", connected_Pts_list)

    all_length = []
    for conne_pts in connected_Pts_list:
        conne_pt00 = conne_pts[0]
        conne_pt01 = conne_pts[1]
        for i, node_bounds_pts in enumerate(node_bounds):
            if conne_pt00 in node_bounds_pts:
                vectors = skeleton.vTangent_vectors[branch_keys[i]]
                vec00 = vectors[-1]
            if conne_pt01 in node_bounds_pts:
                vectors = skeleton.vTangent_vectors[branch_keys[i]]
                vec01 = vectors[-1]
        bezier = discrete_curve.bezier_curve_from_two_vectors(conne_pts, [vec00, vec01],
                                                              resolutionNum=7, degree=0.4, is_node=True)
        length = discrete_curve.measure_langth_of_bezier(bezier)
        all_length.append(length)
    average = sum(all_length) / len(all_length)
    print("average :", average)

    all_beziers = []
    for conne_pts in connected_Pts_list:
        conne_pt00 = conne_pts[0]
        conne_pt01 = conne_pts[1]
        for i, node_bounds_pts in enumerate(node_bounds):
            if conne_pt00 in node_bounds_pts:
                vectors = skeleton.vTangent_vectors[branch_keys[i]]
                vec00 = vectors[-1]
            if conne_pt01 in node_bounds_pts:
                vectors = skeleton.vTangent_vectors[branch_keys[i]]
                vec01 = vectors[-1]
        bezier = discrete_curve.bezier_curve_from_two_vectors(conne_pts, [vec00, vec01],
                                                              resolutionNum=7, degree=0.4, is_node=True)
        length = discrete_curve.measure_langth_of_bezier(bezier)
        addNum = (length - average) / 100
        # print("degree+addNum :", 0.4+addNum)
        bezier = discrete_curve.bezier_curve_from_two_vectors(conne_pts, [vec00, vec01],
                                                              resolutionNum=7, degree=0.4+addNum, is_node=True)
        all_beziers.append(bezier)

    ##########################################################################
    ## convert bezier list into data for visualization in RHINO and send it ##
    all_beziers_data_list = []
    for bezier in all_beziers:
        bezier_data = utils.convert_compas_Points_list_to_Data(bezier)
        all_beziers_data_list.append(bezier_data)
    utils.save_json(all_beziers_data_list, DATA_PATH, "all_beziers_data_list.json")
    ##########################################################################

    node_bounds_pts_list = []
    for node_bound in node_bounds:
        pts = []
        for pt_coord in node_bound:
            pt = Point(pt_coord[0], pt_coord[1], pt_coord[2])
            pts.append(pt)
        node_bounds_pts_list.append(pts)


    ## to get the boundaries ##
    boundary_list = []
    for i, node_pts in enumerate(node_bounds_pts_list):
        print(i)
        boundary_pts = []
        for pt in node_pts:
            ## seach the bezier that this point is belonging to ##
            for bezier in all_beziers:
                cl_pt, distance = utils.select_closest_pt_from_list(pt, bezier)
                if distance < 1:
                    rail_bezier = bezier
                    aim_point = discrete_curve.create_point_around_centre_of_polyline_with_pts_list(rail_bezier, pt, 12)
                    boundary_pts.append(aim_point)
                    break
        boundary_list.append(boundary_pts)


    updated_boundary_list = []
    for boundary_pts in boundary_list:
        updated_boundary = discrete_curve.compliment_polyline_pts_with_bezier(boundary_pts, degree=0.2)
        print("updated_boundary :", updated_boundary)
        updated_boundary_list.append(updated_boundary)

    final_updated_boundary_list = boundary_control.modify_boundaries_pts_position(updated_boundary_list)

    ############################################################################
    ## convert boundary_list into data for visualization in RHINO and send it ##
    final_updated_boundary_list_data = []
    for boundary in final_updated_boundary_list:
        boundary_data = utils.convert_compas_Points_list_to_Data(boundary)
        final_updated_boundary_list_data.append(boundary_data)
    utils.save_json(final_updated_boundary_list_data, DATA_PATH, "final_updated_boundary_list_data.json")














































