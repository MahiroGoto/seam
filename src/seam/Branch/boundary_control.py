from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline

from seam.Branch import skeleton_data
import seam.utils.utils as utils

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

########################################################################################################################

def select_closest_pt_from_list(from_pt, pts_list, skip_pts):
    """
    data format is compas
    """
    global closestPt
    distances = []
    for pt in pts_list:
        if pt not in skip_pts:
            distance = from_pt.distance_to_point(pt)
            distances.append(distance)
    distances.sort()
    min_dis = round(distances[0], 3)
    for pt in pts_list:
        if pt not in skip_pts:
            distance = round(from_pt.distance_to_point(pt), 3)
            if distance == min_dis:
                closestPt = pt
                break
    return closestPt


def create_pair_point_list_between_boundaries(pts_list_00, pts_list_01):
    """
    these point list have compas point data
    """
    pts_00 = [pt for pt in pts_list_00]
    pts_01 = [pt for pt in pts_list_01]

    pair_lines = []
    pairs = []
    num = len(pts_00)
    pts_exis_00 = []
    pts_exis_01 = []
    for k in range(num):
        for pt_00 in pts_00:
            skip = []
            for i in range(len(pts_01)):
                closestPt_01 = select_closest_pt_from_list(pt_00, pts_01, skip)
                skip_pts = []
                closest_pt_00_check = select_closest_pt_from_list(closestPt_01, pts_00, skip_pts)
                if closest_pt_00_check == pt_00:
                    closestPt = closestPt_01
                    pts_exis_01.append(closestPt)
                    pts_exis_00.append(pt_00)
                    pair = [pt_00, closestPt]
                    pairs.append(pair)
                    pair_line = Polyline(pair)
                    pair_lines.append(pair_line)
                    break
        id_00 = pts_00.index(pts_exis_00[-1])
        id_01 = pts_01.index(pts_exis_01[-1])
        pts_00.pop(id_00)
        pts_01.pop(id_01)
    ## array lines ##
    pairs_arrayed = []
    pts_00 = [pt for pt in pts_list_00]
    for pt_00 in pts_00:
        for pair in pairs:
            if pt_00 in pair and pair not in pairs_arrayed:
                pairs_arrayed.append(pair)

    return pairs_arrayed, pair_lines



######################################################################
### branching node ###
######################################################################

def get_point_pairs_at_branching_node(node_pts_list_data):
    bound_pts_list = []
    for bound_pts_data in node_pts_list_data:
        bound_pts = utils.convert_data_pts_list_to_compas_pts(bound_pts_data)
        bound_pts_list.append(bound_pts)

    keys = []
    for i in range(len(bound_pts_list)):
        keys.append(i)
    keys_pair_patterns = []
    for key in keys:
        for key_ in keys:
            if key_ != key:
                key_pair = [key, key_]
                key_pair.sort()
                if key_pair not in keys_pair_patterns:
                    keys_pair_patterns.append(key_pair)
    logger.info("NUMBER OF BRANCH :"+str(len(keys_pair_patterns)))

    pts_pair_all = []
    for keys_pair in keys_pair_patterns:
        pairs, pair_lines = \
            create_pair_point_list_between_boundaries(
                bound_pts_list[keys_pair[0]], bound_pts_list[keys_pair[1]])
        pts_pair_all.extend(pairs)

    ## main code from here ##
    connected_Pts_list = []
    existing = []
    for pts in bound_pts_list:
        for pt in pts:
            if pt not in existing:
                pairs_selected = []
                for pair in pts_pair_all:
                    if pt in pair:
                        pairs_selected.append(pair)
                distances = [round(pair_selected[0].distance_to_point(pair_selected[1]), 3) for pair_selected in pairs_selected]
                distances.sort()
                min_dis = distances[0]
                for pair_selected in pairs_selected:
                    if round(pair_selected[0].distance_to_point(pair_selected[1]), 3) == min_dis:
                        if pair_selected[0] not in existing and pair_selected[1] not in existing:
                            connected_Pts_list.append(pair_selected)
                            existing.extend(pair_selected)
    return connected_Pts_list

def modify_boundaries_pts_position(boundary_list, iterationNum=30):
    for k in range(iterationNum):
        new_boundary_list = []
        move_count = 0
        for i, boundary in enumerate(boundary_list):
            print("move_count :", move_count)
            moved_boundary = []

            check_of_this_boundary = 0
            for pt_for_check in boundary:
                aim_pts = []
                aim_pts_distance = []
                for j, other_boundary in enumerate(boundary_list):
                    if j != i:
                        closestPt, distance = utils.select_closest_pt_from_list(pt_for_check, other_boundary)
                        aim_pts_distance.append(distance)
                        aim_pts.append(closestPt)
                aim_pts_distance.sort()
                if aim_pts_distance[0] < 4:
                    check_of_this_boundary += 0
                else:
                    check_of_this_boundary += 1

            if check_of_this_boundary != 0:
                for pt in boundary:
                    aim_pts = []
                    aim_pts_distance = []
                    for j, other_boundary in enumerate(boundary_list):
                        if j != i:
                            closestPt, distance = utils.select_closest_pt_from_list(pt, other_boundary)
                            aim_pts_distance.append(distance)
                            aim_pts.append(closestPt)
                    aim_pts_distance.sort()
                    if aim_pts_distance[0] < 4:
                        moving = False
                    else:
                        moving = True

                    if moving:
                        move_count += 1
                        print("moving!!")
                        evaluating = aim_pts_distance[-1] - aim_pts_distance[0]
                        print("aim_pts_distance :", len(aim_pts_distance))
                        if evaluating < 7:
                            print("yes!!")
                            aim_vec_00 = aim_pts[0] - pt
                            aim_vec_01 = aim_pts[1] - pt
                            aim_vec = (aim_vec_00.unitized() + aim_vec_01.unitized())/1
                            # aim_vector = aim_vector.unitized()
                        else:
                            for aim_pt in aim_pts:
                                aim_dis = round(aim_pt.distance_to_point(pt), 3)
                                if aim_dis == round(aim_pts_distance[0], 3):
                                    aim_vector = aim_pt - pt
                                    # aim_vector = aim_vector.unitized()
                                    break
                        pt_moved = pt + aim_vector * 0.03
                    else:
                        for aim_pt in aim_pts:
                            aim_dis = round(aim_pt.distance_to_point(pt), 3)
                            if aim_dis == round(aim_pts_distance[0], 3):
                                aim_vector = aim_pt - pt
                                # aim_vector = aim_vector.unitized()
                        pt_moved = pt + aim_vector * 0.01
                    moved_boundary.append(pt_moved)
                new_boundary_list.append(moved_boundary)
            else:
                new_boundary_list.append(boundary)
        boundary_list = new_boundary_list
        if move_count == 0:
            break
    logger.info("modify iteration :"+str(k))
    return boundary_list
































