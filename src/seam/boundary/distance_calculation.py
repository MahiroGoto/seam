import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane

from seam.utils import utils, primitive, parameters
from seam.Branch import discrete_curve, boundary_control
import igl

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################



def get_geodesic_distances_to_every_vertex(mesh, vertices_start):
    """
    compute distances from sevelral vertices to all vertices of the mesh
    vertices are described with keys
    """
    v, f = mesh.to_vertices_and_faces()
    v = np.array(v)
    f = np.array(f)
    vertices_target = np.arange(len(v))  # all vertices are targets
    vstart = np.array(vertices_start)
    distances = igl.exact_geodesic(v, f, vstart, vertices_target)
    return distances


def get_geodesic_from_start_to_target_v_indices(mesh, start_edge_pts_keys_list, target_edge_pts_keys_list):
    """
    compute distances from one edges to another edge and get longest way and shortest way
    """
    v, f = mesh.to_vertices_and_faces()
    v = np.array(v)
    f = np.array(f)
    vertices_start = np.array(start_edge_pts_keys_list)
    vertices_target = np.array(target_edge_pts_keys_list)
    distances = igl.exact_geodesic(v, f, vertices_start, vertices_target)
    return distances


def get_shortest_way_between_two_seams(mesh, seam_ids_list):
    seam_00 = seam_ids_list[0]
    seam_01 = seam_ids_list[1]

    distances = get_geodesic_from_start_to_target_v_indices(mesh, seam_01, seam_00)
    distances = list(distances)
    distances.sort()
    minimum = distances[0]
    for b_id in seam_00:
        distance = get_geodesic_from_start_to_target_v_indices(mesh, seam_01, [b_id])
        if distance == minimum:
            shortest_v_00 = b_id
            break

    distances = get_geodesic_from_start_to_target_v_indices(mesh, seam_00, seam_01)
    distances = list(distances)
    distances.sort()
    minimum = distances[0]
    for b_id in seam_01:
        distance = get_geodesic_from_start_to_target_v_indices(mesh, seam_00, [b_id])
        if distance == minimum:
            shortest_v_01 = b_id
            break
    shortest_distance = get_geodesic_from_start_to_target_v_indices(mesh, [shortest_v_00], [shortest_v_01])
    return shortest_v_00, shortest_v_01, shortest_distance


def get_longest_way_between_two_seams(mesh, seam_ids_list):
    seam_00 = seam_ids_list[0]
    seam_01 = seam_ids_list[1]

    distances = get_geodesic_from_start_to_target_v_indices(mesh, seam_01, seam_00)
    distances = list(distances)
    distances.sort()
    maximum = distances[-1]
    for b_id in seam_00:
        distance = get_geodesic_from_start_to_target_v_indices(mesh, seam_01, [b_id])
        if distance == maximum:
            longest_v_00 = b_id
            break
    distances = get_geodesic_from_start_to_target_v_indices(mesh, seam_00, seam_01)
    distances = list(distances)
    distances.sort()
    maximum = distances[-1]
    for b_id in seam_01:
        distance = get_geodesic_from_start_to_target_v_indices(mesh, seam_00, [b_id])
        if distance == maximum:
            longest_v_01 = b_id
            break
    longest_distance = get_geodesic_from_start_to_target_v_indices(mesh, [longest_v_00], [longest_v_01])
    return longest_v_00, longest_v_01, longest_distance


def get_gap_ratio(mesh, seam_ids_list, gr_max=0.5):
    shortest_v_00, shortest_v_01, s_dis = get_shortest_way_between_two_seams(mesh, seam_ids_list)
    longest_v_00, longest_v_01, l_dis = get_longest_way_between_two_seams(mesh, seam_ids_list)
    GR = (l_dis - s_dis) / l_dis
    if GR > gr_max:
        trans_bound = True
    else:
        trans_bound = False
    return trans_bound, GR


########################
## generate layers ##
########################

def get_layer_number(mesh, seam_ids_list):
    # trans_bound, GR = get_gap_ratio(mesh, seam_ids_list)
    # if not trans_bound:
    shortest_v_00, shortest_v_01, shortest_distance = get_shortest_way_between_two_seams(mesh, seam_ids_list)
    longest_v_00, longest_v_01, longest_distance = get_longest_way_between_two_seams(mesh, seam_ids_list)

    maximum = parameters.get_param("max_layer_height")
    minimum = parameters.get_param("min_layer_height")

    longest_num = longest_distance / maximum
    shortest_num = shortest_distance / minimum

    min_num = longest_num
    max_num = shortest_num
    if min_num <= max_num:
        layer_num = int(round((min_num + max_num) / 2, 0))
    else:
        layer_num = None

    return layer_num







































