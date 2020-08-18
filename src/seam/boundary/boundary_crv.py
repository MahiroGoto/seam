import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh

from seam.utils import utils, primitive
from seam.Branch import discrete_curve, boundary_control
from seam.boundary import distance_calculation
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



###############################
## get the distance differences
###############################
def get_distances_from_two_seams(mesh, seam_ids_list):
    boundary_ids_00 = seam_ids_list[0]
    boundary_ids_01 = seam_ids_list[1]
    distance_00 = list(distance_calculation.get_geodesic_distances_to_every_vertex(mesh, boundary_ids_00))
    distance_01 = list(distance_calculation.get_geodesic_distances_to_every_vertex(mesh, boundary_ids_01))
    return distance_00, distance_01

def get_distance_differences_between_two_seams(mesh, seam_ids_list, time=0.5, ABS=False):
    distance_00, distance_01 = get_distances_from_two_seams(mesh, seam_ids_list)
    if ABS:
        differences = [abs(d2 * time - d1 * (1 - time)) for d1, d2 in zip(distance_00, distance_01)]
    else:
        differences = [(d2 * time - d1 * (1 - time)) for d1, d2 in zip(distance_00, distance_01)]
    return differences


def get_distance_differences_with_cos_from_first(mesh, seam_ids_list, time=0.5, ABS=False):
    seam_distances_00, seam_distances_01 = get_distances_from_two_seams(mesh, seam_ids_list)
    seam_differences_00 = []
    shortest_v_00, shortest_v_01, shortest_distance = distance_calculation.get_shortest_way_between_two_seams(mesh, seam_ids_list)
    longest_v_00, longest_v_01, longest_distance = distance_calculation.get_longest_way_between_two_seams(mesh, seam_ids_list)

    for i in range(len(seam_distances_00)):
        d0 = seam_distances_00[i]
        d1 = seam_distances_01[i]
        way_length = d0 + d1
        way_gap = abs(way_length - (shortest_distance+longest_distance)/2)
        value = (way_gap / abs(longest_distance - (shortest_distance+longest_distance)/2)) * math.pi
        add_time = 0.17 * math.cos(value)
        if add_time > 0:
            add_time = add_time * 0

        if ABS:
            difference = abs(d0 * (time+add_time) - d1 * (1 - (time+add_time)))
        else:
            difference = (d0 * (time+add_time) - d1 * (1 - (time+add_time)))
        seam_differences_00.append(difference)
    return seam_differences_00


def get_distance_differences_with_cos_from_second(mesh, seam_ids_list, time=0.5, ABS=False):
    seam_distances_00, seam_distances_01 = get_distances_from_two_seams(mesh, seam_ids_list)
    seam_differences_01 = []
    shortest_v_00, shortest_v_01, shortest_distance = distance_calculation.get_shortest_way_between_two_seams(mesh, seam_ids_list)
    longest_v_00, longest_v_01, longest_distance = distance_calculation.get_longest_way_between_two_seams(mesh, seam_ids_list)

    for i in range(len(seam_distances_00)):
        d0 = seam_distances_00[i]
        d1 = seam_distances_01[i]
        way_length = d0 + d1
        way_gap = abs(way_length - (shortest_distance+longest_distance)/2)
        value = (way_gap / abs(longest_distance - (shortest_distance+longest_distance)/2)) * math.pi
        add_time = 0.17 * math.cos(value)
        if add_time < 0:
            add_time = add_time * 0
        if ABS:
            difference = abs(d0 * (time+add_time) - d1 * (1 - (time+add_time)))
        else:
            difference = (d0 * (time+add_time) - d1 * (1 - (time+add_time)))
        seam_differences_01.append(difference)
    return seam_differences_01


####################################################################################
## marching triangle for building the boundary curve and mesh splitting
####################################################################################

def get_curve_pts_from_distance_differences_on_Mesh(mesh, differences):
    v_atris, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]
    ## merching triangles to get the boundary points on the mesh edges ##
    crv_pts_list = []
    for face in faces:
        # v_distances = [differences[v_key] for v_key in face]
        nega = []
        posi = []
        # for i, v_d in enumerate(v_distances):
        for i, v_key in enumerate(face):
            seam_distance = differences[v_key]
            if seam_distance < 0:
                nega.append(v_key)
            elif seam_distance >= 0:
                posi.append(v_key)
        if len(nega) != 0 and len(posi) != 0:
            crvPts = []
            if len(nega) == 1:
                fromPt = nega[0]
                ## from pt is "vertices[fromPt]"
                from_distance = differences[fromPt]
                for toPt in posi:
                    to_distance = differences[toPt]
                    ## toPt is "vertices[toPt]"
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt] * (abs(to_distance)/dim) + vertices[toPt] * (abs(from_distance)/dim)
                    crvPts.append(point)
            elif len(posi) == 1:
                fromPt = posi[0]
                ## from pt is "vertices[fromPt]"
                from_distance = differences[fromPt]
                for toPt in nega:
                    to_distance = differences[toPt]
                    ## toPt is "vertices[toPt]"
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt] * (abs(to_distance)/dim) + vertices[toPt] * (abs(from_distance)/dim)
                    crvPts.append(point)
            crv_pts_list.append(crvPts)

    ## create the boundary curve from merching triangles ##
    print("length of crv_pts :", len(crv_pts_list))
    crv_pts = crv_pts_list[0]
    current = crv_pts[0]
    next = crv_pts[1]
    curve_points = []
    curve_points.append(next)
    exist = [0]
    for i in range(len(crv_pts_list)):
        for j, pts in enumerate(crv_pts_list):
            if j not in exist:
                if next in pts:
                    exist.append(j)
                    crv_pts = pts
                    if crv_pts[0] == next:
                        next = crv_pts[1]
                    else:
                        next = crv_pts[0]
                    curve_points.append(next)
    # print("number of the curve_points :", len(curve_points))
    return curve_points


#########################################################
## mesh processing tools
#########################################################

def check_exist_vertex(mesh, pt):
    v_atrs, faces = mesh.to_vertices_and_faces()
    # print("v_atrs :", v_atrs)
    vertices = [Point(x=v_atr[0], y=v_atr[1], z=v_atr[2]) for v_atr in v_atrs]
    if pt in vertices:
        index = vertices.index(pt)
        is_exist = True
    else:
        index = None
        is_exist = False
    return is_exist, index

def split_mesh_with_single_differences_crv(mesh, differences):
    v_atrs, faces = mesh.to_vertices_and_faces()
    ## split mesh ##
    mesh00 = Mesh()
    mesh01 = Mesh()
    faces_00 = []
    faces_01 = []
    vertices = [Point(x=v_atr[0], y=v_atr[1], z=v_atr[2]) for v_atr in v_atrs]
    for face in faces:
        # print("face :", face)
        nega = []
        posi = []
        for v_ind in face:
            diffe = differences[v_ind]
            if diffe < 0:
                nega.append(v_ind)
            elif diffe >= 0:
                posi.append(v_ind)

        if len(nega) != 0 and len(posi) != 0:
            crvPts = []
            if len(nega) == 1:
                fromPt = nega[0]
                ## from pt is "vertices[fromPt]"
                from_distance = differences[fromPt]
                for toPt in posi:
                    to_distance = differences[toPt]
                    ## toPt is "vertices[toPt]"
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt] * (abs(to_distance)/dim) + vertices[toPt] * (abs(from_distance)/dim)
                    crvPts.append(point)

                add_pts_00 = [vertices[fromPt], crvPts[0], crvPts[1]]
                face_00 = []
                for add_pt in add_pts_00:
                    is_exist, index = check_exist_vertex(mesh00, add_pt)
                    if is_exist:
                        key = index
                    else:
                        key = mesh00.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                    face_00.append(key)
                faces_00.append(face_00)

                add_pts_01 = [vertices[posi[0]], vertices[posi[1]], crvPts[1], crvPts[0]]
                face_01 = []
                for add_pt in add_pts_01:
                    is_exist, index = check_exist_vertex(mesh01, add_pt)
                    if is_exist:
                        key = index
                    else:
                        key = mesh01.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                    face_01.append(key)
                face_01_half00 = [face_01[0], face_01[1], face_01[2]]
                face_01_half01 = [face_01[2], face_01[3], face_01[0]]
                faces_01.append(face_01_half00)
                faces_01.append(face_01_half01)

            elif len(posi) == 1:
                fromPt = posi[0]
                from_distance = differences[fromPt]
                for toPt in nega:
                    to_distance = differences[toPt]
                    ## toPt is "vertices[toPt]"
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt] * (abs(to_distance)/dim) + vertices[toPt] * (abs(from_distance)/dim)
                    crvPts.append(point)

                add_pts_01 = [vertices[fromPt], crvPts[0], crvPts[1]]
                face_01 = []
                for add_pt in add_pts_01:
                    is_exist, index = check_exist_vertex(mesh01, add_pt)
                    if is_exist:
                        key = index
                    else:
                        key = mesh01.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                    face_01.append(key)
                faces_01.append(face_01)

                add_pts_00 = [vertices[nega[0]], vertices[nega[1]], crvPts[1], crvPts[0]]
                face_00 = []
                for add_pt in add_pts_00:
                    is_exist, index = check_exist_vertex(mesh00, add_pt)
                    if is_exist:
                        key = index
                    else:
                        key = mesh00.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                    face_00.append(key)
                face_00_half_00 = [face_00[0], face_00[1], face_00[2]]
                face_00_half_01 = [face_00[2], face_00[3], face_00[0]]
                faces_00.append(face_00_half_00)
                faces_00.append(face_00_half_01)

        elif len(posi) == 0:
            face_00 = []
            for v_ind in face:
                v_atri = mesh.vertex_attributes(v_ind)
                pt = Point(x=v_atri["x"], y=v_atri["y"], z=v_atri["z"])
                is_exist, index = check_exist_vertex(mesh00, pt)
                if is_exist:
                    key = index
                else:
                    key = mesh00.add_vertex(x=v_atri["x"], y=v_atri["y"], z=v_atri["z"])
                face_00.append(key)
                # print("v_atri :", v_atri)
            faces_00.append(face_00)

        elif len(nega) == 0:
            face_01 = []
            for v_ind in face:
                v_atri = mesh.vertex_attributes(v_ind)
                pt = Point(x=v_atri["x"], y=v_atri["y"], z=v_atri["z"])
                is_exist, index = check_exist_vertex(mesh01, pt)
                if is_exist:
                    key = index
                else:
                    key = mesh01.add_vertex(x=v_atri["x"], y=v_atri["y"], z=v_atri["z"])
                face_01.append(key)
            faces_01.append(face_01)
    ## finally add faces ##
    for face_00 in faces_00:
        mesh00.add_face(vertices=face_00)
    for face_01 in faces_01:
        mesh01.add_face(vertices=face_01)

    v00, f00 = mesh00.to_vertices_and_faces()
    v01, f01 = mesh01.to_vertices_and_faces()
    print("mesh00 ver_num : %d , face_num : %d " % (len(v00), len(f00)))
    print("mesh01 ver_num : %d , face_num : %d " % (len(v01), len(f01)))

    return mesh00, mesh01












