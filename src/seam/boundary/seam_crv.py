import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh

from seam.utils import utils, primitive, parameters
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


####################################################################################
## marching triangle for building the boundary curve
####################################################################################
def get_layer_crvPts_from_distance_differences_on_Mesh(mesh, differences):
    v_atris, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]

    ## merching triangles to get the boundary points on the mesh edges ##
    crv_pts_list = []
    centrePts = []
    count = 0
    for face in faces:
        posi = []
        nega = []
        for i, v_key in enumerate(face):
            dif = differences[v_key]
            if dif < 0:
                nega.append(v_key)
            else:
                posi.append(v_key)

        if len(posi) != 0 and len(nega) != 0:
            if len(posi) == 1:
                fromPts = posi
                toPts = nega
            else:
                fromPts = nega
                toPts = posi
        else:
            continue
        fromPt = fromPts[0]
        from_distance = differences[fromPt]
        crvPts = []
        for toPt in toPts:
            to_distance = differences[toPt]
            dim = abs(from_distance) + abs(to_distance)
            point = vertices[fromPt] * (abs(to_distance) / dim) + vertices[toPt] * (abs(from_distance) / dim)
            crvPts.append(point)
        crv_pts_list.append(crvPts)

    all_curve_pts = []
    for crvPts in crv_pts_list:
        for pt in crvPts:
            if pt not in all_curve_pts:
                all_curve_pts.append(pt)
    # get the centre point ##
    pts_x = [pt.x for pt in all_curve_pts]
    pts_y = [pt.y for pt in all_curve_pts]
    pts_z = [pt.z for pt in all_curve_pts]
    centrePt = Point(sum(pts_x) / len(all_curve_pts),
                     sum(pts_y) / len(all_curve_pts),
                     sum(pts_z) / len(all_curve_pts))


    ## create the boundary curve with merching triangles ##
    crv_pts = crv_pts_list[0]
    current = crv_pts[0]
    next = crv_pts[1]
    curve_pts = []
    curve_pts.append(next)
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

                    if next not in curve_pts:
                        curve_pts.append(next)
    return curve_pts, centrePt


def get_layer_crvPts_from_distance_attributes_on_Mesh(mesh, attributes, difs, base_seam_num, path_sequence=True):
    v_atris, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]
    crv_pts_list = []
    # print("hey!!!!!")
    for face in faces:
        attri_00 = []
        attri_01 = []
        attri_02 = []
        for i, v_key in enumerate(face):
            attr = attributes[v_key]
            if attr == 0:
                attri_00.append(v_key)
            elif attr == 1:
                attri_01.append(v_key)
            elif attr == 2:
                attri_02.append(v_key)
        # print("attri :", attri_00, attri_01, attri_02)
        ## set the base attribute ##
        if base_seam_num == 0:
            base_attri = attri_00
        elif base_seam_num == 1:
            base_attri = attri_01
        else:
            base_attri = attri_02
        ## check if the face is on the boundary ##
        if max([len(attri_00), len(attri_01), len(attri_02)]) == 3:
            continue
        elif max([len(attri_00), len(attri_01), len(attri_02)]) == 2:
            ## have to consider only if base_attri is not empty ##
            if len(base_attri) != 0:
                if len(attri_00) == 2:
                    toVers = attri_00
                elif len(attri_01) == 2:
                    toVers = attri_01
                else:
                    toVers = attri_02
            else:
                continue
            ## set the curve points ##
            for v_key in face:
                if v_key not in toVers:
                    fromVer = v_key
                    from_dif = difs[fromVer]
            ## get crvPt on these edges of the face ##
            crvPts = []
            for toVer in toVers:
                to_dif = difs[toVer]
                dim = abs(from_dif) + abs(to_dif)
                point = vertices[fromVer] * (abs(to_dif) / dim) + vertices[toVer] * (abs(from_dif) / dim)
                crvPts.append(point)
            crv_pts_list.append(crvPts)
        elif max([len(attri_00), len(attri_01), len(attri_02)]) == 1:
            fromVer = base_attri[0]
            from_dif = difs[fromVer]
            temp_Vlist = [attri_00[0], attri_01[0], attri_02[0]]
            toVers = []
            for v_attr in temp_Vlist:
                if v_attr != fromVer: toVers.append(v_attr)
            ## create the point on the face edge ##
            crvPts = []
            for toVer in toVers:
                to_dif = difs[toVer]
                dim = abs(from_dif) + abs(to_dif)
                point = vertices[fromVer] * (abs(to_dif) / dim) + vertices[toVer] * (abs(from_dif) / dim)
                crvPts.append(point)
            crv_pts_list.append(crvPts)

    # print("crv_pts_list num :", len(crv_pts_list))
    all_curve_pts = []
    for crvPts in crv_pts_list:
        for pt in crvPts:
            if pt not in all_curve_pts:
                all_curve_pts.append(pt)
    # get the centre point ##
    pts_x = [pt.x for pt in all_curve_pts]
    pts_y = [pt.y for pt in all_curve_pts]
    pts_z = [pt.z for pt in all_curve_pts]
    centrePt = Point(sum(pts_x) / len(all_curve_pts),
                     sum(pts_y) / len(all_curve_pts),
                     sum(pts_z) / len(all_curve_pts))

    if path_sequence:
        ## create the boundary curve
        crv_pts = crv_pts_list[0]
        current = crv_pts[0]
        next = [crv_pts[1]]
        curve_pts = []
        curve_pts.append(next[0])
        exist = [0]
        for i in range(len(crv_pts_list)):
            for j, pts in enumerate(crv_pts_list):
                ## j is the number of the face ##
                if j not in exist:
                    connect = False
                    ## check if this face is the next or not ##
                    for nextpt in next:
                        if nextpt in pts: connect = True
                    if connect:
                        exist.append(j)
                        crv_pts = pts
                        if crv_pts[0] in next:
                            next = [crv_pts[1]]
                        else:
                            next = [crv_pts[0]]

                        for pt in crv_pts:
                            if pt not in curve_pts:
                                curve_pts.append(pt)
                    else: pass
    ## this is the case the you do not organize to get centre point ##
    else:
        curve_pts = all_curve_pts

    return curve_pts, centrePt


def get_layer_crvPts_from_distance_four_attributes_on_Mesh(mesh, attributes, difs, base_seam_num, path_sequence=True):
    v_atris, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]
    crv_pts_list = []
    # print("hey!!!!!")
    for face in faces:
        attri_00 = []
        attri_01 = []
        attri_02 = []
        attri_03 = []
        for i, v_key in enumerate(face):
            attr = attributes[v_key]
            if attr == 0:
                attri_00.append(v_key)
            elif attr == 1:
                attri_01.append(v_key)
            elif attr == 2:
                attri_02.append(v_key)
            elif attr == 3:
                attri_03.append(v_key)
        ## set the base attribute ##
        if base_seam_num == 0:
            base_attri = attri_00
        elif base_seam_num == 1:
            base_attri = attri_01
        elif base_seam_num == 2:
            base_attri = attri_02
        else:
            base_attri = attri_03
        ## check if the face is on the boundary ##
        if max([len(attri_00), len(attri_01), len(attri_02), len(attri_03)]) == 3:
            continue
        elif max([len(attri_00), len(attri_01), len(attri_02), len(attri_03)]) == 2:
            ## have to consider only if base_attri is not empty ##
            if len(base_attri) != 0:
                if len(attri_00) == 2:
                    toVers = attri_00
                elif len(attri_01) == 2:
                    toVers = attri_01
                elif len(attri_02) == 2:
                    toVers = attri_02
                else:
                    toVers = attri_03
            else:
                continue
            ## set the curve points ##
            for v_key in face:
                if v_key not in toVers:
                    fromVer = v_key
                    from_dif = difs[fromVer]
            ## get crvPt on these edges of the face ##
            crvPts = []
            for toVer in toVers:
                to_dif = difs[toVer]
                dim = abs(from_dif) + abs(to_dif)
                point = vertices[fromVer] * (abs(to_dif) / dim) + vertices[toVer] * (abs(from_dif) / dim)
                crvPts.append(point)
            crv_pts_list.append(crvPts)
        elif max([len(attri_00), len(attri_01), len(attri_02), len(attri_03)]) == 1:
            if len(base_attri) != 0:
                fromVer = base_attri[0]
                from_dif = difs[fromVer]
                toVers = []
                for v_key in face:
                    if v_key != fromVer: toVers.append(v_key)
                ## create the point on the face edge ##
                crvPts = []
                for toVer in toVers:
                    to_dif = difs[toVer]
                    dim = abs(from_dif) + abs(to_dif)
                    point = vertices[fromVer] * (abs(to_dif) / dim) + vertices[toVer] * (abs(from_dif) / dim)
                    crvPts.append(point)
                crv_pts_list.append(crvPts)
            else:
                continue
    # print("crv_pts_list num :", len(crv_pts_list))
    all_curve_pts = []
    for crvPts in crv_pts_list:
        for pt in crvPts:
            if pt not in all_curve_pts:
                all_curve_pts.append(pt)
    # get the centre point ##
    pts_x = [pt.x for pt in all_curve_pts]
    pts_y = [pt.y for pt in all_curve_pts]
    pts_z = [pt.z for pt in all_curve_pts]
    centrePt = Point(sum(pts_x) / len(all_curve_pts),
                     sum(pts_y) / len(all_curve_pts),
                     sum(pts_z) / len(all_curve_pts))

    if path_sequence:
        ## create the boundary curve
        crv_pts = crv_pts_list[0]
        current = crv_pts[0]
        next = [crv_pts[1]]
        curve_pts = []
        curve_pts.append(next[0])
        exist = [0]
        for i in range(len(crv_pts_list)):
            for j, pts in enumerate(crv_pts_list):
                ## j is the number of the face ##
                if j not in exist:
                    connect = False
                    ## check if this face is the next or not ##
                    for nextpt in next:
                        if nextpt in pts: connect = True
                    if connect:
                        exist.append(j)
                        crv_pts = pts
                        if crv_pts[0] in next:
                            next = [crv_pts[1]]
                        else:
                            next = [crv_pts[0]]

                        for pt in crv_pts:
                            if pt not in curve_pts:
                                curve_pts.append(pt)
                    else: pass
    ## this is the case the you do not organize to get centre point ##
    else:
        curve_pts = all_curve_pts


    return curve_pts, centrePt


##################################
## layer path generation after calculate layers ##
##################################
def layer_curve_length(layer_pts):
    pts_string = zip(layer_pts[:-1], layer_pts[1:])
    Length_measure = 0
    for pt, pt_next in pts_string:
        length = pt.distance_to_point(pt_next)
        Length_measure += length
    Length_measure += layer_pts[-1].distance_to_point(layer_pts[0])
    crv_length = Length_measure
    return crv_length

def get_point_on_polyline_pts_list_with_length(poly_pts, aim_length):
    global aim_pt
    length = layer_curve_length(poly_pts)
    # aim = length * param
    aim = aim_length

    pts_string = zip(poly_pts[:-1], poly_pts[1:])
    measure = 0
    for pre_pt, next_pt in pts_string:
        distance = pre_pt.distance_to_point(next_pt)
        measure += distance
        if measure > aim:
            pre_measure = measure - distance
            left_length = aim - pre_measure
            ratio = left_length / distance

            aim_pt = pre_pt * (1-ratio) + next_pt * ratio
            break
    return aim_pt


def create_layer_path_pts_from_crvPts(crvPts, double_iteration=True, first_resolution=0.7):
    """
    get path points from layer points
    for generation robotic point on one layer path
    by param of length
    """
    firstPt = crvPts[0]
    crvPts.append(firstPt)
    if double_iteration:
        ## first iteration ##
        crv_length = layer_curve_length(crvPts)
        print("crv_length :", crv_length)
        path_pt_span = parameters.get_param("layer_path_pt_span")
        path_pt_span = path_pt_span * first_resolution
        path_pt_num = int(round(crv_length / path_pt_span, 0))
        actual_pt_span = crv_length / path_pt_num
        path_pts = []
        for i in range(path_pt_num):
            aim_length = actual_pt_span * i
            path_pt = get_point_on_polyline_pts_list_with_length(crvPts, aim_length)
            if path_pt not in path_pts:
                path_pts.append(path_pt)

        ## second iteration ##
        path_pts.append(path_pts[0])
        crv_length = layer_curve_length(path_pts)
        path_pt_span = parameters.get_param("layer_path_pt_span")
        path_pt_span = path_pt_span * 1.0
        path_pt_num = int(round(crv_length / path_pt_span, 0))
        actual_pt_span = crv_length / path_pt_num
        second_path_pts = []
        for i in range(path_pt_num):
            aim_length = actual_pt_span * i
            path_pt = get_point_on_polyline_pts_list_with_length(path_pts, aim_length)
            if path_pt not in second_path_pts:
                second_path_pts.append(path_pt)
        path_pts = second_path_pts
    else:
        ## only first iteration ##
        crv_length = layer_curve_length(crvPts)
        path_pt_span = parameters.get_param("layer_path_pt_span")
        path_pt_num = int(round(crv_length / path_pt_span, 0))
        actual_pt_span = crv_length / path_pt_num
        path_pts = []
        for i in range(path_pt_num):
            aim_length = actual_pt_span * i
            path_pt = get_point_on_polyline_pts_list_with_length(crvPts, aim_length)
            if path_pt not in path_pts:
                path_pts.append(path_pt)



    return path_pts




### finish ###


"""
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

def split_mesh_with_double_differences_crvs(mesh, differences01, differences02):
    v_atrs, faces = mesh.to_vertices_and_faces()
    ## split mesh ##
    # mesh00 = Mesh()
    mesh01 = Mesh()
    # mesh02 = Mesh()
    # faces_00 = []
    faces_01 = []
    # faces_02 = []
    vertices = [Point(x=v_atr[0], y=v_atr[1], z=v_atr[2]) for v_atr in v_atrs]
    for face in faces:
        ## for differences01 ##
        nega01 = []
        posi01 = []
        ## for differences02 ##
        nega02 = []
        posi02 = []
        for v_ind in face:
            diffe01 = differences01[v_ind]
            if diffe01 < 0:
                nega01.append(v_ind)
            elif diffe01 >= 0:
                posi01.append(v_ind)
            diffe02 = differences02[v_ind]
            if diffe02 < 0:
                nega02.append(v_ind)
            elif diffe02 >= 0:
                posi02.append(v_ind)

        if len(nega01)==0: d_01 = 1
        elif len(posi01)==0: d_01 = -1
        else: d_01 = 0

        if len(nega02)==0: d_02 = 1
        elif len(posi02)==0: d_02 = -1
        else: d_02 = 0

        ## seach the area ##
        evaluation = d_01 * d_02

        if evaluation == -1:
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

        elif evaluation == 0:
            crvPts = []
            quad = False
            if d_01 == 0 and d_02 != 0:
                differences = [d for d in differences01]
                nega = nega01
                posi = posi01
            elif d_02 == 0 and d_01 != 0:
                differences = [-d for d in differences02]
                nega = nega02
                posi = posi02
            else:
                ## this is the case that all cutting curve on this face at the same time ##
                quad = True

            if quad:
                add_quad_pts = []
                face_01 = []
                ## get cutting points of differences01 ##
                if len(nega01)==1:
                    fromList = nega01
                    toList = posi01
                else:
                    fromList = posi01
                    toList = nega01
                fromPt01 = fromList[0]
                from_distance = differences01[fromPt01]
                for toPt in toList:
                    to_distance = differences01[toPt]
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt01] * (abs(to_distance) / dim) + vertices[toPt] * (abs(from_distance) / dim)
                    add_quad_pts.append(point)

                ## get cutting points of differences02 ##
                if len(nega02)==1:
                    fromList = nega02
                    toList = posi02
                else:
                    fromList = posi02
                    toList = nega02
                fromPt02 = fromList[0]
                from_distance = differences02[fromPt02]
                for toPt in toList:
                    to_distance = differences02[toPt]
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt02] * (abs(to_distance) / dim) + vertices[toPt] * (abs(from_distance) / dim)
                    add_quad_pts.append(point)

                for add_pt in add_quad_pts:
                    is_exist, index = check_exist_vertex(mesh01, add_pt)
                    if is_exist:
                        key = index
                    else:
                        key = mesh01.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                    face_01.append(key)

                if fromPt01 == fromPt02:
                    f_01 = [face_01[0], face_01[1], face_01[2]]
                    f_02 = [face_01[3], face_01[2], face_01[1]]
                    faces_01.append(f_01)
                    faces_01.append(f_02)
                else:
                    for id in face:
                        if id != fromPt01 and id != fromPt02:
                            anotherPt = vertices[id]
                            is_exist, index = check_exist_vertex(mesh01, anotherPt)
                            if is_exist:
                                another_key = index
                            else:
                                another_key = mesh01.add_vertex(x=anotherPt.x, y=anotherPt.y, z=anotherPt.z)
                            break
                    f_01 = [face_01[1], face_01[0], another_key]
                    f_02 = [face_01[2], face_01[3], another_key]
                    f_04 = [face_01[0], face_01[2], another_key]
                    faces_01.append(f_01)
                    faces_01.append(f_02)
                    faces_01.append(f_04)

            else:
                if len(nega) == 1:
                    fromList = nega
                    toList = posi
                else:
                    fromList = posi
                    toList = nega
                fromPt = fromList[0]
                from_distance = differences[fromPt]
                if from_distance > 0: triangle = True
                else: triangle = False
                for toPt in toList:
                    to_distance = differences[toPt]
                    dim = abs(from_distance) + abs(to_distance)
                    point = vertices[fromPt] * (abs(to_distance) / dim) + vertices[toPt] * (abs(from_distance) / dim)
                    crvPts.append(point)

                if triangle:
                    add_pts = [crvPts[0], crvPts[1], vertices[fromPt]]
                    face_01 = []
                    for add_pt in add_pts:
                        is_exist, index = check_exist_vertex(mesh01, add_pt)
                        if is_exist:
                            key = index
                        else:
                            key = mesh01.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                        face_01.append(key)
                    faces_01.append(face_01)
                else:
                    add_pts = [vertices[toList[0]], vertices[toList[1]], crvPts[1], crvPts[0]]
                    face_01 = []
                    for add_pt in add_pts:
                        is_exist, index = check_exist_vertex(mesh01, add_pt)
                        if is_exist:
                            key = index
                        else:
                            key = mesh01.add_vertex(x=add_pt.x, y=add_pt.y, z=add_pt.z)
                        face_01.append(key)
                    f_01 = [face_01[2], face_01[1], face_01[0]]
                    f_01_ = [face_01[0], face_01[3], face_01[2]]
                    faces_01.append(f_01)
                    faces_01.append(f_01_)

    ## add faces ##
    for f in faces_01:
        mesh01.add_face(f)

    v, fs = mesh01.to_vertices_and_faces()
    all_v = [Point(x=v_atr[0], y=v_atr[1], z=v_atr[2]) for v_atr in v]

    return mesh01, all_v

def get_extention_part_pts(extention_pts, mid_crv_pts):
    pts = []
    for extentionPt in extention_pts:
        dis, clPt = distance_calculation.get_distance_to_pts_cloud(extentionPt, mid_crv_pts)
        if dis > 0.01:
            pts.append(extentionPt)
    return pts

"""






















