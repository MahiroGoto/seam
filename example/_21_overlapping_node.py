import math
import os

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh
import igl

from seam.utils import utils, primitive, parameters
from seam.boundary import boundary_crv, seam_crv, distance_calculation
from seam.Layer import path_generation
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



PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype" \
            "/21_overlapping_node/"

FOLDER = "DATA/"
OBJ_INPUT_NAME = "MESH.obj"

DATA_PATH = PATH + FOLDER


###################################################
geodesic_compute = True
check = False
layer_compute = True

###################################################

if __name__ == "__main__":

    ## load mesh from obj ##
    MESH = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_INPUT_NAME))
    print("MESH Vertices : %d , Faces : %d " % (len(list(MESH.vertices())), len(list(MESH.faces()))))

    ## load seam vertex keys list ##
    seam_pts_list_data = utils.load_from_Json(DATA_PATH, "_seam_pts_list_data.json")
    seams = boundary_crv.Seams(MESH, seam_pts_list_data)
    seams.get_seams()
    seam_vertex_keys_list = seams.seam_vertex_keys_list

    vkeys = list(MESH.vertices())
    vs, fs = MESH.to_vertices_and_faces()
    vertices = [Point(v_coords[0], v_coords[1], v_coords[2]) for v_coords in vs]

    if geodesic_compute:
        ## momal_geodesics ##
        d_00, d_01 = distance_calculation.get_distances_list_from_every_seams(MESH, seam_vertex_keys_list)
        utils.save_json(d_00, DATA_PATH, "d_00.json")
        utils.save_json(d_01, DATA_PATH, "d_01.json")

    if check:
        ## get seam realms ##
        realm00 = []
        realm01 = []
        for i in range(len(d_00)):
            d00 = d_00[i]
            d01 = d_01[i]
            if d00 < d01:
                realm00.append(i)
            else:
                realm01.append(i)

        ## layer num ##
        Differences = distance_calculation.Differences(MESH, seam_vertex_keys_list)

        short_way = Differences.short_way
        long_way = Differences.long_way
        max_h = parameters.get_param("max_layer_height")
        min_h = parameters.get_param("min_layer_height")
        num = ((short_way / min_h) + (long_way / max_h)) / 2
        num = int(round(num, 0))
        h_small = short_way / num
        h_big = long_way / num
        for i in range(20):
            if h_small >+ min_h and h_big <= max_h:
                break
            elif h_small < min_h:
                num -= 1
                h_small = short_way / num
                h_big = long_way / num
            elif h_big > max_h:
                num += 1
                h_small = short_way / num
                h_big = long_way / num
        layer_num = int(num)
        print("layer_num :", layer_num)

        time_step = 1 / layer_num
        difs_list = []
        ## get differences and isocrv ##
        path_pts_list = []
        path_pts_list_data = []
        for i in range(layer_num):
            if i == layer_num - 1:
                time = time_step * i - time_step/100
            else:
                time = time_step * i
            difs = [(d01 * time - d00 * (1 - time)) for d00, d01 in zip(d_00, d_01)]

            difs_list.append(difs)

            crvPts = seam_crv.get_layer_crvPts_from_distance_differences_on_Mesh(MESH, difs)
            path_pts = seam_crv.create_layer_path_pts_from_crvPts(crvPts)
            path_pts_data = utils.convert_compas_Points_list_to_Data(path_pts)
            path_pts_list_data.append(path_pts_data)
            path_pts_list.append(path_pts)

        print("path_pts_list :", len(path_pts_list))
        utils.save_json(path_pts_list_data, DATA_PATH, "path_pts_list_data.json")


    if layer_compute:
        Layer_generation = path_generation.Layer_generation(MESH, seam_vertex_keys_list)
        layer_num = Layer_generation.layer_num
        half_layer_num = int(round(layer_num/2, 0))

        base_pathPts_list = Layer_generation.create_layers_path_from_difs_two_seams(diftype=0)
        pathPts_list_00 = Layer_generation.create_layers_path_from_difs_two_seams(diftype=1.1)
        pathPts_list_01 = Layer_generation.create_layers_path_from_difs_two_seams(diftype=1.2)

        ## set the original pieces layer path points before offsetting ##
        piece00 = pathPts_list_00[:half_layer_num-1]
        piece01 = pathPts_list_01[half_layer_num+1:]
        first_base = base_pathPts_list[:half_layer_num]
        second_base = base_pathPts_list[half_layer_num:]
        ## piece 00 offset ##
        other_pieces = [piece01]
        offset00 = path_generation.OffsetTool(MESH, piece00, other_pieces, first_base, half_layer_num)
        paths_piece00 = offset00.offset_layer_pathPts()
        ## piece 01 offset ##
        other_pieces = [piece00]
        offset01 = path_generation.OffsetTool(MESH, piece01, other_pieces, second_base, half_layer_num)
        paths_piece01 = offset01.offset_layer_pathPts()

        ## convert to data and save as json file ##
        paths_piece00_data = utils.convert_compas_pts_list_list_to_Data(paths_piece00)
        paths_piece01_data = utils.convert_compas_pts_list_list_to_Data(paths_piece01)
        utils.save_json(paths_piece00_data, DATA_PATH, "paths_piece00.json")
        utils.save_json(paths_piece01_data, DATA_PATH, "paths_piece01.json")











































