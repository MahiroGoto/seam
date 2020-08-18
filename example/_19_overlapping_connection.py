import math
import os

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh

import igl

from seam.utils import utils, primitive
from seam.boundary import seam_crv, boundary_crv, distance_calculation
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

DATA_PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/19_overlapping_connection/data_updated/"
OBJ_INPUT_NAME = "mesh.obj"

get_splitted_mesh = False
create_layers = True


if __name__ == "__main__":

    ### --- Load initial_mesh
    initial_mesh = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_INPUT_NAME))
    print("Vertices : %d , Faces : %d " % (len(list(initial_mesh.vertices())), len(list(initial_mesh.faces()))))

    ### --- Load seams points
    seams_pts_Data = utils.load_from_Json(DATA_PATH, "_seams_pts.json")
    seams_pts = seam_crv.get_seams_pts_from_data(seams_pts_Data)

    ## organize vertices data (atributes) to convert compas Point ##
    v_keys = list(initial_mesh.vertices(data=False))
    v_atris, faces = initial_mesh.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]

    if get_splitted_mesh:
        ## get seam vertex vertices ##
        seam_ids_list = seam_crv.get_seams_vertex_indices_first_and_second(initial_mesh, seams_pts)

        ##############################################
        ## normal distances from each seam vertices ##
        seam_distances_00, seam_distances_01 = boundary_crv.get_distances_from_two_seams(initial_mesh, seam_ids_list)

        ## measurement of longest and shortest way and shortest way ##
        longest_v_00, longest_v_01, longest_distance = distance_calculation.get_longest_way_between_two_seams(initial_mesh, seam_ids_list)
        shortest_v_00, shortest_v_01, shortest_distance = distance_calculation.get_shortest_way_between_two_seams(initial_mesh, seam_ids_list)

        ## get two types of distance with cosine curve for interlocking boundary ##
        seam_differences_00 = boundary_crv.get_distance_differences_with_cos_from_first(initial_mesh, seam_ids_list)
        seam_differences_01 = boundary_crv.get_distance_differences_with_cos_from_second(initial_mesh, seam_ids_list)


        ## merching triangles to split the initial mesh into two pieces ##
        splitted_mesh_00, splitted_mesh_00_ = boundary_crv.split_mesh_with_single_differences_crv(initial_mesh, seam_differences_00)
        splitted_mesh_00.to_obj(os.path.join(DATA_PATH, "splitted_mesh_00.obj"))
        # splitted_mesh_00_.to_obj(os.path.join(DATA_PATH, "splitted_mesh_00_.obj"))
        splitted_mesh_01, splitted_mesh_01_ = boundary_crv.split_mesh_with_single_differences_crv(initial_mesh, seam_differences_01)
        # splitted_mesh_01.to_obj(os.path.join(DATA_PATH, "splitted_mesh_01.obj"))
        splitted_mesh_01_.to_obj(os.path.join(DATA_PATH, "splitted_mesh_01_.obj"))


    if create_layers:

        ################
        ## first mesh ##
        first_mesh = Mesh.from_obj(os.path.join(DATA_PATH, "splitted_mesh_00.obj"))
        print("first_mesh Vertices : %d , Faces : %d " % (len(list(first_mesh.vertices())), len(list(first_mesh.faces()))))

        first_piece_seams_data = utils.load_from_Json(DATA_PATH, "first_piece_seams_data.json")
        first_seams_pts = seam_crv.get_seams_pts_from_data(first_piece_seams_data)
        first_seam_ids_list = seam_crv.get_seams_vertex_indices_first_and_second(first_mesh, first_seams_pts)

        ## check the gap_ratio ##
        trans_bound, gap_ratio = distance_calculation.get_gap_ratio(first_mesh, first_seam_ids_list)
        print("gap_ratio :", gap_ratio)

        ## get the layer number ##
        layer_num = distance_calculation.get_layer_number(first_mesh, first_seam_ids_list)
        print("layer_num :", layer_num)

        layers = []
        time_step = 1 / (layer_num)
        for i in range(layer_num):
            time = time_step * i
            print("layer_oder :", i)
            print("time :", time)
            differences = boundary_crv.get_distance_differences_between_two_seams(first_mesh, first_seam_ids_list, time)
            layer_pts = boundary_crv.get_curve_pts_from_distance_differences_on_Mesh(first_mesh, differences)
            layers.append(layer_pts)
            ## repositioning layer pts ##
            centrePt =
        print(len(layers))

        #################
        ## second mesh ##
        second_mesh = Mesh.from_obj(os.path.join(DATA_PATH, "splitted_mesh_01_.obj"))
        print("second_mesh Vertices: %d, Faces: %d" % (len(second_mesh.vertices()), len(list(second_mesh.faces()))))

        second_piece_seams_data = utils.load_from_Json(DATA_PATH, "second_piece_seams_data.json")
        second_seams_pts = seam_crv.get_seams_pts_from_data(second_piece_seams_data)
        second_seam_ids_list = seam_crv.get_seams_vertex_indices_first_and_second()








































