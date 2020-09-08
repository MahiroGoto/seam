import math
import os

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh

import igl

from seam.utils import utils, primitive
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
            "/20_overlapping_connection_2.0/"

FOLDER = "DATA/"
OBJ_INPUT_NAME = "MESH.obj"

DATA_PATH = PATH + FOLDER


get_splitted_mesh = True
create_layers = True
offset_layers = False


if __name__ == "__main__":

    ### --- Load initial_mesh
    MESH = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_INPUT_NAME))
    print("Vertices : %d , Faces : %d " % (len(list(MESH.vertices())), len(list(MESH.faces()))))

    ### --- Load seams points
    seams_pts_Data = utils.load_from_Json(DATA_PATH, "_seams_pts.json")
    seams_pts = boundary_crv.get_seams_pts_from_data(seams_pts_Data)

    ## organize vertices data (atributes) to convert compas Point ##
    v_keys = list(MESH.vertices(data=False))
    v_atris, faces = MESH.to_vertices_and_faces()
    vertices = [Point(v_atr[0], v_atr[1], v_atr[2]) for v_atr in v_atris]

    if get_splitted_mesh:
        ## get seam vertex vertices ##
        seam_ids_list = boundary_crv.get_seams_vertex_indices_first_and_second(MESH, seams_pts)

        ##############################################
        ## normal distances from each seam vertices ##
        seam_distances_00, seam_distances_01 = seam_crv.get_distances_from_two_seams(MESH, seam_ids_list)

        ## measurement of longest and shortest way and shortest way ##
        longest_v_00, longest_v_01, longest_distance = distance_calculation.get_longest_way_between_two_seams(MESH, seam_ids_list)
        shortest_v_00, shortest_v_01, shortest_distance = distance_calculation.get_shortest_way_between_two_seams(MESH, seam_ids_list)

        ## mid differences ##
        SD_mid = seam_crv.get_distance_differences_between_two_seams(MESH, seam_ids_list, time=0.5)
        mid_crv_pts = seam_crv.get_curve_pts_from_distance_differences_on_Mesh(MESH, SD_mid)
        # utils.save_json(mid_crv_pts, DATA_PATH, "mid_crv_pts.json")
        normal_SD_00 = seam_crv.get_distance_differences_between_two_seams(MESH, seam_ids_list, time=0.5)
        normal_SD_01 = seam_crv.get_distance_differences_between_two_seams(MESH, seam_ids_list, time=0.5)
        ## custom differences ##
        custom_SD_00 = seam_crv.get_distance_differences_with_cos_from_first(MESH, seam_ids_list)
        custom_SD_01 = seam_crv.get_distance_differences_with_cos_from_second(MESH, seam_ids_list)

        ## split the initial mesh into two pieces ##
        piece_00, _ = seam_crv.split_mesh_with_single_differences_crv(MESH, custom_SD_00)
        _, piece_01 = seam_crv.split_mesh_with_single_differences_crv(MESH, custom_SD_01)

        ## get points of the extra part ##
        extention_00, all_vPts_00 = seam_crv.split_mesh_with_double_differences_crvs(MESH, normal_SD_00, custom_SD_00)
        extention_pts_00 = seam_crv.get_extention_part_pts(all_vPts_00, mid_crv_pts)
        extention_01, all_vPts_01 = seam_crv.split_mesh_with_double_differences_crvs(MESH, custom_SD_01, normal_SD_01)
        extention_pts_01 = seam_crv.get_extention_part_pts(all_vPts_01, mid_crv_pts)


        extention_pts_00 = utils.convert_compas_Points_list_to_Data(extention_pts_00)
        extention_pts_01 = utils.convert_compas_Points_list_to_Data(extention_pts_01)
        utils.save_json(extention_pts_00, DATA_PATH, "extention_pts_00.json")
        utils.save_json(extention_pts_01, DATA_PATH, "extention_pts_01.json")

        piece_00.to_obj(os.path.join(DATA_PATH, "piece_00.obj"))
        piece_01.to_obj(os.path.join(DATA_PATH, "piece_01.obj"))


    if create_layers:
        ################
        ## first mesh ##
        """
        input : mesh, seams_data
        """
        piece_00 = Mesh.from_obj(os.path.join(DATA_PATH, "piece_00.obj"))
        print("piece_00 Vertices : %d , Faces : %d " % (len(list(piece_00.vertices())), len(list(piece_00.faces()))))

        piece_00_seams_data = utils.load_from_Json(DATA_PATH, "_piece_00_seams_data.json")
        piece_00_seams_pts = boundary_crv.get_seams_pts_from_data(piece_00_seams_data)
        piece_00_seam_ids_list = boundary_crv.get_seams_vertex_indices_first_and_second(piece_00, piece_00_seams_pts)

        ## check the gap_ratio ##
        trans_bound, gap_ratio = distance_calculation.get_gap_ratio(piece_00, piece_00_seam_ids_list)
        print("gap_ratio_00 :", gap_ratio)

        ## geodesics output ##
        dis00, dis01 = seam_crv.get_distances_from_two_seams(piece_00, piece_00_seam_ids_list)
        utils.save_json(dis00, DATA_PATH, "distance_00.json")
        utils.save_json(dis01, DATA_PATH, "distance_01.json")

        ## get the layer number ##
        layer_num = path_generation.get_layer_number(piece_00, piece_00_seam_ids_list)
        print("piece_00 layer_num :", layer_num)
        ## get layer paths with points ##
        layer_paths_00 = []
        layer_paths_00_data = []
        time_step = 1 / (layer_num-1)
        for i in range(layer_num):
            if i == layer_num - 1:
                time = time_step * i - time_step / 100
            else:
                time = time_step * i
            differences = seam_crv.get_distance_differences_between_two_seams(piece_00, piece_00_seam_ids_list, time)
            layer_pts = seam_crv.get_curve_pts_from_distance_differences_on_Mesh(piece_00, differences)
            ## get layer path pts with params ##
            path_pts = path_generation.get_layer_path_pts_from_list_of_layer_pts_on_mesh(layer_pts)
            layer_paths_00.append(path_pts)
            path_pts_data = utils.convert_compas_Points_list_to_Data(path_pts)
            layer_paths_00_data.append(path_pts_data)
        print(len(layer_paths_00))

        utils.save_json(layer_paths_00_data, DATA_PATH, "layer_paths_00_data.json")

        #################
        ## second mesh ##
        piece_01 = Mesh.from_obj(os.path.join(DATA_PATH, "piece_01.obj"))
        print("piece_01 Vertices: %d, Faces: %d" % (len(list(piece_01.vertices())), len(list(piece_01.faces()))))

        piece_01_seams_data = utils.load_from_Json(DATA_PATH, "_piece_01_seams_data.json")
        piece_01_seams_pts = boundary_crv.get_seams_pts_from_data(piece_01_seams_data)
        piece_01_seam_ids_list = boundary_crv.get_seams_vertex_indices_first_and_second(piece_01, piece_01_seams_pts)

        ## get layer number ##
        layer_num = path_generation.get_layer_number(piece_01, piece_01_seam_ids_list)
        print("piece_01 layer_num :", layer_num)
        ## get layer paths with points ##
        layer_paths_01 = []
        layer_paths_01_data = []
        time_step = 1 / (layer_num-1)
        for i in range(layer_num):
            if i == layer_num - 1:
                time = time_step * i - time_step / 100
            else:
                time = time_step * i
            differences = seam_crv.get_distance_differences_between_two_seams(piece_01, piece_01_seam_ids_list, time)
            layer_pts = seam_crv.get_curve_pts_from_distance_differences_on_Mesh(piece_01, differences)
            ## get layer path pts with params ##
            path_pts = path_generation.get_layer_path_pts_from_list_of_layer_pts_on_mesh(layer_pts)
            layer_paths_01.append(path_pts)
            path_pts_data = utils.convert_compas_Points_list_to_Data(path_pts)
            layer_paths_01_data.append(path_pts_data)
        print(len(layer_paths_01))

        utils.save_json(layer_paths_01_data, DATA_PATH, "layer_paths_01_data.json")

        ###############################################################################################################
        if offset_layers:
            ## calculate distance to the other piece points ##
            pts_cloud_00 = utils.flatten_list_list_to_list(layer_paths_00)
            pts_cloud_01 = utils.flatten_list_list_to_list(layer_paths_01)

            new_layer_paths_00 = []
            new_layer_paths_00_Data = []
            for path_pts in layer_paths_00:
                ## get centrePt of the layer ##
                centre = path_pts[0]
                for i in range(len(path_pts)):
                    if i != 0:
                        centre += path_pts[i]
                centre = centre / len(path_pts)
                new_path_pts = []
                for fromPt in path_pts:
                    # fromPt = path_pts[0]
                    two_clPts, dists = distance_calculation.search_closest_points_from_pts_cloud(fromPt, pts_cloud_01, 2)
                    # closest_pt = (dists[1]/(dists[0]+dists[1])) * two_clPts[0] + (dists[0]/(dists[0]+dists[1])) * two_clPts[1]
                    x = (dists[1]/(dists[0]+dists[1])) * two_clPts[0].x + (dists[0]/(dists[0]+dists[1])) * two_clPts[1].x
                    y = (dists[1]/(dists[0]+dists[1])) * two_clPts[0].y + (dists[0]/(dists[0]+dists[1])) * two_clPts[1].y
                    z = (dists[1]/(dists[0]+dists[1])) * two_clPts[0].z + (dists[0]/(dists[0]+dists[1])) * two_clPts[1].z
                    closest_pt = Point(x, y, z)
                    distance = fromPt.distance_to_point(closest_pt)
                    if distance < 2 and dists[0] < 2:
                        vec = fromPt - centre
                        vec = vec.unitized()
                        newPt = fromPt + vec
                    else:
                        newPt = fromPt
                    new_path_pts.append(newPt)
                new_path_pts_data = utils.convert_compas_Points_list_to_Data(new_path_pts)
                new_layer_paths_00_Data.append(new_path_pts_data)

                new_layer_paths_00.append(new_path_pts)

            utils.save_json(new_layer_paths_00_Data, DATA_PATH, "new_layer_paths_00_Data.json")




            # for path_pts in layer_paths_00:





        # layer_paths_01














































