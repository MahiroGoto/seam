import math
import os

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import Mesh
import igl

from seam.utils import utils, parameters
from seam.boundary import boundary_crv, seam_crv, distance_calculation
from seam.Layer import path_generation
from seam.Branch import discrete_curve, skeleton_data

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################

PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/" \
       "08_after the project/02_four branching connection/"

PIECE_FOLDER = "pieces/"
###################################################
layer_compute = True
offset = True

merge_modification = False
###################################################


## MESH for layer generation setting ##
if layer_compute:

    DATA_PATH = PATH + PIECE_FOLDER

    OBJ_MESH_NAME = ["mesh_00.obj"]


    for j, OBJ_MESH in enumerate(OBJ_MESH_NAME):
        #######################################
        MESH = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_MESH))
        print("MESH Vertices : %d , Faces : %d " % (len(list(MESH.vertices())), len(list(MESH.faces()))))
        ## seam information load from gh ##
        SEAM_NAME = "_seam_pts_list_data_0" + str(j) + ".json"
        boundary_pts_list_data = utils.load_from_Json(DATA_PATH, SEAM_NAME)
        boundary = boundary_crv.Boundary(MESH, boundary_pts_list_data)
        boundary.get_boundaries()
        boundary_vertex_keys_list = boundary.boundary_vertex_keys_list
        boundary_num = len(boundary_vertex_keys_list)
        print("boundary num :", boundary_num)
        ########################
        ## mesh data organize ##
        # vkeys = list(MESH.vertices())
        # vs, fs = MESH.to_vertices_and_faces()
        # vertices = [Point(v_coords[0], v_coords[1], v_coords[2]) for v_coords in vs]
        #####################################
        ## momal_geodesics from every seam ##
        dis_list = distance_calculation.get_distances_list_from_every_boundary(MESH, boundary_vertex_keys_list)
        logger.info("number of seam :" + str(len(dis_list)))
        for i, dis in enumerate(dis_list):
            utils.save_json(dis, DATA_PATH, "d_0"+str(i)+".json")


        #####################################
        if boundary_num == 2:
            untilTime = 0.51
            Sequence = path_generation.Get_Sequence(MESH, boundary_vertex_keys_list)
            layer_num = Sequence.layer_num
            half_layer_num = int(round((layer_num)/2, 0))
            minValue_00 = 0.5
            minValue_01 = 1.0
            ## custom layer path configuration ##
            sequencePts_000 = Sequence. \
                create_sequence_paths_from_difs_two_boundaries(base_boundary_num=0,
                                                               untilTime=untilTime,
                                                               minValue=minValue_00, frequency=-2.8,
                                                               matchPathDirection=True,
                                                               modifyStartingPathPt=True)
            sequencePts_001 = Sequence. \
                create_sequence_paths_from_difs_two_boundaries(base_boundary_num=1,
                                                               untilTime=untilTime,
                                                               minValue=minValue_01, frequency=-2.8,
                                                               matchPathDirection=True,
                                                               modifyStartingPathPt=True)
            if offset:
                print("untilTime :", untilTime)
                # utils.interrupt()
                ## first offset ##
                other_pieces = [sequencePts_000]
                first_offset = path_generation.OffsetTool(sequencePts_001, other_pieces)
                sequencePts_001 = first_offset.offset_layer_pathPts()

            ## convert and save it as json file ##
            sequencePts_00_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_000)
            SEQUENCE_NAME_00 = "sequencePts_0" + str(j) + "_00_data.json"
            utils.save_json(sequencePts_00_data, DATA_PATH, SEQUENCE_NAME_00)

            sequencePts_01_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
            SEQUENCE_NAME_01 = "sequencePts_0" + str(j) + "_01_data.json"
            utils.save_json(sequencePts_01_data, DATA_PATH, SEQUENCE_NAME_01)


        #####################################
        elif boundary_num == 3:
            untilTime = 0.505
            addLayer = 0
            modify = True
            Sequence = path_generation.Get_Sequence(MESH, boundary_vertex_keys_list)
            layer_num = Sequence.layer_num
            ## compute layer sequence with distance attributes of three boundary curves ##
            sequencePts_000 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=0,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.7, frequency=-1.5, longWayExtention=True,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            """
            ## base_boundary_num=0,minValue=0.75, frequency=1.8, 
            longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
            """
            sequencePts_001 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=1,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.78, frequency=-1.8, longWayExtention=False,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            """
            ## base_boundary_num=1, minValue=0.85, frequency=1.0, 
            longWayExtention=True, matchPathDirection=True, modifyStartingPathPt=True)
            """
            sequencePts_002 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=2,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.78, frequency=-1.8, longWayExtention=False,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            """
            ## base_boundary_num=2,minValue=0.75, frequency=1.8, 
            longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
            """
            ## offset layer sequence ##
            if offset:
                print("untilTime :", untilTime)
                # utils.interrupt()
                ## first offset ##
                other_pieces = [sequencePts_001]
                first_offset = path_generation.OffsetTool(sequencePts_002, other_pieces)
                sequencePts_002 = first_offset.offset_layer_pathPts()
                ## second offset ##
                other_pieces = [sequencePts_001]
                second_offset = path_generation.OffsetTool(sequencePts_000, other_pieces)
                sequencePts_000 = second_offset.offset_layer_pathPts()
                ## third offset ##
                other_pieces = [sequencePts_002]
                second_offset = path_generation.OffsetTool(sequencePts_000, other_pieces)
                sequencePts_000 = second_offset.offset_layer_pathPts()

            ## convert and save as json ##
            sequencePts_000_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_000)
            SEQUENCE_NAME_00 = "sequencePts_0" + str(j) + "_00_data.json"
            utils.save_json(sequencePts_000_data, DATA_PATH, SEQUENCE_NAME_00)

            sequencePts_001_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
            SEQUENCE_NAME_01 = "sequencePts_0" + str(j) + "_01_data.json"
            utils.save_json(sequencePts_001_data, DATA_PATH, SEQUENCE_NAME_01)

            sequencePts_002_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_002)
            SEQUENCE_NAME_02 = "sequencePts_0" + str(j) + "_02_data.json"
            utils.save_json(sequencePts_002_data, DATA_PATH, SEQUENCE_NAME_02)


        #####################################
        elif boundary_num == 4:
            untilTime = 0.505
            addLayer = 0
            modify = True
            Sequence = path_generation.Get_Sequence(MESH, boundary_vertex_keys_list)
            layer_num = Sequence.layer_num
            ## compute layer sequence with distance attributes of four boundary curves ##
            sequencePts_000 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=0,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.85, frequency=-2.3, longWayExtention=False,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            sequencePts_001 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=1,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.85, frequency=-1.8, longWayExtention=False,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            sequencePts_002 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=2,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.85, frequency=-2.3, longWayExtention=True,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            sequencePts_003 = Sequence.\
                create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=3,
                                                                                untilTime=untilTime, addLayer=addLayer,
                                                                                minValue=0.82, frequency=-1.8, longWayExtention=False,
                                                                                matchPathDirection=modify,
                                                                                modifyStartingPathPt=modify)
            if offset:
                print("untilTime :", untilTime)
                # utils.interrupt()
                ## first offset ##
                other_pieces = [sequencePts_001]
                first_offset = path_generation.OffsetTool(sequencePts_002, other_pieces)
                sequencePts_002 = first_offset.offset_layer_pathPts()
                ## second offset ##
                other_pieces = [sequencePts_001]
                second_offset = path_generation.OffsetTool(sequencePts_000, other_pieces)
                sequencePts_000 = second_offset.offset_layer_pathPts()
                ## third offset ##
                other_pieces = [sequencePts_002]
                second_offset = path_generation.OffsetTool(sequencePts_000, other_pieces)
                sequencePts_000 = second_offset.offset_layer_pathPts()
                ## fourth offset ##
                other_pieces = [sequencePts_003]
                fourth_offset = path_generation.OffsetTool(sequencePts_001, other_pieces)
                sequencePts_001 = fourth_offset.offset_layer_pathPts()
                ## fifth offset ##
                other_pieces = [sequencePts_003]
                fifth_offset = path_generation.OffsetTool(sequencePts_002, other_pieces)
                sequencePts_002 = fifth_offset.offset_layer_pathPts()


            ## convert and save as json ##
            sequencePts_000_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_000)
            SEQUENCE_NAME_00 = "sequencePts_0" + str(j) + "_00_data.json"
            utils.save_json(sequencePts_000_data, DATA_PATH, SEQUENCE_NAME_00)

            sequencePts_001_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
            SEQUENCE_NAME_01 = "sequencePts_0" + str(j) + "_01_data.json"
            utils.save_json(sequencePts_001_data, DATA_PATH, SEQUENCE_NAME_01)

            sequencePts_002_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_002)
            SEQUENCE_NAME_02 = "sequencePts_0" + str(j) + "_02_data.json"
            utils.save_json(sequencePts_002_data, DATA_PATH, SEQUENCE_NAME_02)

            sequencePts_003_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_003)
            SEQUENCE_NAME_03 = "sequencePts_0" + str(j) + "_03_data.json"
            utils.save_json(sequencePts_003_data, DATA_PATH, SEQUENCE_NAME_03)


if merge_modification:
    DATA_PATH = PATH + PIECE_FOLDER
    merged_sequence_data_names = ["_merged_sequence_00_data.json"]
    for k, merged_sequence_data_name in enumerate(merged_sequence_data_names):

        ## load ##
        merged_sequence_data = utils.load_from_Json(DATA_PATH, merged_sequence_data_name)
        merged_sequence = utils.convert_data_pts_list_list_to_compas_pts(merged_sequence_data)

        ## modify the merged sequcence ##
        MergeSequence = path_generation.MergedSequence(merged_sequence)
        merged_sequence = MergeSequence.modified_merged_sequence

        ## convert and save json ##
        merged_sequence_00_data = utils.convert_compas_pts_list_list_to_Data(merged_sequence)
        MERGED_SEQUENCE_NAME = "merged_sequence_0" + str(k) + "_data.json"
        utils.save_json(merged_sequence_00_data, DATA_PATH, MERGED_SEQUENCE_NAME)



























