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
       "3.1_final_demo/12_branching_family/"

PIECE_FOLDER = "piece_00/"
###################################################
node_skeleton = False
layer_compute = True
offset = False
modify_merging = False
###################################################


##################################################################################################################################
## first step ####################################################################################################################
## skeleton setting for hole shape ##
if node_skeleton:

    OBJ_NODE_00 = "node.obj"

    OBJ_NODE = OBJ_NODE_00
    NODE_DATA_PATH = PATH + PIECE_FOLDER
    ######################################
    ## load mesh from obj ##
    nodeMESH = Mesh.from_obj(os.path.join(NODE_DATA_PATH, OBJ_NODE))
    print("MESH Vertices : %d , Faces : %d " % (len(list(nodeMESH.vertices())), len(list(nodeMESH.faces()))))

    ## load seam vertex keys list ##
    boundary_pts_list_data = utils.load_from_Json(NODE_DATA_PATH, "_seam_pts_list_data.json")
    boundary = boundary_crv.Boundary(nodeMESH, boundary_pts_list_data)
    boundary.get_boundaries()
    boundary_vertex_keys_list = boundary.boundary_vertex_keys_list

    vkeys = list(nodeMESH.vertices())
    vs, fs = nodeMESH.to_vertices_and_faces()
    vertices = [Point(v_coords[0], v_coords[1], v_coords[2]) for v_coords in vs]

    #####################
    ## momal_geodesics ##
    #####################
    dis_list = distance_calculation.get_distances_list_from_every_boundary(nodeMESH, boundary_vertex_keys_list)
    logger.info("number of seam :" + str(len(dis_list)))
    for i, dis in enumerate(dis_list):
        utils.save_json(dis, NODE_DATA_PATH, "d_0" + str(i) + ".json")

    ## creation of skeleton to get the splitting position ##
    seam_centrePt_list, skeletonPts_list = boundary_crv. \
        get_boundary_centrePts_list_on_branching_node(nodeMESH, dis_list, max_radius=18, even_distance=False)
    seam_centrePt_list_data = utils.convert_compas_pts_list_to_Data(seam_centrePt_list)
    skeletonPts_list_data = utils.convert_compas_pts_list_list_to_Data(skeletonPts_list)
    ## save json file ##
    utils.save_json(seam_centrePt_list_data, NODE_DATA_PATH, "seam_centrePt_list_data.json")
    utils.save_json(skeletonPts_list_data, NODE_DATA_PATH, "skeletonPts_list_data.json")

    ## load skeleton data from gh ##
    skeleton_topology = utils.load_from_Json(NODE_DATA_PATH, "_Skeleton.json")
    all_vertices_data = utils.load_from_Json(NODE_DATA_PATH, "_all_vertices_data.json")
    start_and_node_vertices = utils.load_from_Json(NODE_DATA_PATH, "_start_and_node_vertices.json")
    print("start_and_node_vertices :", start_and_node_vertices)
    ## skeleton and calculate information of it such as vectors and planes ##
    skeleton = skeleton_data.Skeleton(skeleton_topology, all_vertices_data, start_and_node_vertices)
    branch_keys = skeleton.branch_keys
    planes_const_without_node = skeleton.vPlanes_const_without_node
    planes_data_without_node = skeleton.vPlanes_data_without_node
    branch_vertex_dict = skeleton.branch_vertex_dict
    print("branch_vertex_dict :", branch_vertex_dict)

    utils.save_json(branch_vertex_dict, NODE_DATA_PATH, "branch_vertices_dict.json")
    utils.save_json(branch_keys, NODE_DATA_PATH, "branch_keys.json")
    utils.save_json(planes_data_without_node, NODE_DATA_PATH, "planes_data.json")


####################################################################################################################################
## second step #####################################################################################################################
## MESH for layer generation setting ##

if layer_compute:

    OBJ_MESH_NAME = "MESH.obj"
    OBJ_MESH = OBJ_MESH_NAME

    # FOLDER = "data_00/"

    DATA_PATH = PATH + PIECE_FOLDER
    #######################################
    MESH = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_MESH))
    print("MESH Vertices : %d , Faces : %d " % (len(list(MESH.vertices())), len(list(MESH.faces()))))
    ## seam information load from gh ##
    boundary_pts_list_data = utils.load_from_Json(DATA_PATH, "_seam_pts_list_data.json")
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
        untilTime = 0.5
        Sequence = path_generation.Get_Sequence(MESH, boundary_vertex_keys_list)
        layer_num = Sequence.layer_num
        half_layer_num = int(round((layer_num)/2, 0))
        ## custom layer path configuration ##
        sequencePts_000 = Sequence. \
            create_sequence_paths_from_difs_two_boundaries(base_boundary_num=0,
                                                           untilTime=untilTime,
                                                           minValue=1.0, frequency=-1.7,
                                                           matchPathDirection=True,
                                                           modifyStartingPathPt=True)
        sequencePts_001 = Sequence. \
            create_sequence_paths_from_difs_two_boundaries(base_boundary_num=1,
                                                           untilTime=untilTime,
                                                           minValue=0.35, frequency=-2.8,
                                                           matchPathDirection=True,
                                                           modifyStartingPathPt=True)
        if offset:
            print("untilTime :", untilTime)
            utils.interrupt()
            ## first offset ##
            other_pieces = [sequencePts_001]
            first_offset = path_generation.OffsetTool(sequencePts_000, other_pieces)
            sequencePts_000 = first_offset.offset_layer_pathPts()

        ## convert and save it as json file ##
        sequencePts_000_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_000)
        utils.save_json(sequencePts_000_data, DATA_PATH, "sequencePts_000_data.json")
        sequencePts_001_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
        utils.save_json(sequencePts_001_data, DATA_PATH, "sequencePts_001_data.json")

    elif boundary_num == 3:
        untilTime = 0.5
        addLayer = 0
        modify = True
        Sequence = path_generation.Get_Sequence(MESH, boundary_vertex_keys_list)
        layer_num = Sequence.layer_num
        ## compute layer sequence with distance attributes of three boundary curves ##
        sequencePts_000 = Sequence.\
            create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=0,
                                                                            untilTime=untilTime, addLayer=addLayer,
                                                                            minValue=0.85, frequency=-1.0, longWayExtention=True,
                                                                            matchPathDirection=modify,
                                                                            modifyStartingPathPt=modify)
        """
        ## base_boundary_num=0,minValue=0.75, frequency=1.8, 
        longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
        """
        sequencePts_001 = Sequence.\
            create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=1,
                                                                            untilTime=untilTime, addLayer=addLayer,
                                                                            minValue=0.85, frequency=-2.3, longWayExtention=False,
                                                                            matchPathDirection=modify,
                                                                            modifyStartingPathPt=modify)
        """
        ## base_boundary_num=1, minValue=0.85, frequency=1.0, 
        longWayExtention=True, matchPathDirection=True, modifyStartingPathPt=True)
        """
        sequencePts_002 = Sequence.\
            create_sequence_paths_from_distance_attributes_multi_boundaries(base_boundary_num=2,
                                                                            untilTime=untilTime, addLayer=addLayer,
                                                                            minValue=0.85, frequency=-1.8, longWayExtention=False,
                                                                            matchPathDirection=modify,
                                                                            modifyStartingPathPt=modify)
        """
        ## base_boundary_num=2,minValue=0.75, frequency=1.8, 
        longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
        """
        ## offset layer sequence ##
        if offset:
            print("untilTime :", untilTime)
            utils.interrupt()
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
        utils.save_json(sequencePts_000_data, DATA_PATH, "sequencePts_000_data.json")
        sequencePts_001_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
        utils.save_json(sequencePts_001_data, DATA_PATH, "sequencePts_001_data.json")
        sequencePts_002_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_002)
        utils.save_json(sequencePts_002_data, DATA_PATH, "sequencePts_002_data.json")

    elif boundary_num == 4:
        untilTime = 0.5
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
                                                                            minValue=0.85, frequency=-2.3, longWayExtention=False,
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
                                                                            minValue=0.85, frequency=-2.3, longWayExtention=False,
                                                                            matchPathDirection=modify,
                                                                            modifyStartingPathPt=modify)
        if offset:
            print("untilTime :", untilTime)
            utils.interrupt()
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
        utils.save_json(sequencePts_000_data, DATA_PATH, "sequencePts_000_data.json")
        sequencePts_001_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_001)
        utils.save_json(sequencePts_001_data, DATA_PATH, "sequencePts_001_data.json")
        sequencePts_002_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_002)
        utils.save_json(sequencePts_002_data, DATA_PATH, "sequencePts_002_data.json")
        sequencePts_003_data = utils.convert_compas_pts_list_list_to_Data(sequencePts_003)
        utils.save_json(sequencePts_003_data, DATA_PATH, "sequencePts_003_data.json")

# if modify_merging:































