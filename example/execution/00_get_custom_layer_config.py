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
       "3.1_final_demo/10_net_branch/"


###################################################
node_skeleton = False
layer_compute = True
offset = False
###################################################
"""

"""
##################################################################################################################################
## first step ####################################################################################################################
#####################################
## skeleton setting for hole shape ##
OBJ_NODE_00 = "node.obj"

OBJ_NODE = OBJ_NODE_00
NODE_FOLDER = "node_present/"
NODE_DATA_PATH = PATH + NODE_FOLDER
######################################
if node_skeleton:
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
OBJ_MESH_NAME = "MESH.obj"

OBJ_MESH = OBJ_MESH_NAME
FOLDER = "data_00/"
DATA_PATH = PATH + NODE_FOLDER + FOLDER
#######################################
if layer_compute:
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
    ########################
    # vkeys = list(MESH.vertices())
    # vs, fs = MESH.to_vertices_and_faces()
    # vertices = [Point(v_coords[0], v_coords[1], v_coords[2]) for v_coords in vs]
    # ###############################
    # ## set boundaries attributes ##
    # ###############################
    # MESH.update_default_vertex_attributes({"seam": -1})
    # for vkey, data in MESH.vertices(data=True):
    #     for i, seam_ver_keys in enumerate(seam_vertex_keys_list):
    #         if vkey in seam_ver_keys:
    #             data["seam"] = i
    #####################################
    ## momal_geodesics from every seam ##
    #####################################
    dis_list = distance_calculation.get_distances_list_from_every_boundary(MESH, boundary_vertex_keys_list)
    logger.info("number of seam :" + str(len(dis_list)))
    for i, dis in enumerate(dis_list):
        utils.save_json(dis, DATA_PATH, "d_0"+str(i)+".json")

    ############################
    ## check branching or not ##
    node_branching = False
    if boundary_num == 2:
        node_branching = False
    elif boundary_num == 3:
        node_branching = True
    ########################
    if node_branching:
        layer_generation = path_generation.Layer_generation(MESH, boundary_vertex_keys_list)
        layer_num = layer_generation.layer_num
        ## compute layer path with distance attributes of three boundary curves ##
        pathPts_000 = layer_generation.\
            create_layers_path_from_distance_attributes_three_boundaries(base_boundary_num=0,
                                                                         minValue=0.75, frequency=1.8, longWayExtention=False,
                                                                         matchPathDirection=False,
                                                                         modifyStartingPathPt=False)
        """
        ## base_boundary_num=0,minValue=0.75, frequency=1.8, 
        longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
        """
        pathPts_001 = layer_generation.\
            create_layers_path_from_distance_attributes_three_boundaries(base_boundary_num=1,
                                                                         minValue=0.85, frequency=-1.0, longWayExtention=True,
                                                                         matchPathDirection=False,
                                                                         modifyStartingPathPt=False)
        """
        ## base_boundary_num=1, minValue=0.85, frequency=1.0, 
        longWayExtention=True, matchPathDirection=True, modifyStartingPathPt=True)
        """
        pathPts_002 = layer_generation.\
            create_layers_path_from_distance_attributes_three_boundaries(base_boundary_num=2,
                                                                         minValue=0.75, frequency=1.8, longWayExtention=False,
                                                                         matchPathDirection=False,
                                                                         modifyStartingPathPt=False)
        """
        ## base_boundary_num=2,minValue=0.75, frequency=1.8, 
        longWayExtention=False, matchPathDirection=True, modifyStartingPathPt=True)
        """
        pathPts_000_data = utils.convert_compas_pts_list_list_to_Data(pathPts_000)
        utils.save_json(pathPts_000_data, DATA_PATH, "pathPts_000_data.json")
        pathPts_001_data = utils.convert_compas_pts_list_list_to_Data((pathPts_001))
        utils.save_json(pathPts_001_data, DATA_PATH, "pathPts_001_data.json")
        pathPts_002_data = utils.convert_compas_pts_list_list_to_Data((pathPts_002))
        utils.save_json(pathPts_002_data, DATA_PATH, "pathPts_002_data.json")

    else:
        # layer_generation = path_generation.Layer_generation(MESH, boundary_vertex_keys_list)
        # layer_num = layer_generation.layer_num
        # ## compute layer path with distance attributes of two boundary curves ##
        # pathPts_00 = layer_generation.\
        #     create_layers_path_from_distance_attributes_two_boundaries(base_boundary_num=0,
        #                                                                  minValue=0.8, frequency=2.7, longWayExtention=False,
        #                                                                  matchPathDirection=False,
        #                                                                  modifyStartingPathPt=False)
        # # pathPts_01 = layer_generation.\
        # #     create_layers_path_from_distance_attributes_two_boundaries(base_boundary_num=1,
        # #                                                                  minValue=0.6, frequency=1.0, longWayExtention=False,
        # #                                                                  matchPathDirection=True,
        # #                                                                  modifyStartingPathPt=True)
        # pathPts_00_data = utils.convert_compas_pts_list_list_to_Data(pathPts_00)
        # utils.save_json(pathPts_00_data, DATA_PATH, "pathPts_00_data.json")
        # # pathPts_01_data = utils.convert_compas_pts_list_list_to_Data((pathPts_01))
        # # utils.save_json(pathPts_01_data, DATA_PATH, "pathPts_01_data.json")
        Layer_generation = path_generation.Layer_generation(MESH, boundary_vertex_keys_list)
        layer_num = Layer_generation.layer_num
        half_layer_num = int(round((layer_num)/2, 0))
        ## compute layer paths with various type of differences ##
        # pathPts_list = Layer_generation. \
        #     create_layers_path_from_difs_two_seams(diftype=0,
        #                                            directionMatching=True,
        #                                            modifyStartingPathPt=True)
        ## custom layer path configuration ##
        pathPts_list_00 = Layer_generation. \
            create_layers_path_from_difs_two_boundaries(base_boundary_num=0,
                                                        minValue=0.5, frequency=1.5,
                                                        matchPathDirection=False,
                                                        modifyStartingPathPt=False)
        pathPts_list_01 = Layer_generation. \
            create_layers_path_from_difs_two_boundaries(base_boundary_num=1,
                                                        minValue=0.5, frequency=1.5,
                                                        matchPathDirection=False,
                                                        modifyStartingPathPt=False)
        ## out layer paths ##
        # basic_pathPts = base_pathPts_list
        ## convert and save it as json file ##
        pathPts_00_data = utils.convert_compas_pts_list_list_to_Data(pathPts_list_00)
        utils.save_json(pathPts_00_data, DATA_PATH, "pathPts_00_data.json")
        pathPts_01_data = utils.convert_compas_pts_list_list_to_Data(pathPts_list_01)
        utils.save_json(pathPts_01_data, DATA_PATH, "pathPts_01_data.json")



## third step #####################################################################################################################
##########################################
## offset layer pahts on the node piece ##
FOLDER = "data_00/"
DATA_PATH = PATH + NODE_FOLDER + FOLDER
if offset:
    ## load ##
    ## piece_00 ##
    node_paths_00_data = utils.load_from_Json(DATA_PATH, "_node_paths_00_data.json")
    node_paths_00 = utils.convert_data_pts_list_list_to_compas_pts(node_paths_00_data)
    ## piece_01 ##
    node_paths_01_data = utils.load_from_Json(DATA_PATH, "_node_paths_01_data.json")
    node_paths_01 = utils.convert_data_pts_list_list_to_compas_pts(node_paths_01_data)
    ## piece_02 ##
    node_paths_02_data = utils.load_from_Json(DATA_PATH, "_node_paths_02_data.json")
    node_paths_02 = utils.convert_data_pts_list_list_to_compas_pts(node_paths_02_data)

    ## offset ##
    ## first offset ##
    other_pieces = [node_paths_01]
    first_offset = path_generation.OffsetTool(node_paths_00, other_pieces)
    node_paths_00 = first_offset.offset_layer_pathPts()
    ## second offset ##
    other_pieces = [node_paths_02]
    second_offset = path_generation.OffsetTool(node_paths_01, other_pieces)
    node_paths_01 = second_offset.offset_layer_pathPts()
    ## third offset ##
    # other_pieces = [node_paths_02]
    # Offset01_ = path_generation.OffsetTool(node_paths_01, other_pieces)
    # node_paths_01 = Offset01_.offset_layer_pathPts()

    ###############################
    ## convert and send the data ##
    node_paths_00_data = utils.convert_compas_pts_list_list_to_Data(node_paths_00)
    node_paths_01_data = utils.convert_compas_pts_list_list_to_Data(node_paths_01)
    node_paths_02_data = utils.convert_compas_pts_list_list_to_Data(node_paths_02)
    utils.save_json(node_paths_00_data, DATA_PATH, "node_paths_00_data.json")
    utils.save_json(node_paths_01_data, DATA_PATH, "node_paths_01_data.json")
    utils.save_json(node_paths_02_data, DATA_PATH, "node_paths_02_data.json")





