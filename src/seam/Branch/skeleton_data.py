from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane

import seam.utils.utils as utils
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

class Skeleton:
    def __init__(self, skeleton_topology, all_vertices_data, start_and_node_vertices):
        self.topology = skeleton_topology
        self.all_vertices_data = all_vertices_data
        self.start_and_node = start_and_node_vertices
        self.node_vertex_key = self.start_and_node["node_vertices"]
        self.start_vertex_key = self.start_and_node["start_vertices"]

        ### setting ###
        self.branch_keys = self.create_branch_keys_list()

        self.all_vertices = []
        self.get_all_vertex()

        self.branch_vertex_dict = {}
        self.get_branch_vertex_indices()

        ### output ###
        self.vNormal_vectors = {}
        self.vBinorm_vectors = {}
        self.vTangent_vectors = {}
        self.vPlanes = {}
        self.vPlanes_const = {}
        self.vPlanes_data = {}
        ## execute ##
        self.get_vNormals_vBinorms_vPlanes()

        ## cull the planes on the node ##
        self.vPlanes_without_node, \
        self.vPlanes_const_without_node, \
        self.vPlanes_data_without_node = self.cull_node_planes()

    def get_all_vertex(self):
        vertices = utils.convert_data_pts_list_to_compas_pts(self.all_vertices_data)
        self.all_vertices = vertices

    def create_branch_keys_list(self):
        branch_keys = []
        for i in range(len(self.topology)):
            branch_keys.append(i)
        return branch_keys

    def flatten_topology_to_keys(self, list_list):
        indices = []
        for list in list_list:
            for id in list:
                if id not in indices:
                    indices.append(id)
        return indices

    def get_branch_vertex_indices(self):
        for branchID in self.branch_keys:
            edge_topos = self.topology[str(branchID)]
            indices = self.flatten_topology_to_keys(edge_topos)
            print(indices)
            self.branch_vertex_dict[branchID] = indices

    def get_vertices_from_branchID(self, branchID):
        branch = self.topology[str(branchID)]
        vIDs = self.flatten_topology_to_keys(branch)
        print("vIDS :", vIDs)
        vertices = []
        for vID in vIDs:
            vertex = self.all_vertices[vID]
            vertices.append(vertex)
        return vertices

    def get_vNormals_vBinorms_vPlanes(self):
        for branchID in self.branch_keys:
            indices = self.branch_vertex_dict[branchID]
            branch_vertices = []
            for index in indices:
                vertex = self.all_vertices[index]
                branch_vertices.append(vertex)
            Vector_Family = discrete_curve.PolyVertex_Vector_Family(branch_vertices)

            normals = Vector_Family.normals
            binorms = Vector_Family.binorms
            tangents = Vector_Family.tangets
            planes = Vector_Family.planes
            planes_const = Vector_Family.planes_const

            self.vNormal_vectors[branchID] = normals
            self.vBinorm_vectors[branchID] = binorms
            self.vTangent_vectors[branchID] = tangents
            self.vPlanes[branchID] = planes
            self.vPlanes_const[branchID] = planes_const

            planes_data = Vector_Family.planes_data
            self.vPlanes_data[branchID] = planes_data

    def cull_node_planes(self):
        vPlanes_without_node = {}
        vPlanes_const_without_node = {}
        vPlanes_data_without_node = {}
        for branch_key in self.branch_keys:
            branch_vPlanes = self.vPlanes[branch_key]
            branch_vPlanes_const = self.vPlanes_const[branch_key]
            branch_vPlanes_data = self.vPlanes_data[branch_key]
            vertex_keys = self.branch_vertex_dict[branch_key]
            ## set new list in the new dict ##
            vPlanes_without_node[branch_key] = []
            vPlanes_const_without_node[branch_key] = []
            vPlanes_data_without_node[branch_key] = []
            for i, key in enumerate(vertex_keys):
                if key in self.node_vertex_key:
                    pass
                else:
                    vPlane = branch_vPlanes[i]
                    vPlanes_without_node[branch_key].append(vPlane)
                    vPlane_const = branch_vPlanes_const[i]
                    vPlanes_const_without_node[branch_key].append(vPlane_const)
                    vPlane_data = branch_vPlanes_data[i]
                    vPlanes_data_without_node[branch_key].append(vPlane_data)
        return vPlanes_without_node, vPlanes_const_without_node, vPlanes_data_without_node






##############################################
## definition for data structure of skeleton ##
##############################################
def create_data_from_pts(pts_data_dict, log=False):
    vertices = pts_data_dict
    key_list = list(vertices.keys())

    Skeleton = {}
    for i in range(len(key_list)-1):
        keys_pair = [key_list[i], key_list[i+1]]
        v_start = vertices[keys_pair[0]]
        v_end = vertices[keys_pair[1]]
        # create vertex point with compas point #
        v_start = Point(x=v_start[0], y=v_start[1], z=v_start[2])
        v_end = Point(x=v_end[0], y=v_end[1], z=v_end[2])
        if i == 0:
            origin_vertex = v_start
            Skeleton["origin"] = origin_vertex
        # EDGE TANGENT VECTOR is not an unit vector, has a length
        E_Tangent = v_end - v_start

        Skeleton[i] = { "edge_Tangent" : E_Tangent }
    if log:
        logger.info("Branch Num "+str(len(Skeleton)))
        logger.info("Vertices Num "+str(len(vertices)))
    return Skeleton


##############################################
## definition for vectors on every vertices ##
##############################################
def get_Normals_on_vertices(Skeleton, log=False):
    """
    input Skeleton is a dictionary
    """
    vNormals = {}
    indices = list(Skeleton.keys())
    for i, index in enumerate(indices):
        if type(index) != int:
            indices.pop(i)
    for m in range(len(indices)-1):
        index = indices[m]
        index_ = indices[m+1]
        edge_current = Skeleton[index]
        edge_next = Skeleton[index_]
        Edge_Tangent_current = edge_current["edge_Tangent"]
        Edge_Tangent_next = edge_next["edge_Tangent"]

        normal_vec = Edge_Tangent_current - Edge_Tangent_next
        normal_vec = normal_vec.unitized()
        if m == 0:
            start_edge = Skeleton[m]
            second_edge = Skeleton[m+1]
            start_Edge_Tangent = start_edge["edge_Tangent"]
            second_Edge_Tangent = second_edge["edge_Tangent"]
            start_binormal = start_Edge_Tangent.cross(second_Edge_Tangent)
            start_binormal = start_binormal.unitized()
            normal_vec_start = start_Edge_Tangent.cross(start_binormal)
            normal_vec_start = normal_vec_start.unitized()
            vNormals[m] = normal_vec_start
        vNormals[m+1] = normal_vec
        if m == len(indices)-2:
            end_branch = Skeleton[m+1]
            pre_end_branch = Skeleton[m]
            end_Edge_Tangent = end_branch["edge_Tangent"]
            pre_end_Edge_Tangent = pre_end_branch["edge_Tangent"]
            end_binormal = pre_end_Edge_Tangent.cross(end_Edge_Tangent)
            end_binormal = end_binormal.unitized()
            normal_vec_end = end_Edge_Tangent.cross(end_binormal)
            normal_vec_end = normal_vec_end.unitized()
            vNormals[m+2] = normal_vec_end
    if log:
        logger.info("vNormals Num :"+str(len(vNormals)))
    return vNormals


def get_Binormals_on_vertices(Skeleton, log=False):
    vBinormals = {}
    indices = list(Skeleton.keys())
    for i, index in enumerate(indices):
        if type(index) != int:
            indices.pop(i)
    for j in range(len(indices)-1):
        skeleton = Skeleton[j]
        skeleton_ = Skeleton[j+1]
        tangent_vec = skeleton["edge_Tangent"]
        tangent_vec_next = skeleton_["edge_Tangent"]

        binormal_vec = tangent_vec.cross(tangent_vec_next)
        binormal_vec.unitized()
        if j == 0:

            vBinormals[j] = binormal_vec

        vBinormals[j+1] = binormal_vec

        if j == len(indices)-2:
            vBinormals[j+2] = binormal_vec
    if log:
        logger.info("vBinormals Num :"+str(len(vBinormals)))
    return vBinormals


def get_vectors_on_vertices(Skeleton):
    vNormals = get_Normals_on_vertices(Skeleton, log=False)
    vBinormals = get_Binormals_on_vertices(Skeleton, log=False)
    vTangents = {}
    vIndices, eIndices = get_indices_of_vertices_and_edges(Skeleton)
    ## angle checker ##
    for i, vIndex in enumerate(vIndices):
        if i != 0:
            vNormal = vNormals[vIndex]
            vNorm_pre = vNormals[vIndex-1]
            angle_n = vNormal.angle(vNorm_pre)
            if angle_n > 0.5 * math.pi:
                vNormals[vIndex] = vNormal * (-1)

            vBinorm = vBinormals[vIndex]
            vBinorm_pre = vBinormals[vIndex-1]
            angle_b = vBinorm.angle(vBinorm_pre)
            if angle_b > 0.5 * math.pi:
                vBinormals[vIndex] = vBinorm * (-1)
    for vIndex in vIndices:
        normal = vNormals[vIndex]
        binorm = vBinormals[vIndex]

        vTangent = binorm.cross(normal)
        vTangent = vTangent.unitized()
        vTangents[vIndex] = vTangent
    logger.info("vector Num :"+str(len(vBinormals)))
    return vTangents, vNormals, vBinormals


##############################################
## definition for every vertices ##
##############################################
def get_vertices_from_Skeleton(Skeleton):
    vTangents, vNormals, vBinormals = get_vectors_on_vertices(Skeleton)
    #######################################
    # get vertex indices and edge indices #
    vIndices = vTangents.keys()
    eIndices = list(Skeleton.keys())
    for i, index in enumerate(eIndices):
        if type(index) != int:
            eIndices.pop(i)
    #######################################
    # set vertices from branch data #
    branch_origin = Skeleton["origin"]
    vertices = []
    vertices.append(branch_origin)
    vertex = branch_origin
    for eIndex in eIndices:
        skeleton = Skeleton[eIndex]
        Tangent_vec = skeleton["edge_Tangent"]
        vertex = vertex + Tangent_vec
        vertices.append(vertex)
    return vertices


############################################
## definition for indices ##
############################################
def get_separated_edge_indices_list(Skeleton):
    eIndices = list(Skeleton.keys())
    for i, index in enumerate(eIndices):
        if type(index) != int:
            eIndices.pop(i)

    branch = Skeleton[0]
    origin_Tangent = branch["edge_Tangent"]
    skeleton_indices = []
    sub_indices = []
    pieces_Num = 1
    for i in eIndices:
        branch = Skeleton[i]
        eTangent = branch["edge_Tangent"]
        angle = eTangent.angle(origin_Tangent)
        if angle > math.pi / 2:
            pieces_Num += 1
            skeleton_indices.append(sub_indices)
            sub_indices = []
            origin_Tangent = eTangent
        sub_indices.append(i)
    skeleton_indices.append(sub_indices)
    if pieces_Num == 1:
        skeleton_indices = sub_indices

    logger.info("pieces_Num :"+str(pieces_Num))
    return skeleton_indices


def get_indices_of_vertices_and_edges(Skeleton):
    """
    get vIndices and eIndices from skeleton data
    """
    # vTangents, vNormals, vBinormals = get_vectors_on_vertices(Skeleton)
    vNormals = get_Normals_on_vertices(Skeleton, log=False)
    print(type(vNormals))
    # get vertex indices and edge indices #
    indices = list(vNormals.keys())
    for i, index in enumerate(indices):
        if type(index) != int:
            indices.pop(i)
    vIndices = indices
    eIndices = list(Skeleton.keys())
    for i, index in enumerate(eIndices):
        if type(index) != int:
            eIndices.pop(i)
    logger.info("vNum :"+str(len(vIndices))+" eNum :"+str(len(eIndices)))
    return vIndices, eIndices


############################################
## definition for planes on each vertices ##
############################################
def get_planes_on_each_vertex(Skeleton):
    vTangents, vNormals, vBinormals = get_vectors_on_vertices(Skeleton)
    vIndices, eIndices = get_indices_of_vertices_and_edges(Skeleton)
    vertices = get_vertices_from_Skeleton(Skeleton)
    planes = []
    for vIndex in vIndices:
        vtangent = vTangents[vIndex]
        vnormal = vNormals[vIndex]
        vbinorm = vBinormals[vIndex]
        vertex = vertices[vIndex]

        plane_vertex = Plane.from_point_and_two_vectors(vertex, vbinorm, vnormal)
        planes.append(plane_vertex)
    return planes

def get_planes_data_point_and_two_vectors(Skeleton):
    vTangents, vNormals, vBinormals = get_vectors_on_vertices(Skeleton)
    vIndices, eIndices = get_indices_of_vertices_and_edges(Skeleton)
    vertices = get_vertices_from_Skeleton(Skeleton)
    planes_data = []
    for vIndex in vIndices:
        vtangent = vTangents[vIndex]
        vnormal = vNormals[vIndex]
        vbinorm = vBinormals[vIndex]
        vertex = vertices[vIndex]

        vertex_data = utils.convert_compas_Point_to_Data(vertex)
        vnormal_data = utils.convert_compas_vector_to_Data(vnormal)
        vbinorm_data = utils.convert_compas_vector_to_Data(vbinorm)

        plane_data = [vertex_data, vbinorm_data, vnormal_data]
        planes_data.append(plane_data)
    return planes_data






















