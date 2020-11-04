import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane

from seam.utils import utils, primitive
from seam.boundary import seam_crv, distance_calculation
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

def get_boundary_pts_list_from_data(seams_pts_Data):
    seam_pts_list = []
    for seam_pts in seams_pts_Data:
        s_pts = utils.convert_data_pts_list_to_compas_pts(seam_pts, ROUND=False)
        seam_pts_list.append(s_pts)
    return seam_pts_list


def get_boundary_vertex_indices(mesh, seam_pts):
    """
    useful for calculate geodesic distance between boundaries
    """
    v, faces = mesh.to_vertices_and_faces()
    vertices = [Point(v_coods[0], v_coods[1], v_coods[2]) for v_coods in v]

    seam_ids = []
    for bPt in seam_pts:
        closestPt, minimun, index = primitive.get_closest_point_from_pts_list(bPt, vertices)
        if index not in seam_ids:
            seam_ids.append(index)
    return seam_ids


def get_boundary_vertex_keys_list_from_seam_pts_list(mesh, seams_pts):
    seam_vertex_keys_list = []
    for seam_pts in seams_pts:
        seam_ids = get_boundary_vertex_indices(mesh, seam_pts)
        seam_vertex_keys_list.append(seam_ids)
    return seam_vertex_keys_list


class Boundary:
    """
    get "boundary_pts_list" and "boundary_vertex_keys_list"
    """
    def __init__(self, MESH, boundary_pts_data_list):
        self.boundary_pts_data_list = boundary_pts_data_list
        self.MESH = MESH

        self.boundary_pts_list = []
        self.boundary_vertex_keys_list = []

    def get_boundaries(self):
        self.get_boundary_pts_list_from_data()
        self.get_boundary_vertex_keys_list()
        logger.info("get boundary_vertex_keys_list")

    def get_boundary_vertex_keys_list(self):
        v, f = self.MESH.to_vertices_and_faces()
        vertices = [Point(v_coords[0], v_coords[1], v_coords[2]) for v_coords in v]
        for seam_pts in self.boundary_pts_list:
            seam_vkeys = []
            for seam_pt in seam_pts:
                closestPt, minimum, key = primitive.get_closest_point_from_pts_list(seam_pt, vertices)
                if key not in seam_vkeys:
                    seam_vkeys.append(key)
            self.boundary_vertex_keys_list.append(seam_vkeys)

    def get_boundary_pts_list_from_data(self):
        for seam_pts_data in self.boundary_pts_data_list:
            seam_pts = utils.convert_data_pts_list_to_compas_pts(seam_pts_data)
            self.boundary_pts_list.append(seam_pts)


##################################
## get the branching node seam ##
##################################
def get_proper_boundary_centrePt_on_branching_node(mesh, boundary_distances, max_radius=35, even_distance=False):
    cross_section_length = max_radius * 2 * math.pi
    max_distance = max(boundary_distances)
    time_step = 1 / 99
    skeletonPts = []
    for i in range(100):
        found = False
        time = time_step * i
        difs = []
        for distance in boundary_distances:
            dif = max_distance * time - distance
            difs.append(dif)
        curve_pts, centrePt = seam_crv.get_layer_crvPts_from_distance_differences_on_Mesh(mesh, difs)
        skeletonPts.append(centrePt)
        length = seam_crv.layer_curve_length(curve_pts)
        if i == 0:
            original_l = length
        else:
            if even_distance:
                if length > cross_section_length:
                    found = True
            elif not even_distance:
                if length > original_l * 1.5 or length > cross_section_length:
                    found = True
        if found:
            seam_time = time - time_step
            ## create the ccentre pt of this cross crv ##
            boundary_centrePt = skeletonPts[-1]
            break
    skeletonPts_out = []
    for i, sPt in enumerate(skeletonPts):
        if sPt != boundary_centrePt:
            if i % 20 == 0: skeletonPts_out.append(sPt)
    return boundary_centrePt, skeletonPts_out

def get_boundary_centrePts_list_on_branching_node(mesh, distances_list, max_radius, even_distance=False):
    centrePt_list = []
    skeletonPts_list = []
    for seam_distances in distances_list:
        seam_centrePt, skeletonPts = get_proper_boundary_centrePt_on_branching_node(mesh, seam_distances, max_radius, even_distance)
        centrePt_list.append(seam_centrePt)
        skeletonPts_list.append(skeletonPts)
    return centrePt_list, skeletonPts_list

















