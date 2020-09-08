import math

from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Plane
from compas.datastructures import  Mesh

from seam.utils import utils, primitive, parameters
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

######################################
## calculating distance differences ##
class Differences:
    def __init__(self, MESH, seam_vertex_keys_list):
        self.mesh = MESH
        self.svkl = seam_vertex_keys_list
        self.seam_num = len(self.svkl)

        # self.width_01 = width_01

        ## settings ##
        self.distances_list = self.get_distances_list_from_every_boundary()
        self.short_way, self.long_way, self.ave_way, self.way_list = self.get_short_long_ave_way_length_on_every_vertex()
        self.gap_ratio = self.get_gap_ratio()

        # ## results ##
        # self.realm_dict = self.get_seam_realm_dict_from_vertex_geodesic()

    #######################################################
    ## functions ##
    def compute_geodesic_to_every_vertex(self, mesh, vertices_start):
        v, f = mesh.to_vertices_and_faces()
        v = np.array(v)
        f = np.array(f)
        vertices_target = np.arange(len(v))  # all vertices are targets
        vstart = np.array(vertices_start)
        distances = igl.exact_geodesic(v, f, vstart, vertices_target)
        return distances

    def compute_geodesic_from_start_to_target_vkeys(self, mesh, start_v_keys_list, target_v_keys_list):
        v, f = mesh.to_vertices_and_faces()
        v = np.array(v)
        f = np.array(f)
        vertices_start = np.array(start_v_keys_list)
        vertices_target = np.array(target_v_keys_list)
        distances = igl.exact_geodesic(v, f, vertices_start, vertices_target)
        return distances

    ## get values ##

    def get_distances_list_from_every_boundary(self):
        distances_list = []
        for seam_vkeys in self.svkl:
            distances = list(compute_geodesic_to_every_vertex(self.mesh, seam_vkeys))
            distances_list.append(distances)
        return distances_list

    ## calculating ##

    def get_short_long_ave_way_length_on_every_vertex(self):
        # distances_list = get_distances_list_from_every_seams(mesh, seam_vertex_keys_list)
        vkeys = self.mesh.vertices()
        way_list = []
        for vkey in vkeys:
            vdis_list = [distances[vkey] for distances in self.distances_list]
            vdis_list.sort()
            d00 = vdis_list[0]
            d01 = vdis_list[1]
            way = d00 + d01
            way_list.append(way)
        temp_list = [way for way in way_list]
        temp_list.sort()
        short_way = temp_list[0]
        long_way = temp_list[-1]
        ave_way = sum(temp_list) / len(temp_list)
        return short_way, long_way, ave_way, way_list

    def get_gap_ratio(self):
        gap_ratio = (self.long_way - self.short_way) / self.long_way
        return gap_ratio

    # #######################################################
    # def get_seam_realm_dict_from_vertex_geodesic(self):
    #     realm_dict = {}
    #     for i in range(3):
    #         realm_dict["piece_0" + str(i)] = []
    #     vkeys = self.mesh.vertices()
    #     for vkey in vkeys:
    #         dist_list = [distances[vkey] for distances in self.distances_list]
    #         closest = min(dist_list)
    #         seam_number = dist_list.index(closest)
    #         realm_dict["piece_0" + str(seam_number)].append(vkey)
    #     return realm_dict
    #
    # def equal_differences(self, time):
    #     # if len(self.distances_list) <= 2:
    #     difs = [(d01 * time - d00 * (1 - time)) for d00, d01 in zip(self.distances_list[0], self.distances_list[1])]
    #     return difs

    ## for connection detail ##
    def calculate_custom_differences_with_two_boundaries(self, base_boundary_num, time,
                                                         minValue=0.5,
                                                         frequency=2.5):
        difs = []
        distances_00 = self.distances_list[0]
        distances_01 = self.distances_list[1]
        for D_00, D_01, way in zip(distances_00, distances_01, self.way_list):
            x = abs(self.long_way - way) / (self.long_way - self.short_way)
            width = self.long_way * minValue
            freq = frequency
            alpha = freq * math.pi * x
            a = minValue
            y = ((1 - a) / 2) * math.cos(alpha) + ((a + 1) / 2)
            if y > 1:
                y = 1
            elif y < a:
                y = a

            if base_boundary_num == 0:
                dif = (D_01) * time - (D_00 * y) * (1 - time)
            else:
                dif = (D_00) * time - (D_01 * y) * (1 - time)
            # value = width * math.cos(alpha) + width / 2
            # if value > width:
            #     value = width
            # elif value < 0:
            #     value = 0
            # addv = value * (0.5 - abs(0.5 - time))
            # ## get the difference value ##
            # if base_boundary_num == 0:
            #     dif = ((D_01 + addv) * time - (D_00) * (1 - time))
            # else:
            #     dif = ((D_00 + addv) * time - (D_01) * (1 - time))
            difs.append(dif)
        return difs

        # for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
        #     way = d00 + d01
        #     x = abs(self.long_way - way)
        #     y = abs(self.ave_way - way)
        #     check_value = abs(self.long_way - self.ave_way)
        #     value = width * math.cos((2.7 * math.pi / (self.long_way - self.short_way)) * x) + width / 2
        #     # value = value * (1 - 1 / (y + 1))
        #     if value >= width:
        #         value = width
        #     elif value < 0: value = 0
        #     else:
        #         value = value
        #     addv = value * (0.5 - abs(0.5 - time))
        #     dif00 = ((d01 + addv) * time - d00 * (1 - time))
        #     difs00.append(dif00)
        # return difs00
    # def custom_differences_01_second(self, time, width=70):
    #     """
    #     diftype == 1.2
    #     """
    #     # width = self.width_01
    #     difs01 = []
    #     time_ = time
    #     for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
    #         way = d00 + d01
    #         x = abs(self.long_way - way)
    #         y = abs(self.ave_way - way)
    #         check_value = abs(self.long_way - self.ave_way)
    #         value = 10 * width * math.cos((2 * math.pi / (self.long_way - self.short_way)) * x) - 1.5 * width
    #         #    value = value * (1 - 1/(y+1))
    #         if value <= 0:
    #             value = 0
    #         elif value > width:
    #             value = width
    #         else:
    #             value = value
    #         addv = value * (0.5 - abs(0.5 - time))
    #         dif01 = (d01 - addv) * time - (d00) * (1 - time)
    #         difs01.append(dif01)
    #     return difs01
    #
    # ## for connection detail 2.0 ##
    # def custom_differences_03_first(self, time, width=210, flat_ratio=0.2):
    #     """
    #     diftype == 3.1
    #     """
    #     difs00 = []
    #     for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
    #         way = d00 + d01
    #         check_value = abs(self.ave_way - way)
    #         l_value = abs(self.long_way - way)
    #         s_value = abs(self.short_way - way)
    #         wid = width
    #         if self.ave_way - way >= 0:
    #             ## define the short side ##
    #             value = wid * ((1/(s_value+1))**0.4) - wid * ((1/abs(self.short_way-self.ave_way))**0.4) * (1 + flat_ratio)
    #         else:
    #             ## define the long side ##
    #             value = wid * ((1/(l_value+1))**0.2) - wid * ((1/abs(self.long_way-self.ave_way))**0.2) * (1 + flat_ratio)
    #         if value < 0: value = 0
    #         addv = value * (0.5 - abs(0.5-time))
    #
    #         dif = (d01 + addv/2)*time - (d00)*(1-time)
    #         difs00.append(dif)
    #     return difs00
    #
    # def custom_differences_03_second(self, time, width=210, flat_ratio=0.2):
    #     """
    #     diftype == 3.2
    #     """
    #     difs01 = []
    #     for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
    #         way = d00 + d01
    #         check_value = abs(self.ave_way - way)
    #         l_value = abs(self.long_way - way)
    #         s_value = abs(self.short_way - way)
    #         wid = width
    #         if self.ave_way - way >= 0:
    #             ## define the short side ##
    #             value = wid * ((1/(s_value+1))**0.3) - wid * ((1/abs(self.short_way -self.ave_way))**0.3) * (1 + flat_ratio)
    #         else:
    #             ## define the long side ##
    #             value = wid * ((1/(l_value+1))**0.15) - wid * ((1/abs(self.long_way - self.ave_way))**0.15) * (1 + flat_ratio)
    #         if value < 0: value = 0
    #         addv = value * (0.5 - abs(0.5-time))
    #
    #         dif = (d01 - addv/2)*time - (d00)*(1-time)
    #         difs01.append(dif)
    #     return difs01
    #
    # ## for distance gap ##
    # def custom_differences_02_first(self, time):
    #     """
    #     diftype == 2.1
    #     """
    #     width = (self.short_way * self.gap_ratio)
    #     ## get the differences from start seam and from last seam ##
    #     difs00 = []
    #     for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
    #         way = d00 + d01
    #         x = abs(self.ave_way - way)
    #         check_value = abs(self.ave_way - self.long_way)
    #         wid = width
    #         if (self.ave_way - way) >= 0:
    #             ## make the value smoother ##
    #             value = wid * ((x / check_value) ** 1.5) * (1 - 1 / (x + 1))
    #         else:
    #             value = 0
    #         addv = value * (0.5 - abs(0.5 - time))
    #         dif00 = ((d01 - addv) * time - d00 * (1 - time))
    #         difs00.append(dif00)
    #     return difs00
    #
    # def custom_differences_02_second(self, time):
    #     """
    #     diftype == 2.2
    #     """
    #     width = (self.short_way * self.gap_ratio)
    #     difs01 = []
    #     time_ = time
    #     for d00, d01 in zip(self.distances_list[0], self.distances_list[1]):
    #         way = d00 + d01
    #         x = abs(self.ave_way - way)
    #         check_value = abs(self.ave_way - self.long_way)
    #         wid = width
    #         if (self.ave_way - way) >= 0:
    #             ## make the value smoother ##
    #             value = wid * ((x / check_value) ** 1.5) * (1 - 1 / (x + 1))
    #         else:
    #             value = 0
    #         addv = value * (0.5 - abs(0.5 - time_))
    #         dif01 = ((d01 + addv) * time_ - d00 * (1 - time_))
    #         difs01.append(dif01)
    #     return difs01
######################################
## calculating distance attribution ##
class Distance_Attributes_two:
    def __init__(self, MESH, boundary_vertex_keys_list):
        self.mesh = MESH
        self.bvkl = boundary_vertex_keys_list
        self.boundary_num = len(self.bvkl)
        ## settings ##
        self.distances_list = self.get_distances_list_from_every_boundary()
        self.short_way, self.long_way, self.ave_way, self.way_list \
            = self.get_short_long_ave_way_length_on_every_vertex()
        self.gap_ratio = self.get_gap_ratio()

    def get_distances_list_from_every_boundary(self):
        distances_list = []
        for boundary_vkeys in self.bvkl:
            distances = list(compute_geodesic_to_every_vertex(self.mesh, boundary_vkeys))
            distances_list.append(distances)
        return distances_list

    ## calculating ##
    def get_short_long_ave_way_length_on_every_vertex(self):
        # distances_list = get_distances_list_from_every_seams(mesh, seam_vertex_keys_list)
        vkeys = self.mesh.vertices()
        way_list = []
        for vkey in vkeys:
            vdis_list = [distances[vkey] for distances in self.distances_list]
            vdis_list.sort()
            d00 = vdis_list[0]
            d01 = vdis_list[1]
            way = d00 + d01
            way_list.append(way)
        temp_list = [way for way in way_list]
        temp_list.sort()
        short_way = temp_list[0]
        long_way = temp_list[-1]
        ave_way = sum(temp_list) / len(temp_list)
        return short_way, long_way, ave_way, way_list

    def get_gap_ratio(self):
        gap_ratio = (self.long_way - self.short_way) / self.long_way
        return gap_ratio

    def get_attributes_from_one_base_boundary(self, base_boundary_num, time,
                                           minValue=0.5, frequency=1.0, longWayExtention=False):
        attrs = []
        difs = []
        distances_00 = self.distances_list[0]
        distances_01 = self.distances_list[1]
        for D_00, D_01, way in zip(distances_00, distances_01, self.way_list):

            x = (way / self.long_way)
            compare = self.short_way / self.long_way
            freq = frequency ## from 1 to 3 ##
            alpha = freq * math.pi * (x - compare) / (1 - compare)
            a = minValue
            ## fomura for the distortion ##
            if not longWayExtention:
                ## short way extention ##
                y = (((1 - a) / 2) + ((a + 1) / 4)) * math.sin(alpha) + ((a + 1) / 2) + ((a + 1) / 4)
                if y > 1: y = 1
            else:
                ## long way extention ##
                y = (((1 - a) / 2) + ((a + 1) / 4)) * math.cos(alpha) + ((a + 1) / 2) + ((a + 1) / 4)
                if y > 1: y = 1

            ## distribute the boundary area depending on the distance calculation with distortion fomura ##
            if base_boundary_num == 0:
                d_00 = D_00 * (1-time) * y
                d_01 = D_01 * (time)
                ## set base distance ##
                base_distance = d_00
                target_distance = d_01
            elif base_boundary_num == 1:
                d_00 = D_00 * (time)
                d_01 = D_01 * (1 - time) * y
                ## set base distance ##
                base_distance = d_01
                target_distance = d_00
            else:
                print("error with setting the base_boundary_num")
                break
            ## select the short value ##
            short_distance = min([d_00, d_01])
            if short_distance == d_00:
                attr = 0
            else:
                attr = 1
            attrs.append(attr)

            ## calculate differences ##
            dif = base_distance - target_distance
            difs.append(dif)

        return attrs, difs

class Distance_Attributes_three:
    def __init__(self, MESH, boundary_vertex_keys_list):
        self.mesh = MESH
        self.bvkl = boundary_vertex_keys_list
        self.boundary_num = len(self.bvkl)
        ## settings ##
        self.distances_list = self.get_distances_list_from_every_boundary()
        self.short_way, self.long_way, self.ave_way, self.way_list \
            = self.get_short_long_ave_way_length_on_every_vertex()
        self.gap_ratio = self.get_gap_ratio()

    def get_distances_list_from_every_boundary(self):
        distances_list = []
        for boundary_vkeys in self.bvkl:
            distances = list(compute_geodesic_to_every_vertex(self.mesh, boundary_vkeys))
            distances_list.append(distances)
        return distances_list

    ## calculating ##
    def get_short_long_ave_way_length_on_every_vertex(self):
        # distances_list = get_distances_list_from_every_seams(mesh, seam_vertex_keys_list)
        vkeys = self.mesh.vertices()
        way_list = []
        for vkey in vkeys:
            vdis_list = [distances[vkey] for distances in self.distances_list]
            vdis_list.sort()
            d00 = vdis_list[0]
            d01 = vdis_list[1]
            way = d00 + d01
            way_list.append(way)
        temp_list = [way for way in way_list]
        temp_list.sort()
        short_way = temp_list[0]
        long_way = temp_list[-1]
        ave_way = sum(temp_list) / len(temp_list)
        return short_way, long_way, ave_way, way_list

    def get_gap_ratio(self):
        gap_ratio = (self.long_way - self.short_way) / self.long_way
        return gap_ratio

    def get_attributes_from_one_base_boundary(self, base_boundary_num, time,
                                              minValue=0.75, frequency=2.0, longWayExtention=False):
        attrs = []
        difs = []
        distances_00 = self.distances_list[0]
        distances_01 = self.distances_list[1]
        distances_02 = self.distances_list[2]
        for D_00, D_01, D_02, way in zip(distances_00, distances_01, distances_02, self.way_list):

            # x = way / self.long_way
            # compare = self.short_way / self.long_way
            x = (self.long_way - way) / (self.long_way - self.short_way)
            ## x is changing from 0 with long_way to 1 with short_way ##
            freq = frequency
            alpha = freq * math.pi * x
            a = minValue
            ## fomura for the distortion ##
            if longWayExtention:
                ## short way extention ##
                y = ((1 - a) / 2) * math.sin(alpha) + ((a + 1) / 2)
                if y > 1:
                    y = 1
                elif y < a:
                    y = a
            else:
                ## long way extention ##
                y = ((1 - a) / 2) * math.cos(alpha) + ((a + 1) / 2)
                if y > 1:
                    y = 1
                elif y < a:
                    y = a
            ## distribute the boundary area depending on the distance calculation with distortion fomura ##
            if base_boundary_num == 0:
                d_00 = D_00 * (1-time) * y
                d_01 = D_01 * (time)
                d_02 = D_02 * (time)
                ## set base distance ##
                base_dist = d_00
                target_dists = [d_01, d_02]
            elif base_boundary_num == 1:
                d_00 = D_00 * (time)
                d_01 = D_01 * (1-time) * y
                d_02 = D_02 * (time)
                ## set base distance ##
                base_dist = d_01
                target_dists = [d_00, d_02]
            elif base_boundary_num == 2:
                d_00 = D_00 * (time)
                d_01 = D_01 * (time)
                d_02 = D_02 * (1-time) * y
                ## set base distance ##
                base_dist = d_02
                target_dists = [d_00, d_01]
            else:
                print("error with setting the base_boundary_num")
                break
            ## select short value ##
            short_distance = min([d_00, d_01, d_02])
            if short_distance == d_00:
                attr = 0
            elif short_distance == d_01:
                attr = 1
            else:
                attr = 2
            attrs.append(attr)

            ## calculate differences ##
            dif = base_dist - min(target_dists)
            difs.append(dif)

        return attrs, difs

class Distance_Attributes_four:
    def __init__(self, MESH, boundary_vertex_keys_list):
        self.mesh = MESH
        self.bvkl = boundary_vertex_keys_list
        self.boundary_num = len(self.bvkl)
        ## settings ##
        self.distances_list = self.get_distances_list_from_every_boundary()
        self.short_way, self.long_way, self.ave_way, self.way_list \
            = self.get_short_long_ave_way_length_on_every_vertex()
        self.gap_ratio = self.get_gap_ratio()

    def get_distances_list_from_every_boundary(self):
        distances_list = []
        for boundary_vkeys in self.bvkl:
            distances = list(compute_geodesic_to_every_vertex(self.mesh, boundary_vkeys))
            distances_list.append(distances)
        return distances_list

    ## calculating ##
    def get_short_long_ave_way_length_on_every_vertex(self):
        vkeys = self.mesh.vertices()
        way_list = []
        for vkey in vkeys:
            vdis_list = [distances[vkey] for distances in self.distances_list]
            vdis_list.sort()
            d00 = vdis_list[0]
            d01 = vdis_list[1]
            way = d00 + d01
            way_list.append(way)
        temp_list = [way for way in way_list]
        temp_list.sort()
        short_way = temp_list[0]
        long_way = temp_list[-1]
        ave_way = sum(temp_list) / len(temp_list)
        return short_way, long_way, ave_way, way_list

    def get_gap_ratio(self):
        gap_ratio = (self.long_way - self.short_way) / self.long_way
        return gap_ratio

    def get_attributs_from_one_base_boundary(self, base_boundary_num, time,
                                             minValue=0.75, frequency=2.0, longWayExtention=False):
        attrs = []
        difs = []
        distances_00 = self.distances_list[0]
        distances_01 = self.distances_list[1]
        distances_02 = self.distances_list[2]
        distances_03 = self.distances_list[3]
        for D_00, D_01, D_02, D_03, way in zip(distances_00,
                                               distances_01,
                                               distances_02,
                                               distances_03,
                                               self.way_list):
            x = (self.long_way - way) / (self.long_way - self.short_way)
            ## x is changing from 0 at long_way to 1 at short_way ##
            freq = frequency
            alpha = freq * math.pi * x
            a = minValue
            ## fomura for the distortion ##
            if longWayExtention:
                ## short way extention ##
                y = ((1 - a) / 2) * math.sin(alpha) + ((a + 1) / 2)
                if y > 1:
                    y = 1
                elif y < a:
                    y = a
            else:
                ## long way extention ##
                y = ((1 - a) / 2) * math.cos(alpha) + ((a + 1) / 2)
                if y > 1:
                    y = 1
                elif y < a:
                    y = a
            ## distribute the boundary area depending on the distance calculation with distortion fomura ##
            if base_boundary_num == 0:
                d_00 = D_00 * (1-time) * y
                d_01 = D_01 * (time)
                d_02 = D_02 * (time)
                d_03 = D_03 * (time)
                ## set base distance ##
                base_dist = d_00
                target_dists = [d_01, d_02, d_03]
            elif base_boundary_num == 1:
                d_00 = D_00 * (time)
                d_01 = D_01 * (1-time) * y
                d_02 = D_02 * (time)
                d_03 = D_03 * (time)
                ## set base distance ##
                base_dist = d_01
                target_dists = [d_00, d_02, d_03]
            elif base_boundary_num == 2:
                d_00 = D_00 * (time)
                d_01 = D_01 * (time)
                d_02 = D_02 * (1-time) * y
                d_03 = D_03 * (time)
                ## set base distance ##
                base_dist = d_02
                target_dists = [d_00, d_01, d_03]
            elif base_boundary_num == 3:
                d_00 = D_00 * (time)
                d_01 = D_01 * (time)
                d_02 = D_02 * (time)
                d_03 = D_03 * (1-time) * y
                ## set base distance ##
                base_dist = d_03
                target_dists = [d_00, d_01, d_02]
            else:
                print("error with setting the base_boundary_num")
                break
            ## select short value ##
            short_distance = min([d_00, d_01, d_02, d_03])
            if short_distance == d_00:
                attr = 0
            elif short_distance == d_01:
                attr = 1
            elif short_distance == d_02:
                attr = 2
            else:
                attr = 3
            attrs.append(attr)
            ## calculate differences ##
            dif = base_dist - min(target_dists)
            difs.append(dif)
        return attrs, difs


##################################################################################
## fandamental geodesics ##
##################################################################################
def compute_geodesic_to_every_vertex(mesh, vertices_start):
    """
    compute distances from sevelral vertices to all vertices of the mesh
    vertices are described with keys
    """
    v, f = mesh.to_vertices_and_faces()
    v = np.array(v)
    f = np.array(f)
    vertices_target = np.arange(len(v))  # all vertices are targets
    vstart = np.array(vertices_start)
    distances = igl.exact_geodesic(v, f, vstart, vertices_target)
    return distances

def compute_geodesic_from_start_to_target_vkeys(mesh, start_v_keys_list, target_v_keys_list):
    """
    compute distances from one edges to another edge and get longest way and shortest way
    """
    v, f = mesh.to_vertices_and_faces()
    v = np.array(v)
    f = np.array(f)
    vertices_start = np.array(start_v_keys_list)
    vertices_target = np.array(target_v_keys_list)
    distances = igl.exact_geodesic(v, f, vertices_start, vertices_target)
    return distances

def get_distances_list_from_every_boundary(mesh, boundary_vertex_keys_list):
    distances_list = []
    for seam_vkeys in boundary_vertex_keys_list:
        distances = list(compute_geodesic_to_every_vertex(mesh, seam_vkeys))
        distances_list.append(distances)
    return distances_list

## get way informations ##
def calculate_way_length_on_every_vertex(mesh, seam_vertex_keys_list):
    distances_list = get_distances_list_from_every_boundary(mesh, seam_vertex_keys_list)
    vkeys = mesh.vertices()
    way_list = []
    for vkey in vkeys:
        vdis_list = [distances[vkey] for distances in distances_list]
        vdis_list.sort()
        d00 = vdis_list[0]
        d01 = vdis_list[1]
        way = d00 + d01
        way_list.append(way)
    temp_list = [way for way in way_list]
    temp_list.sort()
    short_way = temp_list[0]
    long_way = temp_list[-1]
    ave_way = sum(temp_list) / len(temp_list)

    return short_way, long_way, ave_way, way_list

def get_gap_ratio(mesh, boundary_vertex_keys_list):
    short_way, long_way, ave_way, Ds_list = calculate_way_length_on_every_vertex(mesh, boundary_vertex_keys_list)
    gap_ratio = (long_way - short_way) / long_way
    return gap_ratio


# ##################################################################################
# ## combination of several types of geodesics in a mesh ##
# ##################################################################################
# def vertex_distribution_of_geodesic_realm(mesh, distances_list):
#     # distances_list = get_distances_list_from_every_seams(mesh, seam_vertex_keys_list)
#     realm_dict = {}
#     for i in range(len(distances_list)):
#         realm_dict["seam_0" + str(i)] = []
#     vkeys = mesh.vertices()
#     for vkey in vkeys:
#         dist_list = [distances[vkey] for distances in distances_list]
#         closest = min(dist_list)
#         seam_number = dist_list.index(closest)
#         realm_dict["seam_0" + str(seam_number)].append(vkey)
#     return realm_dict
#
# def custom_differences_00(mesh, distances_list, time):
#     ## pure distance differences no additional distance ##
#     global result
#     # distances_list = get_distances_list_from_every_seams(mesh, seam_vertex_keys_list)
#     vkeys = mesh.vertices()
#     if len(distances_list) <= 2:
#         difs = [abs(d01 * time - d00 * (1 - time)) for d00, d01 in zip(distances_list[0], distances_list[1])]
#         result = difs
#     elif len(distances_list) >= 3:
#         difs_list = []
#         for i in range(len(distances_list)):
#             first_distances = distances_list[i]
#             difs = []
#             for vkey in vkeys:
#                 left_distances_list = []
#                 for j, distances in enumerate(distances_list):
#                     if j != i:
#                         left_distances_list.append(distances)
#                 dist_list = [distances[vkey] for distances in distances_list]
#                 first_seam_dist = first_distances[vkey]
#                 target_seam_dist = min(dist_list)
#
#                 d00 = first_seam_dist
#                 d01 = target_seam_dist
#                 dif = abs(d01*time - d00*(1-time))
#                 difs.append(dif)
#             difs_list.append(difs)
#         result = difs_list
#     return result
#
# def custom_differences_01(mesh, seam_vertex_keys_list, time, width=50):
#     """
#     this transformed distances is for the connection detail between two seams
#     """
#     ## custom distance differences with changing offset ##
#     distances_list = get_distances_list_from_every_boundary(mesh, seam_vertex_keys_list)
#     short_way, long_way, ave_way, Ds_list = calculate_way_length_on_every_vertex(mesh, seam_vertex_keys_list)
#     ## get the differences from start seam and from last seam ##
#     difs00 = []
#     for d00, d01 in zip(distances_list[0], distances_list[1]):
#         way = d00 + d01
#         x = abs(long_way - way)
#         y = abs(ave_way - way)
#         check_value = abs(long_way - ave_way)
#         value = width * math.cos((2 * math.pi / (long_way - short_way)) * x)
#         value = value * (1 - 1 / (y + 1))
#         if value <= 0:
#             value = 0
#         else:
#             value = value
#         addv = value * (0.5 - abs(0.5-time))
#         dif = abs((d01 + addv) * time - d00 * (1 - time))
#         difs00.append(dif)
#     difs01 = []
#     for d00, d01 in zip(distances_list[0], distances_list[1]):
#         way = d00 + d01
#         x = abs(long_way - way)
#         y = abs(ave_way - way)
#         check_value = abs(long_way - ave_way)
#         wid = width
#         value = wid * math.cos((2 * math.pi / (long_way - short_way)) * x)
#         #    value = value * (1 - 1/(y+1))
#         if value >= 0:
#             value = 0
#         else:
#             value = value
#         addv = value * (0.5 - abs(0.5 - time))
#         dif01 = abs((d01 + addv) * time - (d00) * (1 - time))
#         difs01.append(dif01)
#     difs_list = [difs00, difs01]
#     return difs_list
#
# def custom_differences_02(mesh, seam_vertex_keys_list, time, gap_ratio):
#     """
#
#     this transformed distances is for the adapting way gap solution
#
#     """
#     ## custom distance differences with changing offset ##
#     difs_list = []
#     distances_list = get_distances_list_from_every_boundary(mesh, seam_vertex_keys_list)
#     short_way, long_way, ave_way, Ds_list = calculate_way_length_on_every_vertex(mesh, seam_vertex_keys_list)
#     width = (short_way * gap_ratio)
#     ## get the differences from start seam and from last seam ##
#     difs00 = []
#     for d00, d01 in zip(distances_list[0], distances_list[1]):
#         way = d00 + d01
#         x = abs(ave_way - way)
#         check_value = abs(ave_way - long_way)
#         wid = width
#         if (ave_way - way) >= 0:
#             ## make the value smoother ##
#             value = wid * ((x / check_value) ** 1.5) * (1 - 1 / (x + 1))
#         else:
#             value = 0
#         addv = value * (0.5 - abs(0.5 - time))
#         dif00 = abs((d01 - addv) * time - d00 * (1 - time))
#         difs00.append(dif00)
#     difs01 = []
#     for d00, d01 in zip(distances_list[0], distances_list[1]):
#         way = d00 + d01
#         x = abs(ave_way - way)
#         check_value = abs(ave_way - long_way)
#         wid = width
#         if (ave_way - way) >= 0:
#             ## make the value smoother ##
#             value = wid * ((x / check_value) ** 1.5) * (1 - 1 / (x + 1))
#         else:
#             value = 0
#         addv = value * (0.5 - abs(0.5 - time))
#         dif01 = abs((d01 + addv) * time - d00 * (1 - time))
#         difs01.append(dif01)
#     difs_list = [difs00, difs01]
#     return difs_list
#

"""
"""
#############################################################################
## for offsetting path points ##
#############################################################################


def get_closest_points_from_pts_cloud(fromPt, toPtsCloud, num_getPts):
    distances = []
    for toPt in toPtsCloud:
        dis = fromPt.distance_to_point(toPt)
        distances.append(dis)
    list = [dis for dis in distances]
    list.sort()

    closest_pts = []
    closest_distances = list[:num_getPts]
    for distance in closest_distances:
        count = 0
        for toPt in toPtsCloud:
            d = fromPt.distance_to_point(toPt)
            if d == distance:
                closest_pts.append(toPt)
                count += 1
                if count == num_getPts: break
    if num_getPts == 1:
        closest_pts = closest_pts[0]
        closest_distances = closest_distances[0]
    return closest_pts, closest_distances

def get_distance_to_pts_cloud(fromPt, toPtsCloud):
    closest_pts, closest_distances = get_closest_points_from_pts_cloud(fromPt, toPtsCloud, 3)
    closest_distance = closest_distances[0]
    for i in range(1):
        ## first two points ##
        measure = closest_distances[0] + closest_distances[1]
        x = (closest_distances[1] / measure) * closest_pts[0].x + (closest_distances[0] / measure) * closest_pts[1].x
        y = (closest_distances[1] / measure) * closest_pts[0].y + (closest_distances[0] / measure) * closest_pts[1].y
        z = (closest_distances[1] / measure) * closest_pts[0].z + (closest_distances[0] / measure) * closest_pts[1].z
        point_00 = Point(x, y, z)
        distance_00 = fromPt.distance_to_point(point_00)
        if distance_00 == 0:
            cl_distance = 0
            cl_point = point_00
            continue
        ## third point ##
        measure = closest_distances[0] + closest_distances[2]
        x = (closest_distances[0] / measure) * closest_pts[2].x + (closest_distances[2] / measure) * closest_pts[0].x
        y = (closest_distances[0] / measure) * closest_pts[2].y + (closest_distances[2] / measure) * closest_pts[0].y
        z = (closest_distances[0] / measure) * closest_pts[2].z + (closest_distances[2] / measure) * closest_pts[0].z
        point_01 = Point(x, y, z)
        distance_01 = fromPt.distance_to_point(point_01)
        if distance_01 == 0:
            cl_distance = 0
            cl_point = point_01
            continue
        ## cl point 00 ##
        measure = distance_00 + distance_01
        x = (distance_01 / measure) * point_01.x + (distance_00 / measure) * point_00.x
        y = (distance_01 / measure) * point_01.y + (distance_00 / measure) * point_00.y
        z = (distance_01 / measure) * point_01.z + (distance_00 / measure) * point_00.z
        clpt00 = Point(x, y, z)
        cld00 = fromPt.distance_to_point(clpt00)
        if cld00 == 0:
            cl_distance = 0
            cl_point = clpt00
            continue
        ## cl point_01 ##
        measure = closest_distances[0] + cld00
        x = (closest_distances[0] / measure) * clpt00.x + (cld00 / measure) * closest_pts[0].x
        y = (closest_distances[0] / measure) * clpt00.y + (cld00 / measure) * closest_pts[0].y
        z = (closest_distances[0] / measure) * clpt00.z + (cld00 / measure) * closest_pts[0].z
        cl_point = Point(x, y, z)
        cl_distance = fromPt.distance_to_point(cl_point)
        if cl_distance == 0:
            cl_distance = 0
            cl_point = cl_point
            continue
        cllist = [closest_distance, distance_00, distance_01, cld00, cl_distance]
        clptlist = [closest_pts[0], point_00, point_01, clpt00, cl_point]
        cl_distance = min(cllist)
        cl_point = clptlist[cllist.index(cl_distance)]
    return cl_distance, cl_point
































