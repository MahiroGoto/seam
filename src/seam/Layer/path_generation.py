from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline, intersection_line_line, intersection_line_plane

from seam.boundary import distance_calculation, seam_crv
from seam.utils import utils, parameters

import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################

def get_intersection_pts_two_polyline(poly_0, poly_1, touch=True):
    """
    search intersection points of two polylines with using compas Poly
    This intersection point is the position where these two polylines are touching each other.
    If "touch = False", can get intersection points that is not where they are touching.
    """
    pts = []
    for i in range(len(poly_0) - 1):
        line1 = poly_0[i:i + 2]
        for j in range(len(poly_1) - 1):
            line2 = poly_1[j:j + 2]
            interPts = intersection_line_line(line1, line2)
            ## coordinate should be rounded to check if these two points are the same point or not ##
            interPts_coord = []
            for interPt in interPts:
                interPt_coord = [round(interPt[0],2), round(interPt[1],2), round(interPt[2],2)]
                interPts_coord.append(interPt_coord)
            # print(interPts_coord)

            if touch:
                if interPts_coord[0] == interPts_coord[1] and interPts_coord[0] != None:
                    coord = interPts[0]
                    interPt = Point(coord[0], coord[1], coord[2])
                    # print("interPt :", interPt)
                    pts.append(interPt)
            else:
                count = 0
                for interPt in interPts:
                    if interPt.on_polyline(poly_0) or interPt.on_polyline(poly_1):
                        count += 1
                if count == len(interPts):
                    pts.extend(interPts)
    # print(len(pts))
    if len(pts) == 1:
        return pts[0]
    elif len(pts) == 0:
        return None
    else:
        return pts


##################################
## layer generation ##
##################################

class Get_Sequence:
    """
    to get layer path pts dictionary, you run "create_layers_path_dict_from_difs_dict(diftype)"
    then self.layer_pathPts_dict
    """
    def __init__(self, MESH, boundary_vertex_keys_list, iteration=20):
        self.mesh = MESH
        self.bvkl = boundary_vertex_keys_list
        ## set differences class from distance_calculation ##
        if len(self.bvkl) == 2:
            self.Differences = distance_calculation.Differences(self.mesh, self.bvkl)
            self.distances_list = self.Differences.distances_list
            self.short_way = self.Differences.short_way
            self.long_way = self.Differences.long_way

        elif len(self.bvkl) == 3:
            self.Distance_Attributes_three = distance_calculation.Distance_Attributes_three(self.mesh, self.bvkl)
            self.distances_list = self.Distance_Attributes_three.get_distances_list_from_every_boundary()
            self.short_way = self.Distance_Attributes_three.short_way
            self.long_way = self.Distance_Attributes_three.long_way

        elif len(self.bvkl) == 4:
            self.Distance_Attributes_four = distance_calculation.Distance_Attributes_four(self.mesh, self.bvkl)
            self.distances_list = self.Distance_Attributes_four.get_distances_list_from_every_boundary()
            self.short_way = self.Distance_Attributes_four.short_way
            self.long_way = self.Distance_Attributes_four.long_way


        self.boundary_num = len(self.distances_list)
        ## set the layer values ##
        self.iter = iteration
        self.layer_num = self.get_layer_number()
        logger.info("layer_num"+str(self.layer_num))
        logger.info("min_layer_height :"+str(self.short_way/self.layer_num))
        logger.info("max_layer_height :"+str(self.long_way/self.layer_num))
        self.gap_ratio = distance_calculation.get_gap_ratio(self.mesh, self.bvkl)
        self.time_step = 1 / (self.layer_num-1)


    def get_layer_number(self):
        max_h = parameters.get_param("max_layer_height")
        min_h = parameters.get_param("min_layer_height")
        print()
        num = ((self.short_way / min_h) + (self.long_way / max_h)) / 2
        num = int(round(num, 0))
        ## check the small and big height in the proper range ##
        h_small = self.short_way / num
        h_big = self.long_way / num
        for i in range(self.iter):
            if h_small >+ min_h and h_big <= max_h:
                break
            elif h_small < min_h:
                num -= 1
                h_small = self.short_way / num
                h_big = self.long_way / num
            elif h_big > max_h:
                num += 1
                h_small = self.short_way / num
                h_big = self.long_way / num
        layer_num = int(num)
        return layer_num

    ## layer sequence generation depending on the number of branch ##
    ##################################################################################################################
    ## creation of layer path of two brancing connection by distance differecnes ##
    def differences_function_two_boundaries(self, base_boundary_num,
                                            addLayer = 0,
                                            untilTime=0.5,
                                            minValue=0.5,
                                            frequency=2.7):
        difs_list = []
        if minValue < 0.7:
            add = 1 + (0.7 - minValue) * 1.2
            layer_num = int(round(self.layer_num * add, 0)) + addLayer
            time_step = 1 / layer_num
        else:
            layer_num = self.layer_num + addLayer
            time_step = 1 / layer_num

        if minValue == 1:
            untilTime = untilTime + 0.1

        for i in range(layer_num):
            if i == 0:
                time = time_step / 1000
            elif i == layer_num - 1:
                time = 1 - time_step / 1000
            else:
                time = time_step * i
            ## squeeze with untilTime ##
            if time > untilTime:
                break

            difs = self.Differences.\
                calculate_custom_differences_with_two_boundaries(base_boundary_num=base_boundary_num,
                                                                 time=time,
                                                                 minValue=minValue,
                                                                 frequency=frequency)
            difs_list.append(difs)
        return difs_list, layer_num
        # if diftype == 0:
        #     for i in range(self.layer_num):
        #         if i == self.layer_num - 1:
        #             time = 1  - self.time_step / (10**10)
        #         else:
        #             time = self.time_step * i
        #         difs = self.Differences.equal_differences(time)
        #         difs_list.append(difs)
        # ## connection detail ##
        # elif diftype == 1.1:
        #     for i in range(self.layer_num):
        #         if i == self.layer_num - 1 :
        #             time = 1 - self.time_step / (10**10)
        #         else :
        #             time = self.time_step * i
        #         difs = self.Differences.custom_differences_01_first(base_boundary_num=base_boundary_num,
        #                                                             time=time,
        #                                                             width=75)
        #         difs_list.append(difs)
        # elif diftype == 1.2:
        #     for i in range(self.layer_num):
        #         if i == self.layer_num - 1 :
        #             time = 1 - self.time_step / (10**10)
        #         else :
        #             time = self.time_step * i
        #         # time = time - 1
        #         difs = self.Differences.custom_differences_01_second(time, width=10)
        #         difs_list.append(difs)
        #
        # # ## connection detail 2.0 ##
        # # elif diftype == 3.1:
        # #     for i in range(self.layer_num):
        # #         if i == self.layer_num - 1:
        # #             time = 1 - self.time_step / (10**10)
        # #         else :
        # #             time = self.time_step * i
        # #         difs = self.Differences.custom_differences_03_first(time, width=280, flat_ratio=0.2)
        # #         difs_list.append(difs)
        # # elif diftype == 3.2:
        # #     for i in range(self.layer_num):
        # #         if i == self.layer_num - 1:
        # #             time = self.time_step * i - self.time_step / 1000
        # #         else:
        # #             time = self.time_step * i
        # #         difs = self.Differences.custom_differences_03_second(time, width=200, flat_ratio=0.2)
        # #         difs_list.append(difs)
        # #
        # # ## way gap solution ##
        # # elif diftype == 2.1:
        # #     for i in range(self.layer_num):
        # #         if i == self.layer_num - 1:
        # #             time = 1 - self.time_step / (10**10)
        # #         else:
        # #             time = self.time_step * i
        # #         difs = self.Differences.custom_differences_02_first(time)
        # #         difs_list.append(difs)
        # # elif diftype == 2.2:
        # #     for i in range(self.layer_num):
        # #         if i == self.layer_num - 1:
        # #             time = 1 - self.time_step / (10**10)
        # #         else:
        # #             time = self.time_step * i
        # #         # time = time - 1
        # #         difs = self.Differences.custom_differences_02_second(time)
        # #         difs_list.append(difs)
        # return difs_list

    def create_sequence_paths_from_difs_two_boundaries(self, base_boundary_num=0,
                                                       untilTime=0.5, addLayer=0,
                                                       minValue=0.5, frequency=2.5,
                                                       matchPathDirection=True,
                                                       modifyStartingPathPt=True):
        difs_list, layer_num = self.differences_function_two_boundaries(base_boundary_num=base_boundary_num,
                                                                        addLayer=addLayer,
                                                                        untilTime=untilTime,
                                                                        minValue=minValue, frequency=frequency)
        layer_pathPts_list = []
        layer_centrePt_list = []
        for difs in difs_list:
            crvPts, centrePt = seam_crv.get_layer_crvPts_from_distance_differences_on_Mesh(self.mesh, difs)
            layer_centrePt_list.append(centrePt)
            layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(crvPts, double_iteration=True,
                                                                       first_resolution=0.8)
            layer_pathPts_list.append(layer_pathPts)

        ## check layer height ##
        iteration = 20
        max_height, min_height, ave_height = self.measure_layer_heights(layer_pathPts_list)
        print("first_max_height, first_min_height =", max_height, min_height)
        for i in range(iteration):
            if i != 0:
                max_height, min_height, ave_height = self.measure_layer_heights(layer_pathPts_list)
            if max_height > 2.1:
                addLayer += 1
                difs_list, layer_num = self.differences_function_two_boundaries(base_boundary_num=base_boundary_num,
                                                                                addLayer=addLayer,
                                                                                untilTime=untilTime,
                                                                                minValue=minValue, frequency=frequency)
                layer_pathPts_list = []
                layer_centrePt_list = []
                for difs in difs_list:
                    crvPts, centrePt = seam_crv.get_layer_crvPts_from_distance_differences_on_Mesh(self.mesh, difs)
                    layer_centrePt_list.append(centrePt)
                    layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(crvPts, double_iteration=True,
                                                                               first_resolution=0.8)
                    layer_pathPts_list.append(layer_pathPts)

            elif min_height < 0.7:
                addLayer -= 1
                difs_list, layer_num = self.differences_function_two_boundaries(base_boundary_num=base_boundary_num,
                                                                                addLayer=addLayer,
                                                                                untilTime=untilTime,
                                                                                minValue=minValue, frequency=frequency)
                layer_pathPts_list = []
                layer_centrePt_list = []
                for difs in difs_list:
                    crvPts, centrePt = seam_crv.get_layer_crvPts_from_distance_differences_on_Mesh(self.mesh, difs)
                    layer_centrePt_list.append(centrePt)
                    layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(crvPts, double_iteration=True,
                                                                               first_resolution=0.8)
                    layer_pathPts_list.append(layer_pathPts)

            else:
                break

        print("max_height, min_height =", max_height, min_height)
        logger.info("layer num :" + str(layer_num))

        ## modifying the layer points ##
        if modifyStartingPathPt:
            layer_pathPts_list = self.get_proper_next_pt_in_the_next_layer(layer_pathPts_list)
        if matchPathDirection:
            layer_pathPts_list = self.match_the_every_layer_path_direction(layer_pathPts_list)
        if modifyStartingPathPt:
            layer_pathPts_list = self.get_proper_next_pt_in_the_next_layer(layer_pathPts_list)
        if matchPathDirection:
            layer_pathPts_list = self.match_the_every_layer_path_direction(layer_pathPts_list)
        return layer_pathPts_list


    ##################################################################################################################
    ## creation of layer paths of three branching node by using boundary attributes ##
    def attributes_function_multi_boundaries(self, base_boundary_num, untilTime=0.5, addLayer=0,
                                             minValue=0.5, frequency=1.0,
                                             longWayExtenction=False):
        attrs_list = []
        difs_list = []
        if minValue < 0.9:
            add = 1 + (0.9 - minValue) * 2.5
            layer_num = int(round(self.layer_num * add, 0)) + addLayer
            time_step = 1 / layer_num
            for i in range(layer_num):
                if i == 0:
                    time = time_step / 1000
                elif i == layer_num - 1:
                    time = 1 - time_step / 1000
                else:
                    time = time_step * i
                ## time break ##
                if time > untilTime:
                    break

                if len(self.bvkl) == 3:
                    attrs, difs = self.Distance_Attributes_three.\
                        get_attributes_from_one_base_boundary(base_boundary_num, time,
                                                              minValue=minValue,
                                                              frequency=frequency,
                                                              longWayExtention=longWayExtenction)
                    attrs_list.append(attrs)
                    difs_list.append(difs)
                elif len(self.bvkl) == 4:
                    attrs, difs = self.Distance_Attributes_four.\
                        get_attributs_from_one_base_boundary(base_boundary_num, time,
                                                             minValue=minValue,
                                                             frequency=frequency,
                                                             longWayExtention=longWayExtenction)
                    attrs_list.append(attrs)
                    difs_list.append(difs)

        else:
            layer_num = self.layer_num + addLayer
            time_step = 1 / layer_num
            for i in range(layer_num):
                if i == 0:
                    time = time_step / 1000
                elif i == layer_num-1:
                    time = 1 - time_step / 1000
                else:
                    time = time_step * i
                ## time break ##
                if time > untilTime:
                    break

                if len(self.bvkl) == 3:
                    attrs, difs = self.Distance_Attributes_three.\
                        get_attributes_from_one_base_boundary(base_boundary_num, time,
                                                              minValue=minValue,
                                                              frequency=frequency,
                                                              longWayExtention=longWayExtenction)
                    attrs_list.append(attrs)
                    difs_list.append(difs)
                elif len(self.bvkl) == 4:
                    attrs, difs = self.Distance_Attributes_four.\
                        get_attributs_from_one_base_boundary(base_boundary_num, time,
                                                             minValue=minValue,
                                                             frequency=frequency,
                                                             longWayExtention=longWayExtenction)
                    attrs_list.append(attrs)
                    difs_list.append(difs)

        return attrs_list, difs_list, layer_num


    def create_sequence_paths_from_distance_attributes_multi_boundaries(self, base_boundary_num,
                                                                        untilTime=0.5, addLayer=0,
                                                                        minValue=0.5, frequency=1.0,
                                                                        longWayExtention=False,
                                                                        matchPathDirection=True,
                                                                        modifyStartingPathPt=True):
        ## set the layer centre point for the directionMatching depending on "base_boundary_num" ##
        attrs_list, difs_list, layer_num = self.attributes_function_multi_boundaries(base_boundary_num, untilTime=untilTime,
                                                                                     addLayer=addLayer,
                                                                                     minValue=minValue,
                                                                                     frequency=frequency,
                                                                                     longWayExtenction=longWayExtention)
        layer_pathPts_list = []
        layer_centrePt_list = []
        for attrs, difs in zip(attrs_list, difs_list):
            ## get the curve_pts and centrePt of this layer ##
            if len(self.bvkl) == 3:
                curve_pts, centrePt = seam_crv.\
                    get_layer_crvPts_from_distance_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                      base_boundary_num, path_sequence=True)
                layer_centrePt_list.append(centrePt)
                layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                layer_pathPts_list.append(layer_pathPts)
            elif len(self.bvkl) == 4:
                curve_pts, centrePt = seam_crv.\
                    get_layer_crvPts_from_distance_four_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                           base_boundary_num, path_sequence=True)
                layer_centrePt_list.append(centrePt)
                layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                layer_pathPts_list.append(layer_pathPts)

        ## check layer heights ##
        iteration = 20
        max_height, min_height, ave_height = self.measure_layer_heights(layer_pathPts_list)
        print("first_max_height, first_min_height =", max_height, min_height)
        for i in range(iteration):
            if i != 0:
                max_height, min_height, ave_height = self.measure_layer_heights(layer_pathPts_list)
            if max_height > 2.5:
                addLayer += 1
                attrs_list, difs_list, layer_num = self.attributes_function_multi_boundaries(base_boundary_num, untilTime=untilTime,
                                                                                  addLayer=addLayer,
                                                                                  minValue=minValue,
                                                                                  frequency=frequency,
                                                                                  longWayExtenction=longWayExtention)
                layer_pathPts_list = []
                layer_centrePt_list = []
                for attrs, difs in zip(attrs_list, difs_list):
                    ## get the curve_pts and centrePt of this layer ##
                    if len(self.bvkl) == 3:
                        curve_pts, centrePt = seam_crv. \
                            get_layer_crvPts_from_distance_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                              base_boundary_num, path_sequence=True)
                        layer_centrePt_list.append(centrePt)
                        layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                        layer_pathPts_list.append(layer_pathPts)
                    elif len(self.bvkl) == 4:
                        curve_pts, centrePt = seam_crv. \
                            get_layer_crvPts_from_distance_four_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                                   base_boundary_num, path_sequence=True)
                        layer_centrePt_list.append(centrePt)
                        layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                        layer_pathPts_list.append(layer_pathPts)

            elif min_height < 1.0:
                addLayer -= 1
                attrs_list, difs_list, layer_num = self.attributes_function_multi_boundaries(base_boundary_num, untilTime=untilTime,
                                                                                  addLayer=addLayer,
                                                                                  minValue=minValue,
                                                                                  frequency=frequency,
                                                                                  longWayExtenction=longWayExtention)
                layer_pathPts_list = []
                layer_centrePt_list = []
                for attrs, difs in zip(attrs_list, difs_list):
                    ## get the curve_pts and centrePt of this layer ##
                    if len(self.bvkl) == 3:
                        curve_pts, centrePt = seam_crv. \
                            get_layer_crvPts_from_distance_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                              base_boundary_num, path_sequence=True)
                        layer_centrePt_list.append(centrePt)
                        layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                        layer_pathPts_list.append(layer_pathPts)
                    elif len(self.bvkl) == 4:
                        curve_pts, centrePt = seam_crv. \
                            get_layer_crvPts_from_distance_four_attributes_on_Mesh(self.mesh, attrs, difs,
                                                                                   base_boundary_num, path_sequence=True)
                        layer_centrePt_list.append(centrePt)
                        layer_pathPts = seam_crv.create_layer_path_pts_from_crvPts(curve_pts)
                        layer_pathPts_list.append(layer_pathPts)

            else:
                break
        print("max_height, min_height =", max_height, min_height)
        logger.info("layer num :" + str(layer_num))
        ## modifying the layer points ##
        if modifyStartingPathPt and matchPathDirection:
            layer_pathPts_list = self.get_proper_next_pt_in_the_next_layer(layer_pathPts_list)
            layer_pathPts_list = self.match_the_every_layer_path_direction(layer_pathPts_list)
        if modifyStartingPathPt and matchPathDirection:
            layer_pathPts_list = self.get_proper_next_pt_in_the_next_layer(layer_pathPts_list)
            layer_pathPts_list = self.match_the_every_layer_path_direction(layer_pathPts_list)
        if modifyStartingPathPt and matchPathDirection:
            layer_pathPts_list = self.get_proper_next_pt_in_the_next_layer(layer_pathPts_list)
            layer_pathPts_list = self.match_the_every_layer_path_direction(layer_pathPts_list)

        return layer_pathPts_list


    ##################################################################################################################
    ## layer sequence modify tool ##
    def match_the_every_layer_path_direction(self, layer_pathPts_list):
        ## alternative way ##
        updated_pathPts_list = []
        first_pathPts = layer_pathPts_list[0]
        pre_pathPts = first_pathPts
        for i, layer_pathPts in enumerate(layer_pathPts_list):
            if i == 0:
                updated_pathPts_list.append(layer_pathPts)
            elif i != 0:
                pre_direction = pre_pathPts[-1] - pre_pathPts[1]
                pre_direction.unitize()
                current_direction = layer_pathPts[-1] - layer_pathPts[1]
                current_direction.unitize()
                angle = pre_direction.angle(current_direction)
                ## second check ##

                if angle > 0.5 * math.pi:
                    print("reverse!! ", "layer_num is :", i)
                    layer_pathPts.reverse()
                    updated_pathPts_list.append(layer_pathPts)
                else:
                    updated_pathPts_list.append(layer_pathPts)
                pre_pathPts = layer_pathPts
        return updated_pathPts_list

    def get_proper_next_pt_in_the_next_layer(self, pathPts_list):
        first_pathPts = pathPts_list[0]
        pre_first_pt = first_pathPts[0]
        pre_last_pt = first_pathPts[-1]
        updated_pts_list = []
        updated_pts_list.append(first_pathPts)
        for i, pathPts in enumerate(pathPts_list):
            if i != 0:
                clpts, cldistances = distance_calculation.get_closest_points_from_pts_cloud(pre_first_pt, pathPts, 2)
                # vec01 = clpts[0] - pre_last_pt
                # vec02 = clpts[1] - pre_last_pt
                # vec = pre_first_pt - pre_last_pt
                # ## select next point by calculating angle ##
                # angle01 = vec.angle(vec01)
                # angle02 = vec.angle(vec02)
                # if angle01 < angle02:
                #     start_index = pathPts.index(clpts[0])
                # else:
                #     start_index = pathPts.index(clpts[1])
                start_index = pathPts.index(clpts[0])
                updated_pathPts = pathPts[start_index:] + pathPts[:start_index]
                updated_pts_list.append(updated_pathPts)
                pre_first_pt = updated_pathPts[0]
                pre_last_pt = updated_pathPts[-1]
        return updated_pts_list

    def measure_layer_heights(self, layer_pathPts_list):
        string = zip(layer_pathPts_list[:-1], layer_pathPts_list[1:])
        layer_height_list = []
        for pre_pathPts, post_pathPts in string:
            pre_pathPt = pre_pathPts[0]
            clpt, cldis = distance_calculation.get_closest_points_from_pts_cloud(pre_pathPt, post_pathPts, 1)
            layer_height_list.append(cldis)
        layer_height_list.sort()
        max_height = layer_height_list[-1]
        min_height = layer_height_list[0]
        ave_height = sum(layer_height_list) / len(layer_height_list)

        return max_height, min_height, ave_height


#######################################################################################################################
## modify merged sequence tool ##
class MergedSequence:
    def __init__(self, merged_sequence):
        self.merged_sequence = merged_sequence
        ## excecusion ##
        self.modified_merged_sequence = self.modify_sequence(self.merged_sequence)

    def modify_sequence(self, layer_pathPts_list):
        updated_pts_list = self.get_proper_next_pts_in_the_next_layer(layer_pathPts_list)
        updated_pts_list = self.match_the_every_layer_path_direction(updated_pts_list)
        updated_pts_list = self.get_proper_next_pts_in_the_next_layer(updated_pts_list)
        updated_pts_list = self.match_the_every_layer_path_direction(updated_pts_list)

        return updated_pts_list

    def get_proper_next_pts_in_the_next_layer(self, layer_pathPts_list):
        first_pathPts = layer_pathPts_list[0]
        pre_first_pt = first_pathPts[0]
        pre_last_pt = first_pathPts[-1]
        updated_pts_list = []
        updated_pts_list.append(first_pathPts)
        for i, pathPts in enumerate(layer_pathPts_list):
            if i != 0:
                clpts, cldistances = distance_calculation.get_closest_points_from_pts_cloud(pre_first_pt, pathPts, 2)
                # vec01 = clpts[0] - pre_last_pt
                # vec02 = clpts[1] - pre_last_pt
                # vec = pre_first_pt - pre_last_pt
                # ## select next point by calculating angle ##
                # angle01 = vec.angle(vec01)
                # angle02 = vec.angle(vec02)
                # if angle01 < angle02:
                #     start_index = pathPts.index(clpts[0])
                # else:
                #     start_index = pathPts.index(clpts[1])
                start_index = pathPts.index(clpts[0])
                updated_pathPts = pathPts[start_index:] + pathPts[:start_index]
                updated_pts_list.append(updated_pathPts)
                pre_first_pt = updated_pathPts[0]
                pre_last_pt = updated_pathPts[-1]
        return updated_pts_list

    def match_the_every_layer_path_direction(self, layer_pathPts_list):
        updated_pathPts_list = []
        first_pathPts = layer_pathPts_list[0]
        pre_pathPts = first_pathPts
        for i, layer_pathPts in enumerate(layer_pathPts_list):
            if i == 0:
                updated_pathPts_list.append(layer_pathPts)
            elif i != 0:
                pre_direction = pre_pathPts[-1] - pre_pathPts[1]
                pre_direction.unitize()
                current_direction = layer_pathPts[-1] - layer_pathPts[1]
                current_direction.unitize()
                angle = pre_direction.angle(current_direction)
                ## second check ##

                if angle > 0.5 * math.pi:
                    print("reverse!! ", "layer_num is :", i)
                    layer_pathPts.reverse()
                    updated_pathPts_list.append(layer_pathPts)
                else:
                    updated_pathPts_list.append(layer_pathPts)
                pre_pathPts = layer_pathPts
        return updated_pathPts_list


#######################################################################################################################
class OffsetTool:
    def __init__(self, layer_pathPts_list, other_pieces):
        self.layer_pathPts_list = layer_pathPts_list
        self.other_pieces = other_pieces
        self.eachother = False
        logger.info("offsetting...")

    def get_other_pieces_layers(self):
        other_pieces_layers = []
        for pathPts_list in self.other_pieces:
            for pathPts in pathPts_list:
                other_pieces_layers.append(pathPts)
        return other_pieces_layers
    ###########################
    ## execute tools ##
    def offset_layer_pathPts(self):
        pre_offset = 0.8
        offset_wid = 2.9
        other_pieces_layers = self.get_other_pieces_layers()
        new_layerPts_list = []
        for layerPts in self.layer_pathPts_list:
            ## get the centre point of this layer ##
            pts_x = [pt.x for pt in layerPts]
            pts_y = [pt.y for pt in layerPts]
            pts_z = [pt.z for pt in layerPts]
            x = sum(pts_x) / len(layerPts)
            y = sum(pts_y) / len(layerPts)
            z = sum(pts_z) / len(layerPts)
            centrePt = Point(x, y, z)
            ## move the points ##
            new_layerPts = []
            prev = pre_offset
            for layerPt in layerPts:
                is_in_realm = True
                is_on_others = False
                ## check if on other pieces ##
                dis_list_ = []
                for pathPts in other_pieces_layers:
                    cl_dis_, cl_pt = distance_calculation.\
                        get_distance_to_pts_cloud(layerPt, pathPts)
                    dis_list_.append(cl_dis_)
                closest_ = min(dis_list_)
                id = dis_list_.index(closest_)
                if closest_ < 1.5:
                    is_on_others = True
                # if self.eachother:
                #     ## check if in this realm or not ##
                #     if not is_on_others:
                #         newPt = layerPt
                #         new_layerPts.append(newPt)
                #         prev = pre_offset
                #         continue
                #     dis_list = []
                #     for base_pathPts in self.base_pathPts_list:
                #         cl_dis, cl_pt = distance_calculation.\
                #             get_distance_to_pts_cloud(layerPt, base_pathPts)
                #         dis_list.append(cl_dis)
                #     closest = min(dis_list)
                #     if closest <= 1.5:
                #         is_in_realm = True
                #     else:
                #         is_in_realm = False
                ## offset this point ##
                if is_in_realm and is_on_others:
                    if id == 0 or id == len(dis_list_)-1:
                        add = pre_offset
                    else:
                        add = 0
                    vector = layerPt - centrePt
                    vector.unitize()
                    vector.scale(offset_wid-prev-add)
                    vector = vector.scaled(-1)
                    newPt = layerPt + vector
                    prev = 0
                else:
                    if prev == 0:
                        vector = layerPt - centrePt
                        vector.unitize()
                        vector.scale(pre_offset)
                        vector = vector.scaled(-1)
                        newPt = layerPt + vector
                    else:
                        newPt = layerPt
                    prev = pre_offset
                new_layerPts.append(newPt)
            new_layerPts_list.append(new_layerPts)
        logger.info("offset done!!")
        return new_layerPts_list
























