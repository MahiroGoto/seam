from typing import Dict, Union
import math
import numpy as np

from compas.geometry import Vector, Point, Rotation, Circle, Plane, Polyline, intersection_line_line


import seam.utils.utils as utils
import logging

## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
########################


##########################
## polyline analysis utils
##########################

def calculate_circle_centre_and_radius_from_three_pts(three_pts_list):
    """
    points is compas Point
    """
    A = three_pts_list[0]
    B = three_pts_list[1]
    C = three_pts_list[2]
    vBA = B - A
    vBC = C - B
    normVec = vBA.cross(vBC)
    midBA = (B + A) / 2
    midBC = (B + C) / 2
    vec_BA = vBA.cross(normVec)
    vec_BC = vBC.cross(normVec)
    poly_0 = Polyline([midBA + vec_BA*10, midBA - vec_BA*10])
    poly_1 = Polyline([midBC - vec_BC*10, midBC + vec_BC*10])
    centrePt = get_intersection_pts_two_polyline(poly_0, poly_1, touch=True)

    radius = round(centrePt.distance_to_point(B), 3)

    return centrePt, radius

def calculate_normal_on_vertex_with_three_pts(three_pts_list):
    centrePt, radius = calculate_circle_centre_and_radius_from_three_pts(three_pts_list)
    point = three_pts_list[1]
    vector = point - centrePt
    # vector = centrePt - point
    normal = vector.unitized()

    return normal, radius, centrePt


class PolyVertex_Vector_Family:
    """
    this class is for the branch (polyline) without branching
    using compas Points
    """
    def __init__(self, pts_list):
        self.pts_list = pts_list

        self.normals = []
        self.get_normals_on_vertices()
        self.binorms = []
        self.get_binorms_on_vertices()
        self.tangets = []
        self.get_tangents_on_vertices()

        self.planes = []
        self.get_planes_vertices()
        self.planes_const = []
        self.get_planes_constitution()
        self.planes_data = []
        self.get_planes_data()

    def get_normals_on_vertices(self):
        if len(self.pts_list) > 2:
            threePts_string = zip(self.pts_list[:-2], self.pts_list[1:-1], self.pts_list[2:])
            for threePts in threePts_string:
                normal, radius, centrePt = calculate_normal_on_vertex_with_three_pts(threePts)
                self.normals.append(normal)
            ## first vertex ##
            vecfirst = self.pts_list[0] - self.pts_list[1]
            vecsecond = self.pts_list[2] - self.pts_list[1]
            binormal_temp = vecfirst.cross(vecsecond)
            binormal = binormal_temp.unitized()
            normal_temp = vecfirst.cross(binormal)
            normal = normal_temp.unitized()
            self.normals.insert(0, normal)
            ## last vertex ##
            veclast = self.pts_list[-1] - self.pts_list[-2]
            vecseclast = self.pts_list[-3] - self.pts_list[-2]
            binormal_temp = vecseclast.cross(veclast)
            binormal = binormal_temp.unitized()
            normal_temp = binormal.cross(veclast)
            normal = normal_temp.unitized()
            self.normals.append(normal)
        else:
            firstPt = self.pts_list[0]
            lastPt = self.pts_list[1]
            vec = firstPt - lastPt
            normal = vec.cross(Vector(-1,0,0))
            normal.unitize()
            self.normals.append(normal)
            self.normals.append(normal)

    def get_binorms_on_vertices(self):
        if len(self.pts_list) > 2:
            threePts_string = zip(self.pts_list[:-2], self.pts_list[1:-1], self.pts_list[2:])
            for threePts in threePts_string:
                vecA = threePts[0] - threePts[1]
                vecB = threePts[2] - threePts[1]
                binorm_temp = vecA.cross(vecB)
                binorm = binorm_temp.unitized()
                self.binorms.append(binorm)
            ## first binorm ##
            first_binorm = self.binorms[0]
            self.binorms.insert(0, first_binorm)
            ## last binorm ##
            last_binom = self.binorms[-1]
            self.binorms.append(last_binom)
        else:
            firstPt = self.pts_list[0]
            lastPt = self.pts_list[1]
            vec = firstPt - lastPt
            binorm = vec.cross(Vector(0,1,0))
            binorm.unitize()
            self.binorms.append(binorm)
            self.binorms.append(binorm)

    def get_tangents_on_vertices(self):
        if len(self.pts_list) > 0:
            for i in range(len(self.pts_list)):
                normal = self.normals[i]
                binorm = self.binorms[i]
                tangent_temp = normal.cross(binorm)
                tangent = tangent_temp.unitized()
                self.tangets.append(tangent)
        else:
            for normal in self.normals:
                self.tangets.append(normal)

    def get_planes_vertices(self):
        if len(self.pts_list) > 0:
            for i, vertex in enumerate(self.pts_list):
                normal = self.normals[i]
                binorm = self.binorms[i]
                if i != 0:
                    if self.normals[i-1].angle(normal) > 0.5*math.pi and \
                            self.binorms[i-1].angle(binorm) > 0.5*math.pi:
                        normal = normal*(-1)
                        binorm = binorm*(-1)
                plane = Plane.from_point_and_two_vectors(vertex, normal, binorm)
                self.planes.append(plane)


    def get_planes_constitution(self):
        """
        this plane data has centre point and two u and u  vectors. looks like;
        data = [Pt(0,0,0), [Vec(0,0,0), Vec(0,0,0)]]
        """
        for i, vertex in enumerate(self.pts_list):
            normal = self.normals[i]
            binorm = self.binorms[i]
            if i != 0:
                if self.normals[i-1].angle(normal) > 0.5*math.pi and \
                        self.binorms[i-1].angle(binorm) > 0.5*math.pi:
                    normal = normal*(-1)
                    binorm = binorm*(-1)
            const = [vertex, [normal, binorm]]
            self.planes_const.append(const)

    def get_planes_data(self):
        for i, vertex in enumerate(self.pts_list):
            normal = self.normals[i]
            binorm = self.binorms[i]
            if i != 0:
                if self.normals[i-1].angle(normal) > 0.5*math.pi and \
                        self.binorms[i-1].angle(binorm) > 0.5*math.pi:
                    normal = normal*(-1)
                    binorm = binorm*(-1)
            vertex_data = [vertex.x, vertex.y, vertex.z]
            pNormal_data = [normal.x, normal.y, normal.z]
            pBinorm_data = [binorm.x, binorm.y, binorm.z]
            const = [vertex_data, [pNormal_data, pBinorm_data]]
            self.planes_data.append(const)






##########################
## polyline
##########################

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


##########################
## bezier curve
##########################

def bezier_curve_from_two_vectors(start_end_pts, start_end_vectors, resolutionNum=10, degree=0.5, is_node=False):
    """
    create a bezier curve points from two vectors on the start point and end point

    direction of the vectors is the same with the flow of the bezier curve points
    """
    ## control points ##
    if is_node:
        flip_vec = 1
    else:
        flip_vec = -1
    distance = start_end_pts[0].distance_to_point(start_end_pts[1])
    start_vector = start_end_vectors[0].unitized()
    start_vector = start_vector * distance * degree * (1)
    end_vector = start_end_vectors[1].unitized()
    end_vector = end_vector * distance * degree * (flip_vec)
    control_pt_0 = start_end_pts[0] + start_vector
    control_pt_1 = start_end_pts[1] + end_vector

    pts = [start_end_pts[0], control_pt_0, control_pt_1, start_end_pts[1]]

    ############################################################################
    bezier_pts = []

    pts_list = []
    for i in range(len(pts) - 2):
        pts_sub = pts[i:3 + i]
        pts_list.append(pts_sub)

    for pts in pts_list:
        lines_second = []
        for j in range(resolutionNum + 1):
            control_linestring = zip(pts[:-1], pts[1:])
            time = j / resolutionNum
            pts_new = []
            for pt1, pt2 in control_linestring:
                pt = pt1 * (1 - time) + pt2 * (time)
                pts_new.append(pt)
            line = Polyline(pts_new)
            lines_second.append(line)

        bezier_pts_sub = []
        control_string = zip(lines_second[:-1], lines_second[1:])
        for line1, line2 in control_string:
            # print("line1 :", line1)
            # print("line2 ;", line2)
            intersecPts = get_intersection_pts_two_polyline(line1, line2, touch=True)
            bezier_pts_sub.append(intersecPts)

        bezier_pts.append(bezier_pts_sub)
    first_half = bezier_pts[0]
    second_half = bezier_pts[1]
    bezier = []
    number = len(first_half)
    for i in range(number):
        t = (i / (number - 1))
        pt_first_half = first_half[i]
        pt_second_half = second_half[i]
        pt = pt_first_half * (1-t) + pt_second_half * (t)
        bezier.append(pt)
    bezier.insert(0, start_end_pts[0])
    bezier.append(start_end_pts[1])

    return bezier

def measure_langth_of_bezier(bezier):
    pts_string = zip(bezier[:-1], bezier[1:])
    measure = 0
    for pre_pt, next_pt in pts_string:
        distance = pre_pt.distance_to_point(next_pt)
        measure += distance
    return measure

def create_point_on_polyline_with_pts_list(poly_pts, start_Pt, param):
    global aim_pt
    length = measure_langth_of_bezier(poly_pts)
    aim = length * param

    ## select the starting point ##
    dis = start_Pt.distance_to_point(poly_pts[0])
    dis_ = start_Pt.distance_to_point(poly_pts[-1])
    if dis <= dis_:
        poly_pts = poly_pts
    elif dis > dis_:
        reversed(poly_pts)

    pts_string = zip(poly_pts[:-1], poly_pts[1:])
    measure = 0
    for pre_pt, next_pt in pts_string:
        distance = pre_pt.distance_to_point(next_pt)
        measure += distance
        if measure > aim:
            pre_measure = measure - distance
            left_length = aim - pre_measure
            ratio = left_length / distance
            aim_pt = pre_pt * (1-ratio) + next_pt * ratio
    return aim_pt

def create_point_around_centre_of_polyline_with_pts_list(poly_pts, start_Pt, left_length):
    """
    get point that is located on the intersection with circle whose centre is the centre of the polyline
    """
    global aim_pt
    length = measure_langth_of_bezier(poly_pts)
    aim = (length-left_length)/2

    ## select the starting point ##
    dis = start_Pt.distance_to_point(poly_pts[0])
    dis_ = start_Pt.distance_to_point(poly_pts[-1])
    if dis <= dis_:
        poly_pts = poly_pts
    elif dis > dis_:
        poly_pts.reverse()

    pts_string = zip(poly_pts[:-1], poly_pts[1:])
    measure = 0
    for pre_pt, next_pt in pts_string:
        distance = pre_pt.distance_to_point(next_pt)
        measure += distance
        if measure > aim:
            extra = measure - aim
            aim_pt = pre_pt * (extra/distance) + next_pt * (1-(extra/distance))
            break
    return aim_pt

def compliment_polyline_pts_with_bezier(line_pts_list, degree=0.3, is_closed=True):
    num_segment = len(line_pts_list) - 1
    length = measure_langth_of_bezier(line_pts_list)
    average = length / num_segment
    print("average :", average)
    # pts_string = zip(line_pts_list[:-1], line_pts_list[1:])
    line_pts_list_modified = []
    for i in range(len(line_pts_list)):
        pt = line_pts_list[i]
        pt_ = line_pts_list[(i+1)%len(line_pts_list)]
        distance = pt.distance_to_point(pt_)
        if distance != 0:
            line_pts_list_modified.append(pt)

    # vectors = []
    # for pre_pt, next_pt in pts_string:
    #     if pre_pt.distance_to_point(next_pt) != 0:
    #         vector = next_pt - pre_pt
    #         vector = vector.unitized()
    #         vectors.append(vector)
    # last_vec = line_pts_list[0] - line_pts_list[-1]
    # last_vec.unitized()
    # vectors.append(last_vec)

    fixed_pts = []
    line_pts_list = line_pts_list_modified
    if is_closed:
        for i in range(len(line_pts_list)):
            prePt = line_pts_list[(i-1)%len(line_pts_list)]
            pt = line_pts_list[i%len(line_pts_list)]
            pt_ = line_pts_list[(i+1)%len(line_pts_list)]
            postPt = line_pts_list[(i+2)%len(line_pts_list)]

            distance = pt.distance_to_point(pt_)
            if distance > average *1.2:
                vec = (pt - prePt).unitized()
                vec_ = (postPt - pt_).unitized()
                vec_pair = [vec, vec_]
                pt_pair = [pt, pt_]
                bezier = bezier_curve_from_two_vectors(pt_pair, vec_pair, resolutionNum=5, degree=degree, is_node=False)
                bezier.pop(1)
                bezier.pop(-1)
                bezier.pop(-1)
                fixed_pts.extend(bezier)
            else:
                fixed_pts.append(pt)
    else:
        for i in range(len(line_pts_list)-1):
            prePt = line_pts_list[(i - 1) % len(line_pts_list)]
            pt = line_pts_list[i % len(line_pts_list)]
            pt_ = line_pts_list[(i + 1) % len(line_pts_list)]
            postPt = line_pts_list[(i + 2) % len(line_pts_list)]

            distance = pt.distance_to_point(pt_)
            if distance > average * 1.2:
                vec = (pt - prePt).unitized()
                vec_ = (postPt - pt_).unitized()
                vec_pair = [vec, vec_]
                pt_pair = [pt, pt_]
                bezier = bezier_curve_from_two_vectors(pt_pair, vec_pair, resolutionNum=5, degree=degree, is_node=False)
                bezier.pop(1)
                bezier.pop(-1)
                bezier.pop(-1)
                fixed_pts.extend(bezier)
            else:
                fixed_pts.append(pt)

    # string = zip(line_pts_list[:-1], line_pts_list[1:])
    # i = -1
    # for pt, pt_ in string:
    #     i += 1
    #     distance = pt.distance_to_point(pt_)
    #     print("distance :", distance)
    #     if distance > average * 1.5:
    #         pts_pair = [pt, pt_]
    #         vec_pair = [vectors[(i-1)], vectors[(i+1)]]
    #         bezier = bezier_curve_from_two_vectors(pts_pair, vec_pair, resolutionNum=5, degree=0.4, is_node=False)
    #         print("adding points!", bezier)
    #         fixed_pts.extend(bezier)
    #     else:
    #         fixed_pts.append(pt)
    #         fixed_pts.append(pt_)
    # if is_closed:
    #     distance = line_pts_list[-1].distance_to_point(line_pts_list[0])
    #     if distance > average * 2:
    #         pts_pair = [line_pts_list[-1], line_pts_list[0]]
    #         vec_pair = [vectors[-2], vectors[0]]
    #         bezier = bezier_curve_from_two_vectors(pts_pair,vec_pair, resolutionNum=5, degree=0.4, is_node=False)
    #         fixed_pts.extend(bezier)
    return fixed_pts

















