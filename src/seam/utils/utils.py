import os
import json
import sys
import math

from compas.geometry import Point, Vector

import logging
## logging settings ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s : %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
#######################################

#######################################
### Rhino utils for communicating
#######################################

def save_Json_from_gh(data, path, folder, name):
    print("name :", name)
    filename = os.path.join(path, folder, name)
    with open(filename, 'w') as f:
        f.write(json.dumps(data, indent=3, sort_keys=True))
    logger.info("saved at :"+str(filename))

def convert_pts_list_to_dict_data(pts_list):
    pts_coord = {}
    for i, pt in enumerate(pts_list):
        pts_coord[i] = [ pt.X, pt.Y, pt.Z ]
    return pts_coord

def convert_pts_list_to_data(pts_list):
    pts_coord = []
    for i, pt in enumerate(pts_list):
        pts_coord.append([ pt.X, pt.Y, pt.Z ])
    return pts_coord



#######################################
### Json utils
#######################################

### load Json file
def load_from_Json(path, name):
    filename = os.path.join(path, name)
    with open(filename, 'r') as f:
        data = json.load(f)
    logger.info('Loaded Json: ' + filename)
    return data


# def isocurves_segments_to_json(segments, path, name):
#     data = {}
#     for i, segment in enumerate(segments):
#         data['Segment_' + str(i)] = {}
#         for j, isocurve in enumerate(segment.isocurves):
#             data['Segment_' + str(i)]['Isocurve_' + str(j)] = [point_coords(pt) for pt in isocurve.points \
#                                                                if not math.isnan(pt[0]) and not math.isnan(
#                     pt[1]) and not math.isnan(pt[2])]
#     save_json(data, path, name)
#
#
# def isocurves_to_json(isocurves, path, name):
#     data = {}
#     for i, isocurve in enumerate(isocurves):
#         for pt in isocurve.points:
#             data[i] = [point_coords(pt) for pt in isocurve.points \
#                        if not math.isnan(pt[0]) and not math.isnan(pt[1]) and not math.isnan(pt[2])]
#     save_json(data, path, name)


def save_json(data, path, name):
    filename = os.path.join(path, name)
    logger.info("Saving to Json: " + filename)
    with open(filename, 'w') as f:
        f.write(json.dumps(data, indent=3, sort_keys=True))



#######################################
### convert from compas to data
#######################################

def convert_compas_Point_to_Data(compasPt):
    pt = [compasPt.x, compasPt.y, compasPt.z]
    return pt

def convert_compas_Points_list_to_Data(compasPts_list):
    pts_list = []
    for compasPt in compasPts_list:
        pt = convert_compas_Point_to_Data(compasPt)
        pts_list.append(pt)
    return pts_list

def convert_compas_vector_to_Data(compasVector):
    vec_data = [compasVector.x, compasVector.y, compasVector.z]
    return vec_data

def convert_compas_vectors_list_to_Data(compasVectors_list):
    vec_data_list = []
    for vec in compasVectors_list:
        vec_data = convert_compas_vector_to_Data(vec)
        vec_data_list.append(vec_data)
    return vec_data_list

#######################################
### convert from data to compas
#######################################

def convert_data_pts_list_to_compas_pts(pt_data_list, ROUND=False):
    """
    pt_coordinates looks like this;
        pt_coordinates = [x, y, z]
    """
    compasPt_list = []
    for pt_data in pt_data_list:
        if ROUND:
            compasPt= Point(round(pt_data[0], 3), round(pt_data[1], 3), round(pt_data[2], 3))
        else:
            compasPt= Point(pt_data[0], pt_data[1], pt_data[2])
        compasPt_list.append(compasPt)
    return compasPt_list

def convert_data_vectors_list_to_compas_vectors(vectors_data_list, ROUND=False):
    """
    vector_coordinates looks like this;
        vector_coordinates = [x, y, z]
    """
    compasVector_list = []
    for vec_data in vectors_data_list:
        if ROUND:
            compasVector = Vector(round(vec_data[0], 3), round(vec_data[1], 3), round(vec_data[2], 3))
        else:
            compasVector = Vector(vec_data[0], vec_data[1], vec_data[2])
        compasVector_list.append(compasVector)
    return compasVector_list


#######################################
### point utils
#######################################

def select_closest_pt_from_list(Point, pt_list):
    closest_point = None
    distances = [Point.distance_to_point(pt) for pt in pt_list]
    distances.sort()
    min_dis = distances[0]
    for pt in pt_list:
        distance = Point.distance_to_point(pt)
        if distance == min_dis:
            closest_point = pt
    return closest_point, min_dis









