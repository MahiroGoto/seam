import stratum.utils.utils as utils

import math

from compas.geometry import Vector, scale_vector, add_vectors, Point, curve, distance_point_point, angle_vectors
from compas.geometry import distance_point_point_sqrd


import logging

def distance_from_targetPts_to_cloudPts(targetPts, cloudPts):
    if len(targetPts) == len(cloudPts):
        distance_out = []
        for i in range(len(cloudPts)):
            distance = distance_point_point(targetPts[i], cloudPts[i])
            distance_out.append(distance)
    return distance_out

def measure_angle_targetPts_cloudPts(vector01, vector02, smaller=True):

    angle = angle_vectors(vector01, vector02)




