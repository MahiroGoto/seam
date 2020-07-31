import numpy as np
import stratum.utils.utils as utils
import open3d as o3d
import point_cloud_utils as pcu

###
input_path = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/04_describe_printedShape/input/"
output_path = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/04_describe_printedShape/output/"
dataname = "pts_list.json"
###

point_cloud = utils.load_from_Json(input_path, dataname)
print(point_cloud)

pcu.