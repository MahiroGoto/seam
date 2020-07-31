import numpy as np

import seam.utils.utils as utils

import logging


DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/000_Exploration/07_branch_data_structure/data/'


#######################################################################################################

vertices = utils.load_from_Json(DATA_PATH, "data_pts.json")
print(vertices)