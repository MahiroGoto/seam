import os
import seam.src.seam.utils.utils as utils

###############################################################################
import logging

logger = logging.getLogger("logger")
logging.basicConfig(format='%(levelname)s - %(message)s', level=logging.INFO)
###############################################################################

OBJ_INPUT_NAME = 'shape01'
DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/10_raddershape/data/'

###############################################################################

construct_mesh = True

# if __name__ = "__main__":
#
#     if construct_mesh:
#         ### load points list on both curves
#         pts_crv_01 = 