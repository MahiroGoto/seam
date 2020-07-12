import os
from compas.datastructures import Mesh
from compas.geometry import Polyline

import stratum.utils.utils as utils
from seam.src.seam.construct_geometry.construct_mesh_from_geometry import mesh_from_two_rails

###############################################################################
import logging

logger = logging.getLogger("logger")
logging.basicConfig(format='%(levelname)s - %(message)s', level=logging.INFO)
###############################################################################

### setting
OBJ_INPUT_NAME = 'shape01'
DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/3_Prototype/10_raddershape/data/'
###

constructmesh = True

if __name__ == "__main__":

     if constructmesh:
         mesh_from_two_rails(DATA_PATH, name_rail00="pts_crv_01.json", name_rail01="pts_crv_02.json")
