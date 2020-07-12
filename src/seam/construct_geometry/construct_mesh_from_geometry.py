from compas.datastructures import Mesh
from compas.geometry import Polyline

import stratum.utils.utils as utils


###############################################################################
import logging

logger = logging.getLogger("logger")
logging.basicConfig(format='%(levelname)s - %(message)s', level=logging.INFO)
###############################################################################


def mesh_from_two_rails(DATA_PATH, name_rail00, name_rail01):
    ### load points list on both curves
    pts_crv_01 = utils.load_from_Json(DATA_PATH, name_rail00)
    pts_crv_02 = utils.load_from_Json(DATA_PATH, name_rail01)

    mesh = Mesh()
    vertex_keys = []
    for i in range(len(pts_crv_01)):

        pt_01 = pts_crv_01[i]
        pt_02 = pts_crv_02[i]

        key00 = mesh.add_vertex(x=pt_01[0], y=pt_01[1], z=pt_01[2])
        key01 = mesh.add_vertex(x=pt_02[0], y=pt_02[1], z=pt_02[2])

        vertex_keys.append(key00)
        vertex_keys.append(key01)

    faces = []
    for j in range(len(vertex_keys)):
        if j+2 > vertex_keys[-1]:
            break
        face = [j, j+1, j+2]
        faces.append(face)

    flip = False
    for keys in faces:
        if flip:
            keylist = [ keys[k] for k in range(len(keys)) ]
            keylist.reverse()
            flip = False
        elif not flip:
            keylist = [ keys[k] for k in range(len(keys)) ]
            flip = True

        mesh.add_face( keylist )

    mesh.to_obj(DATA_PATH + "mesh_radder.obj", precision=False)
    logger.info("saving to obj: " + "mesh_radder.obj")

















