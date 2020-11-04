import os
from compas.datastructures import Mesh
from compas.geometry import Polyline

import stratum.utils.utils as utils
from stratum.isocurves.marching_triangles import MarchingTriangles, find_desired_number_of_isocurves
from stratum.isocurves.compound_target import CompoundTarget
from stratum.isocurves.clustering import VertexClustering

from stratum.printpaths.path_collection import PathCollection
from stratum.printpaths.path import Path
from stratum.printpaths.boundary import Boundary

from stratum.fabrication.fabrication_sequence import FabricationSequence
from stratum.fabrication.robot_printer import RobotPrinter

# from stratum.region_split.scalar_evaluation import ScalarEvaluation
# import stratum.region_split.topological_sort as topo_sort
# from stratum.region_split.region_split import RegionSplitManager
#
######################## Logging
import logging

logger = logging.getLogger('logger')
logging.basicConfig(format='%(levelname)s - %(message)s', level=logging.INFO)  # , filename='/temp/all_out.log'
########################

########################
# DATA_PATH = 'G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/' \
#             '3_Prototype/23_overlapping_01_correct_order/data/'

PATH = "G:/.shortcut-targets-by-id/19j2p-s21q1pYuoGXuk11Bu934R1tS8MU/MAS Thesis 2020/" \
       "08_after the project/02_four branching connection/"
PRINT_PIECE_FOLDER = "print_03/"

DATA_PATH = PATH + PRINT_PIECE_FOLDER
COMMANDS_PATH = DATA_PATH + '/commands/'

OBJ_INPUT_NAME = 'mesh.obj'
########################

generate_paths = True
fabrication_sequence = True

### --- Load initial_mesh
mesh = Mesh.from_obj(os.path.join(DATA_PATH, OBJ_INPUT_NAME))
print("Vertices : %d , Faces : %d " % (len(list(mesh.vertices())), len(list(mesh.faces()))))

### --- Load boundaries and update vertex attributes
vertices_start_LOW = utils.load_from_json(DATA_PATH, "boundaryLOW.json")
mesh.update_default_vertex_attributes({'boundary': 0})
for vkey, data in mesh.vertices(data=True):
    if vkey in vertices_start_LOW:
        data['boundary'] = 1

if generate_paths:

    ### --- Load isocurves
    isocurves = utils.load_from_json(DATA_PATH, "gh_isocurves.json")
    paths_index = 0
    curves_dict = isocurves['Segment_' + str(0)]

    ### --- find starting boundary
    vertex_clustering = VertexClustering(mesh, 'boundary', 1, DATA_PATH)
    boundary_pts = vertex_clustering.get_flattened_list_of_all_vertices()
    boundary = Boundary(mesh, boundary_pts)

    ### --- Create paths
    paths = []
    for j in range(len(curves_dict)):
        path_points = curves_dict['Isocurve_' + str(j)]
        path_points.append(path_points[0])  # Close curve. Attention! Only works for closed curves!
        paths.append(Path(Polyline(path_points), mesh))

    path_collection = PathCollection(paths=paths, lower_boundary=boundary, mesh=mesh)
    path_collection.generate(first_layer_height=1.0)
    utils.save_json(path_collection.to_dict(), DATA_PATH, "paths_collection_" + str(paths_index) + ".json")

    utils.interrupt()

if fabrication_sequence:
    print('')
    filenames = utils.get_files_with_name('paths_collection_', '.json', DATA_PATH)
    robot_printer = RobotPrinter('UR5')
    try:
        env_pts = utils.load_from_json(DATA_PATH, 'environment_collision_points.json')
    except:
        logger.info('Did not find environment_collision_points.json')
        env_pts = None

    for i, filename in enumerate(filenames):
        p_collection = PathCollection.from_json(DATA_PATH, filename)

        fabrication_sequence = FabricationSequence(p_collection.paths, DATA_PATH,
                                                   TOOL_COLLISION_PTS='tool_pts_for_collision_check.json')

        fabrication_sequence.generate_commands(robotprinter=robot_printer, apply_collision_control=True,
                                               calculate_ik=False, environment_collision_points=env_pts)

        fabrication_sequence.save_fabrication_commands_to_Json(path=COMMANDS_PATH,
                                                               name="commands_" + str(i) + ".json")
        fabrication_sequence.command_frames_to_json(path=DATA_PATH, name="print_frames_" + str(i) + ".json")
        utils.save_json(fabrication_sequence.opt_angles, DATA_PATH, "opt_angles_" + str(i) + ".json")

    # utils.interrupt()
