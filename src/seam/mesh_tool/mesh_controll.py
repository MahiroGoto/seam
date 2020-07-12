from compas.datastructures import Mesh

import numpy as np
import open3d as o3d

import stratum.utils.utils as utils

###
import logging
logger = logging.getLogger('logger')
###



def add_vertex_from_PtsList(mesh, points_list):
    mesh = Mesh()

def create_mesh_from_layer_path_pts(output_path, point_cloud):

    # point_cloud = np.loadtxt(input_path + dataname, skiprows=1)

    pt_cld = o3d.geometry.PointCloud()
    pt_cld.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
    pt_cld.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:6] / 255)
    pt_cld.normals = o3d.utility.Vector3dVector(point_cloud[:, 6:9])

    o3d.visualization.draw_geometries([pt_cld])

    ### processing point cloud data ###
    distances = pt_cld.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist

    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pt_cld, o3d.utility.DoubleVector([radius, radius * 2]))

    dec_mesh = bpa_mesh.simplify_quadric_decimation(10000)

    o3d.io.write_triangle_mesh(output_path + "bpa_mesh.ply", dec_mesh)

    logger.info("save mesh:" + "bpa_mesh.ply" )










