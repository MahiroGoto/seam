parameters = {
    ### --- isocurves generation
    # 'isocurves_simplification_threshold': 0.8,
    'layer_path_pt_span': 3.8,

    'min_layer_height': 0.8,
    'max_layer_height': 1.6,

    # 'min_layer_height': 10,
    # 'max_layer_height': 15,

}


def get_param(key):
    return parameters[key]



## parameter ##
"""
parameters = {
    ### --- region splitting
    'T_PARAMS_recompute_process': True,
    't_param_buffer': 0.005,
    't_search_resolution': 100,

    'CUT_mesh_process': True,
    'SEPARATE_meshes_process': True,

    'welding_scale_factor_region_split': 1.0,

    ### --- isocurves generation
    'isocurves_simplification_threshold': 0.8,
    'avg_layer_height': 1.7,

    'min_print_thickness': 0.9,
    'max_print_thickness': 1.7,

    ### --- print paths
    'regularizer_printpoint_up_vector': 0.3,

}

def get_param(key):
    return parameters[key]

"""



## ur_script ##
"""
# Constants
MAX_ACCEL = 1.5
MAX_VELOCITY = 2.3
extruder_IO = 1
# tool_angle_axis = [8.51, 48.25, 162.83, 0.0, 0.0, 0.0]
# tool_angle_axis = [14.61, 43.41, 161.30, 0.0, 0.0, 0.0]
tool_angle_axis = [8.26, 42.74, 161.58, 0.0, 0.0, 0.0]
# tool_angle_axis = [15.53, 45.71, 161.95, 0.0, 0.0, 0.0]
# ===============================================================
velocity_mult = 0.16
# xoffset = -120
# yoffset = -30
# zoffset = -6.3
xoffset = 200
yoffset = -500
zoffset = -483


"""