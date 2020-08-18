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

    'min_layer_height': 0.8,
    'max_layer_height': 1.9,

    ### --- print paths
    'regularizer_printpoint_up_vector': 0.5,

}


def get_param(key):
    return parameters[key]
