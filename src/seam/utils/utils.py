import os
import json

import logging

logger = logging.getLogger('logger')

#######################################
### Json utils
#######################################

### load Json file
def load_from_Json(path, name):
    filename = os.path.join(path, name)
    with open(filename, 'r') as f:
        data = json.load(f)
    logger.info('Loaded Json: ' + filename)
    return data


def isocurves_segments_to_json(segments, path, name):
    data = {}
    for i, segment in enumerate(segments):
        data['Segment_' + str(i)] = {}
        for j, isocurve in enumerate(segment.isocurves):
            data['Segment_' + str(i)]['Isocurve_' + str(j)] = [point_coords(pt) for pt in isocurve.points \
                                                               if not math.isnan(pt[0]) and not math.isnan(
                    pt[1]) and not math.isnan(pt[2])]
    save_json(data, path, name)


def isocurves_to_json(isocurves, path, name):
    data = {}
    for i, isocurve in enumerate(isocurves):
        for pt in isocurve.points:
            data[i] = [point_coords(pt) for pt in isocurve.points \
                       if not math.isnan(pt[0]) and not math.isnan(pt[1]) and not math.isnan(pt[2])]
    save_json(data, path, name)


def save_json(data, path, name):
    filename = os.path.join(path, name)
    logger.info("Saving to Json: " + filename)
    with open(filename, 'w') as f:
        f.write(json.dumps(data, indent=3, sort_keys=True))
