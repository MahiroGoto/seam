import os
import json

import logging

logger = logging.getLogger('logger')

#####################
### Json utils
#####################

### load Json file
def load_from_Json(path, name):
    filename = os.path.join(path, name)
    with open(filename, 'r') as f:
        data = json.load(f)
    logger.info('Loaded Json: ' + filename)
    return data