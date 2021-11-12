import json
import numpy as np

def importSkeleton(**kwargs):
    with open(kwargs.get('filePathOrName'), 'r') as f:
        data = json.load(f)
        for key in data:
            data[key] = np.array(data[key])
    return data