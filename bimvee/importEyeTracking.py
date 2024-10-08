import json
import numpy as np

def importEyeTracking(**kwargs):
    with open(kwargs.get('filePathOrName')) as f:
        eyes = json.load(f)
    return {k: np.array([x[k] for x in eyes]) for k in eyes[0]}
