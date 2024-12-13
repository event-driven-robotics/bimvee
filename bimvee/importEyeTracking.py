import json
import numpy as np

def importEyeTracking(json_file):
    with open(json_file) as f:
        eyes = json.load(f)
    return {k: np.array([x[k] for x in eyes]) for k in eyes[0]}
