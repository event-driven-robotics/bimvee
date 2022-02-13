import json
import numpy as np
import re
import os

def importSkeletonDataLog(filePath, keys):
    pattern = re.compile('\d* (\d*.\d*) SKLT \((.*)\)')
    with open(filePath) as f:
        content = f.readlines()
    data_dict = {k: [] for k in keys}
    timestamps = []
    for line in content:
        ts, data = pattern.findall(line)[0]
        data = np.array(data.split(' ')).astype(int).reshape(-1, 2)
        for d, label in zip(data, data_dict):
            data_dict[label].append(d)
        timestamps.append(ts)
    data_dict['ts'] = np.array(timestamps).astype(float)
    for d in data_dict:
        data_dict[d] = np.array(data_dict[d])
    return data_dict

def importSkeleton(**kwargs):
    with open(kwargs.get('filePathOrName'), 'r') as f:
        data_dict = json.load(f)
    return importSkeletonDataLog(os.path.join(os.path.dirname(kwargs['filePathOrName']), data_dict['file']), data_dict['labels'])
