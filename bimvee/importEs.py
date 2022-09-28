import loris
import numpy as np


def importEs(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    data = loris.read_file(filePathOrName)
    ts = []
    xs = []
    ys = []
    ps = []
    for t, x, y, _, p in data['events']:
        ts.append(t)
        xs.append(x)
        ys.append(y)
        ps.append(p)
    timestamps = np.array(ts, dtype=float)
    x = np.array(xs)
    y = np.array(ys)
    pol = np.array(ps)
    tsOffset = timestamps[0]
    timestamps -= tsOffset
    timestamps /= 1e6
    out_dict = {'info': {'tsOffset': tsOffset,
                         'filePathOrName': filePathOrName,
                         'fileFormat': 'es'},
                'data': {
                    'ch0': {
                        'dvs': {
                            'ts': timestamps,
                            'x': x,
                            'y': y,
                            'pol': pol
                        }}}}
    return out_dict
