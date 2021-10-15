import numpy as np


def importBoundingBoxes(**kwargs):
    bboxes = np.loadtxt(kwargs.get('filePathOrName'))
    bboxes = bboxes[np.argsort(bboxes[:, 0])]
    outDict = {'ts': bboxes[:, 0],
               'minY': bboxes[:, 1],
               'minX': bboxes[:, 2],
               'maxY': bboxes[:, 3],
               'maxX': bboxes[:, 4],
               'label': bboxes[:, 5],
               }
    return outDict
