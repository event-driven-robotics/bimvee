from .ImporterBase import ImporterBase
import json
import numpy as np
from scipy.interpolate import interp1d

class ImporterBoundingBoxes(ImporterBase):

    def _do_indexing(self):        
        self._file_stream.seek(0)
        bboxes = np.loadtxt(self._file_stream)
        if len(bboxes.shape) == 1:
            bboxes = np.expand_dims(bboxes,0)
        bboxes = bboxes[np.argsort(bboxes[:, 0])]
        self._data = {'minY': bboxes[:, 1],
                      'minX': bboxes[:, 2],
                      'maxY': bboxes[:, 3],
                      'maxX': bboxes[:, 4],
                      'label': bboxes[:, 5],
                      }
        self._timestamps = bboxes[:, 0]

    def get_data_type(self):
        return 'boxes'
    
    def get_data_at_time(self, time, time_window=None, **kwargs):
        if not kwargs.get('interpolate'):
            data_idx = self.get_idx_at_time(time) 
            if data_idx < 0:
                return None
            if np.abs(self._timestamps[data_idx] - time) > time_window:
                return None
            return self._data[data_idx]
        else:
            ids_to_interpolate = self.get_idx_at_time(time, 1)
            data_to_interpolate = self._data[ids_to_interpolate]
            out_dict = {}
            for key in data_to_interpolate[0].keys():
                val = [x[key] for x in data_to_interpolate]
                linear_interp = interp1d(self._timestamps[ids_to_interpolate], val, kind='linear')
                try:
                    out_dict[key] = linear_interp(time)
                except ValueError:
                    return None
            out_dict['interpolated'] = True
            return out_dict
        