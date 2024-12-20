from .ImporterBase import ImporterBase
import json
import numpy as np
from scipy.interpolate import interp1d

class ImporterEyeTracking(ImporterBase):

    def _do_indexing(self):        
        self._file_stream.seek(0)
        self._data = json.load(self._file_stream)
        self._timestamps = [x['ts'] for x in self._data]

    def get_data_type(self):
        return 'eyeTracking'
    
    def get_data_at_time(self, time, time_window=None, **kwargs):
        if not kwargs.get('interpolate'):
            data_idx = self.get_idx_at_time(time)
            if np.abs(self._timestamps[data_idx] - time) > time_window:
                return None
            return self._data[data_idx]
        else:
            ids_to_interpolate = self.get_idx_at_time(time, 1)
            data_to_interpolate = self._data[ids_to_interpolate]
            out_dict = {}
            for key in data_to_interpolate[0].keys():
                val = [x[key] for x in data_to_interpolate]
                if key == 'eye_closed':
                    out_dict[key] = val[0] and val[1]
                    continue
                linear_interp = interp1d(self._timestamps[ids_to_interpolate], val, kind='linear')
                try:
                    out_dict[key] = linear_interp(time)
                except ValueError:
                    return None
            out_dict['interpolated'] = True
            return out_dict
        
    def insert_sorted(self, new_entry, timestamp):
        insert_idx = np.searchsorted(self._timestamps, timestamp)
        self._timestamps = np.insert(self._timestamps, insert_idx, timestamp)
        self._data.insert(insert_idx, new_entry)

    def set_fixed_radius(self, radius):
        if radius is None:
            return
        for entry in self._data:
            entry['eyeball_radius'] = radius
    
    def set_fixed_uv(self, u, v):
        if u is None or v is None:
            return
        for entry in self._data:
            entry['eyeball_x'] = u
            entry['eyeball_y'] = v