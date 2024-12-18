import os
import numpy as np
import random

class ImporterBase(dict): # TODO remove dict inheritance
    def __init__(self, dir, file):
        self._containing_dir_name = dir
        self._full_file_path = os.path.join(dir, file)
        self._file_stream = open(self._full_file_path)
        try:
            self._file_stream.readline()
        except UnicodeDecodeError:
            self._file_stream = open(self._full_file_path, 'rb')
        self._file_stream.seek(0)
        self._timestamps = []
        self['tsOffset'] = 0
        self._do_indexing()

    def _do_indexing(self):
        raise NotImplementedError('Indexing must be implemented in derived Importer class')
    
    def get_data_at_time(self, time, time_window=None):
        raise NotImplementedError('Data retrieval must be implemented in derived Importer class')

    def get_dims(self):
        raise NotImplementedError('Data dimension is only known to inerhited class')

    def get_data_type(self):
        raise NotImplementedError('Data type is only known to inerhited class')

    def get_idx_at_time(self, time):
        return min(np.searchsorted(self._timestamps, time), len(self._timestamps) - 1)

    def get_last_ts(self):
        return self._timestamps[-1]
    
    def get_first_ts(self):
        return self._timestamps[0]

    def set_ts_offset(self, ts_offset):
        self._timestamps -= ts_offset
        self['tsOffset'] = ts_offset

    def __getitem__(self, key):
        return super().__getitem__(key)
    
class ImporterEventsBase(ImporterBase):
    
    def __init__(self, dir, file):
        self._bitstrings = []
        super().__init__(dir, file)

    def _do_indexing(self):
        self._file_stream.seek(0)
        self._timestamps, self._bitstrings = self._extract_events_from_data_file(self._file_stream)

    def get_data_at_time(self, time, time_window):
        data_idx_start, data_idx_end = self._get_time_window_as_idx_range(time, time_window)
        data = np.concatenate(self._bitstrings[data_idx_start:data_idx_end])
        new_dict = {'x': [],
                    'y': [],
                    'pol': [],
                    'ts': [],
                    }
        pol, x, y = self._decode_events(data)
        new_dict['x'] = x
        new_dict['y'] = y
        new_dict['pol'] = pol
        new_dict['ts'] = [self._timestamps[data_idx_start]] * len(pol)
        return new_dict
    
    def _get_time_window_as_idx_range(self, time, time_window):
        data_idx_start = self.get_idx_at_time(time - time_window / 2)
        data_idx_end = self.get_idx_at_time(self._timestamps[data_idx_start] + time_window)
        return data_idx_start, data_idx_end
    
    def get_dims(self):
        if not hasattr(self, 'dimX'):
            _, x, y = self._decode_events(np.concatenate(random.sample(self._bitstrings, 5000)))
            self._dimX = max(x) + 1
            self._dimY = max(y) + 1
        return self._dimX, self._dimY
    
    def get_data_type(self):
        return 'dvs'
    
    @staticmethod
    def _decode_events(bitstring_array):
        raise NotImplementedError("Event decoding function must be implemented in child class")

    @staticmethod
    def _extract_events_from_data_file(file_stream):
        raise NotImplementedError("Event extraction from file function must be implemented in child class")
