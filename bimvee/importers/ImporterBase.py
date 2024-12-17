import os
import numpy as np
import random

class ImporterBase(dict): # TODO remove dict inheritance
    def __init__(self, dir, file):
        self.containing_dir_name = dir
        self.full_file_path = os.path.join(dir, file)
        self.file_stream = open(self.full_file_path)
        try:
            self.file_stream.readline()
        except UnicodeDecodeError:
            self.file_stream = open(self.full_file_path, 'rb')
        self.file_stream.seek(0)
        self.timestamps = []
        self['tsOffset'] = 0
        self.do_indexing()

    def do_indexing(self):
        raise NotImplementedError('Indexing must be implemented in derived Importer class')
    
    def get_data_at_time(self, time, time_window=None):
        raise NotImplementedError('Data retrieval must be implemented in derived Importer class')

    def get_dims(self):
        raise NotImplementedError('Data dimension is only known to inerhited class')

    def get_data_type(self):
        raise NotImplementedError('Data type is only known to inerhited class')

    def get_idx_at_time(self, time):
        return min(np.searchsorted(self.timestamps, time), len(self.timestamps) - 1)

    def get_last_ts(self):
        return self.timestamps[-1]
    
    def get_first_ts(self):
        return self.timestamps[0]

    def set_ts_offset(self, ts_offset):
        self.timestamps -= ts_offset
        self['tsOffset'] = ts_offset

    def __getitem__(self, key):
        return super().__getitem__(key)
    
class ImporterEventsBase(ImporterBase):
    
    def __init__(self, dir, file):
        self.bitstrings = []
        super().__init__(dir, file)

    def do_indexing(self):
        self.file_stream.seek(0)
        self.timestamps, self.bitstrings = self.extract_events_from_data_file(self.file_stream)

    def get_data_at_time(self, time, time_window):
        data_idx_start, data_idx_end = self._get_time_window_as_idx_range(time, time_window)
        data = np.concatenate(self.bitstrings[data_idx_start:data_idx_end])
        new_dict = {'x': [],
                    'y': [],
                    'pol': [],
                    'ts': [],
                    }
        pol, x, y = self.decode_events(data)
        new_dict['x'] = x
        new_dict['y'] = y
        new_dict['pol'] = pol
        new_dict['ts'] = [self.timestamps[data_idx_start]] * len(pol)
        return new_dict
    
    def _get_time_window_as_idx_range(self, time, time_window):
        data_idx_start = self.get_idx_at_time(time - time_window / 2)
        data_idx_end = self.get_idx_at_time(self.timestamps[data_idx_start] + time_window)
        return data_idx_start, data_idx_end
    
    def get_dims(self):
        if not hasattr(self, 'dimX'):
            _, x, y = self.decode_events(np.concatenate(random.sample(self.bitstrings, 5000)))
            self.dimX = max(x) + 1
            self.dimY = max(y) + 1
        return self.dimX, self.dimY
    
    def get_data_type(self):
        return 'dvs'
    
    @staticmethod
    def decode_events(bitstring_array):
        raise NotImplementedError("Event decoding function must be implemented in child class")

    @staticmethod
    def extract_events_from_data_file(file_stream):
        raise NotImplementedError("Event extraction from file function must be implemented in child class")
