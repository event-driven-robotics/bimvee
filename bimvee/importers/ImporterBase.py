import os
import numpy as np
from copy import deepcopy


class ImporterBase:
    def __init__(self, dir=None, file=None):
        if dir is None or file is None:
            self._timestamps = []
            self._data = []
        else:
            self._containing_dir_name = dir
            self._full_file_path = os.path.join(dir, file)
            self._file_stream = open(self._full_file_path)
            try:
                self._file_stream.readline()
            except UnicodeDecodeError:
                self._file_stream = open(self._full_file_path, 'rb')
            self._file_stream.seek(0)
            self._timestamps = []
            self._ts_offset = 0
            self._do_indexing()

    def _do_indexing(self):
        raise NotImplementedError('Indexing must be implemented in derived Importer class')

    def get_data_at_time(self, time, time_window=None, **kwargs):
        raise NotImplementedError('Data retrieval must be implemented in derived Importer class')

    def get_dims(self):
        raise NotImplementedError('Data dimension is only known to inerhited class')

    def get_data_type(self):
        raise NotImplementedError('Data type is only known to inerhited class')

    def get_time_of_next_data_point(self, time, backward=False):
        try:
            idx = self.get_idx_at_time(time)
            if backward:
                if self._timestamps[idx] < time:
                    return self._timestamps[idx]
                else:
                    return self._timestamps[(idx - 1)]
            else:
                if self._timestamps[idx] > time:
                    return self._timestamps[idx]
                else:
                    return self._timestamps[(idx + 1) % len(self)]
        except IndexError:
            return time

    def get_idx_at_time(self, time, ids_around_time=0):
        idx = np.searchsorted(self._timestamps, time)
        if ids_around_time == 0:
            try:
                if abs(self._timestamps[idx - 1] - time) < abs(self._timestamps[idx] - time):
                    idx -= 1
            except IndexError:
                pass
            return min(idx, len(self._timestamps) - 1)
        else:
            return slice(max(0, idx - ids_around_time), min(len(self._timestamps), idx + ids_around_time))

    def get_last_ts(self):
        return self._timestamps[-1]

    def get_first_ts(self):
        return self._timestamps[0]

    @property
    def ts_offset(self):
        return self._ts_offset

    @ts_offset.setter
    def ts_offset(self, ts_offset):
        self._timestamps -= ts_offset
        self._ts_offset = ts_offset

    def get_full_data_as_dict(self, **kwargs):
        interpolate = kwargs.get('interpolate', False)
        if interpolate:
            timestamps = np.arange(self.get_first_ts(), self.get_last_ts(), kwargs.get('time_step', 0.01))
        else:
            timestamps = self._timestamps
        data_list = [self.get_data_at_time(ts, 0, **kwargs) for ts in timestamps]
        # merge list of dicts in one dict
        # TODO handle case with empty dict
        return {k: [d[k] for d in data_list] for k in data_list[0].keys()}

    def __len__(self):
        return len(self._timestamps)


class EditableImporterBase(ImporterBase):

    def __init__(self, dir=None, file=None):
        super().__init__(dir, file)
        self._timestamps_histories = []
        self._data_histories = []
        self._history_idx = 0
        self.update_history()

    def delete_by_time(self, time):
        self.delete_by_index(self.get_idx_at_time(time))  # TODO check if time is close enough to timestamp

    def delete_by_index(self, idx):
        self._data.pop(idx)
        self._timestamps = np.delete(self._timestamps, idx)
        self.update_history()

    def undo(self):
        self._history_idx = max(0, self._history_idx - 1)
        self.recover_history(self._history_idx)

    def redo(self):
        self._history_idx = min(len(self._timestamps_histories) - 1, self._history_idx + 1)
        self.recover_history(self._history_idx)

    def update_history(self):
        self._timestamps_histories = self._timestamps_histories[:self._history_idx + 1]
        self._data_histories = self._data_histories[:self._history_idx + 1]
        self._timestamps_histories.append(deepcopy(self._timestamps))
        self._data_histories.append(deepcopy(self._data))
        self._history_idx = len(self._timestamps_histories) - 1

    def recover_history(self, idx):
        self._timestamps = deepcopy(self._timestamps_histories[idx])
        self._data = deepcopy(self._data_histories[idx])

    def insert_sorted(self, new_entry, timestamp):
        insert_idx = np.searchsorted(self._timestamps, timestamp)
        self._timestamps = np.insert(self._timestamps, insert_idx, timestamp)
        self._data.insert(insert_idx, new_entry)

class ImporterEventsBase(ImporterBase):

    def __init__(self, dir, file):
        self._bitstrings = []
        super().__init__(dir, file)

    def _do_indexing(self):
        self._file_stream.seek(0)
        self._timestamps, self._bitstrings = self._extract_events_from_data_file(self._file_stream)

    def get_data_at_time(self, time, time_window, **kwargs):
        data_idx_start, data_idx_end = self._get_time_window_as_idx_range(time, time_window)
        data = self._bitstrings[data_idx_start:data_idx_end]
        timestamps = self._timestamps[data_idx_start:data_idx_end]
        new_dict = {}
        if not len(timestamps):
            new_dict['x'] = []
            new_dict['y'] = []
            new_dict['pol'] = []
            new_dict['ts'] = []
            return new_dict
        pol, x, y, ts = self._decode_events(data, timestamps)
        new_dict['x'] = x
        new_dict['y'] = y
        new_dict['pol'] = pol
        new_dict['ts'] = ts
        return new_dict

    def _get_time_window_as_idx_range(self, time, time_window):
        data_idx_start = self.get_idx_at_time(time - time_window / 2)
        data_idx_end = self.get_idx_at_time(self._timestamps[data_idx_start] + time_window)
        if data_idx_end == data_idx_start:
            data_idx_end += 1
        return data_idx_start, data_idx_end


    def get_full_data_as_dict(self):
        out_dict = super().get_full_data_as_dict()
        return {k: np.concatenate(out_dict[k]) for k in out_dict.keys()}
    
    def get_dims(self):
        if not hasattr(self, '_dimX'):
            self._dimX = 0
            self._dimY = 0
            random_indices = np.random.choice(np.arange(len(self._timestamps)), 50, replace=False)
            for i in random_indices:
                events_dict = self.get_data_at_time(self._timestamps[i], 0.3)
                self._dimX = max(self._dimX, max(events_dict['x']) + 1)
                self._dimY = max(self._dimY, max(events_dict['y']) + 1)
        return self._dimX, self._dimY

    def get_data_type(self):
        return 'dvs'

    @staticmethod
    def _decode_events(bitstring_array, timestamps):
        raise NotImplementedError("Event decoding function must be implemented in child class")

    @staticmethod
    def _extract_events_from_data_file(file_stream):
        raise NotImplementedError("Event extraction from file function must be implemented in child class")
