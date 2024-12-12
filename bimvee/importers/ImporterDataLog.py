from .ImporterBase import ImporterBase
import numpy as np
import random
from .decoding.iitYarpDataLog import extract_events_from_data_log, decode_events

class ImporterDataLog(ImporterBase):

    def do_indexing(self):
        self.file_stream.seek(0)
        self.timestamps, self.ts_associated_bitstrings = extract_events_from_data_log(self.file_stream)

    def get_event_window(self, time, time_window):
        data_idx_start, data_idx_end = self.get_time_window_as_idx_range(time, time_window)
        data = self.ts_associated_bitstrings[data_idx_start:data_idx_end]
        new_dict = {'x': [],
                    'y': [],
                    'pol': [],
                    'ch': [],
                    'ts': [],
                    }
        for ts, d in zip(self.timestamps[data_idx_start:data_idx_end], data):
            pol, x, y, ch = decode_events(d)
            new_dict['x'].append(x)
            new_dict['y'].append(y)
            new_dict['pol'].append(pol)
            new_dict['ch'].append(ch)
            new_dict['ts'].append([ts] * len(pol))
        for key in new_dict:
            try:
                new_dict[key] = np.concatenate(new_dict[key])
            except ValueError:
                new_dict[key] = new_dict[key]
        return new_dict

    def get_time_window_as_idx_range(self, time, time_window):
        data_idx_start = np.searchsorted(self.timestamps, time - time_window / 2)
        data_idx_end = np.searchsorted(self.timestamps, self.timestamps[data_idx_start] + time_window)
        return data_idx_start, data_idx_end
    
    def get_dims(self):
        if not hasattr(self, 'dimX'):
            _, x, y, _ = decode_events(np.concatenate(random.sample(self.ts_associated_bitstrings, 5000)))
            self.dimX = max(x) + 1
            self.dimY = max(y) + 1
        return self.dimX, self.dimY