from .ImporterBase import ImporterEventsBase
from .decoding.prophesee import extract_events_from_data_file, decode_events

class ImporterProph(ImporterEventsBase):
    @staticmethod
    def decode_events(bitstring_array):
        return decode_events(bitstring_array)

    @staticmethod
    def extract_events_from_data_file(file_stream):
        return extract_events_from_data_file(file_stream)
    
    def get_data_at_time(self, time, time_window):
        data_idx_start, data_idx_end = self._get_time_window_as_idx_range(time, time_window)
        data = self.bitstrings[data_idx_start:data_idx_end]
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