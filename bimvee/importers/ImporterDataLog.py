from .ImporterBase import ImporterEventsBase
import numpy as np
from .decoding.iitYarpDataLog import extract_events_from_data_file, extract_events_from_data_file_v1, decode_events_v1


        
class ImporterDataLog(ImporterEventsBase):
    
    def _decode_events(self, bitstring_array, timestamps):
        if self.is_v1:
            return decode_events_v1(bitstring_array, timestamps)
        return decode_events_v1(bitstring_array, timestamps)

    def _extract_events_from_data_file(self, file_stream):
        self.is_v1 = not 'b' in file_stream.mode
        if self.is_v1:
            return extract_events_from_data_file_v1(file_stream)
        return extract_events_from_data_file(file_stream)
