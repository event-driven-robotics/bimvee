from .ImporterBase import ImporterEventsBase
import numpy as np
from .decoding.iitYarpDataLog import extract_events_from_data_file, decode_events

class ImporterDataLog(ImporterEventsBase):
    @staticmethod
    def _decode_events(bitstring_array, timestamps):
        return decode_events(bitstring_array, timestamps)

    @staticmethod
    def _extract_events_from_data_file(file_stream):
        return extract_events_from_data_file(file_stream)
