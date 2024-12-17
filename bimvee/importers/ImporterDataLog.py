from .ImporterBase import ImporterEventsBase
import numpy as np
from .decoding.iitYarpDataLog import extract_events_from_data_file, decode_events

class ImporterDataLog(ImporterEventsBase):
    @staticmethod
    def decode_events(bitstring_array):
        return decode_events(bitstring_array)

    @staticmethod
    def extract_events_from_data_file(file_stream):
        return extract_events_from_data_file(file_stream)
