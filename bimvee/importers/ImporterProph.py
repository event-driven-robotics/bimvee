from .ImporterBase import ImporterEventsBase
from .decoding.prophesee import extract_events_from_data_file, decode_events

class ImporterProph(ImporterEventsBase):
    @staticmethod
    def _decode_events(bitstring_array, timestamps):
        return decode_events(bitstring_array, timestamps)

    @staticmethod
    def _extract_events_from_data_file(file_stream):
        return extract_events_from_data_file(file_stream)
    
