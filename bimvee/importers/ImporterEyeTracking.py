from .ImporterBase import ImporterBase
import json
import numpy as np

class ImporterEyeTracking(ImporterBase):

    def do_indexing(self):        
        self.file_stream.seek(0)
        eyes = json.load(self.file_stream)
        self.update({k: np.array([x[k] for x in eyes]) for k in eyes[0]})
        self.timestamps = self['ts']

    def get_data_type(self):
        return 'eyeTracking'