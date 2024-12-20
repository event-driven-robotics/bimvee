from .ImporterBase import ImporterBase
import numpy as np
import imageio
import os
class ImporterFrames(ImporterBase):
    
    def _do_indexing(self):
        self._file_stream.seek(0)
        self._timestamps = np.loadtxt(self._file_stream)
        self._image_list = sorted([x for x in os.listdir(self._containing_dir_name) if os.path.splitext(x)[-1] in ['.jpg', '.jpeg', '.png']])

    def get_data_at_time(self, time, time_window=None, **kwargs):
        data_idx = self.get_idx_at_time(time)
        return imageio.imread(os.path.join(self._containing_dir_name, self._image_list[data_idx]))

    def get_dims(self):
        if not hasattr(self, '_dimX'):
            example_frame = self.get_data_at_time(0)
            self._dimX = example_frame.shape[1]
            self._dimY = example_frame.shape[0]
        return self._dimX, self._dimY
    
    def get_data_type(self):
        return 'frame'