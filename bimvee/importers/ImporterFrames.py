from .ImporterBase import ImporterBase
import numpy as np
import imageio
import os
class ImporterFrames(ImporterBase):
    
    def do_indexing(self):
        self.file_stream.seek(0)
        self.timestamps = np.loadtxt(self.file_stream)
        self.image_list = sorted([x for x in os.listdir(self.containing_dir_name) if os.path.splitext(x)[-1] in ['.jpg', '.jpeg', '.png']])

    def get_data_at_time(self, time, time_window=None):
        data_idx = np.searchsorted(self.timestamps, time)
        return imageio.imread(os.path.join(self.containing_dir_name, self.image_list[data_idx]))

    def get_dims(self):
        if not hasattr(self, 'dimX'):
            example_frame = imageio.imread(os.path.join(self.containing_dir_name, self.image_list[0]))
            self.dimX = example_frame.shape[1]
            self.dimY = example_frame.shape[0]
        return self.dimX, self.dimY
    
    def get_data_type(self):
        return 'frame'