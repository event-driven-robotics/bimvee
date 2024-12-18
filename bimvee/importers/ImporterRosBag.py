from .ImporterFrames import ImporterFrames
from ..importRosbag.importRosbag.importRosbag import *
import os

def generateRosBagImporters(dir, f):
    importers = {}
    topic_dict = importRosbag(os.path.join(dir, f), listTopics=True)
    for x in topic_dict:
        if topic_dict[x]['type'] == 'sensor_msgs_Image':
            importer = ImporterRosBagFrames(dir, f, x)
            importers[importer.get_data_type()] = importer
    return importers
class ImporterRosBagFrames(ImporterFrames):
    def __init__(self, dir, file, topic):
        self._topic = topic
        super().__init__(dir, file)

    def _do_indexing(self):
        data = importRosbag(self._full_file_path, importTopics=[self._topic])
        self._timestamps = data[self._topic]['ts']
        self._images = data[self._topic]['frames']

    def get_data_at_time(self, time, time_window=None):
        return self._images[self.get_idx_at_time(time)]