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
        self.topic = topic
        super().__init__(dir, file)

    def do_indexing(self):
        data = importRosbag(self.full_file_path, importTopics=[self.topic])
        self.timestamps = data[self.topic]['ts']
        self.images = data[self.topic]['frames']

    def get_data_at_time(self, time, time_window=None):
        return self.images[self.get_idx_at_time(time)]