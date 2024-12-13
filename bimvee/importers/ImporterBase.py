import os

class ImporterBase(dict): # TODO remove dict inheritance
    def __init__(self, dir, file):
        self.containing_dir_name = dir
        self.full_file_path = os.path.join(dir, file)
        self.file_stream = open(self.full_file_path)
        try:
            self.file_stream.readline()
        except UnicodeDecodeError:
            self.file_stream = open(self.full_file_path, 'rb')
        self.file_stream.seek(0)
        self.timestamps = []
        self['tsOffset'] = 0
        self.do_indexing()

    def add_gt(self, gt_file):
        self.gt_file = gt_file

    def do_indexing(self):
        raise NotImplementedError('Indexing must be implemented in derived Importer class')
    
    def get_data_at_time(self, time, time_window=None):
        raise NotImplementedError('Data retrieval must be implemented in derived Importer class')

    def get_dims(self):
        raise NotImplementedError('Data dimension is only known to inerhited class')

    def get_last_ts(self):
        return self.timestamps[-1]