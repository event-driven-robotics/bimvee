import pickle

def importSkeleton(**kwargs):
    with open(kwargs.get('filePathOrName'), 'rb') as f:
        return pickle.load(f)
