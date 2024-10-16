class ImporterBase:
    def __init__(self, dir, file):
        print(dir, file)

    def add_gt(self, gt_file):
        print(f'Adding gt {gt_file}')
