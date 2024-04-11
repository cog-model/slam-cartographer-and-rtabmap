import os.path as osp
import glob


class SavePathsGenerator:
    def __init__(self, save_folder, extension, continue_saving=False,
            start_from=0):
        if extension[0] != '.':
            raise RuntimeError("SavePathsGenerator: use '.' before extension.")

        self.save_folder = osp.expanduser(save_folder)
        self.extension = extension
        self.continue_saving = continue_saving
        self.start_from = start_from

        if self.continue_saving:
            files = glob.glob(f"{self.save_folder}/????{self.extension}")
            max_num = -1
            for file in files:
                num = osp.splitext(osp.basename(file))[0]
                if num.isdigit():
                    num = int(num)
                    max_num = max(max_num, num)
            self.counter = max_num + 1
        else:
            self.counter = self.start_from

    def __iter__(self):
        return self

    def __next__(self):
        next_save_path = f'{self.counter:04}{self.extension}'
        next_save_path = osp.join(self.save_folder, next_save_path)
        self.counter += 1
        return next_save_path