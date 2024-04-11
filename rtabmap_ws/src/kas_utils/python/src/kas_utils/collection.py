import os
import os.path as osp
from time import time, strftime, localtime
from threading import Lock


class Collection:
    def __init__(self, name, print_results=None, header_to_str=None, observation_to_str=None):
        self.name = name
        self.print_results = print_results
        self.header_to_str = header_to_str
        self.observation_to_str = observation_to_str

        self._mutex = Lock()
        self._observations = list()

        self._construction_time = time()
        self._abbreviation = "COL"

    def add(self, observation):
        with self._mutex:
            self._add_under_lock(observation)

    def _add_under_lock(self, observation):
        assert self._mutex.locked()
        self._observations.append(observation)

    def __del__(self):
        with self._mutex:
            if self.print_results:
                self.print_results(self.name, self._observations)

            if self.observation_to_str:
                out_log_file = self._get_out_log_file()
                if out_log_file:
                    out_log_file = osp.expanduser(out_log_file)
                    with open(out_log_file, 'w') as f:
                        first_line = True
                        if self.header_to_str:
                            f.write(self.header_to_str())
                            first_line = False
                        for observation in self._observations:
                            if not first_line:
                                f.write("\n")
                            f.write(self.observation_to_str(observation))
                            first_line = False

    def _get_out_log_file(self):
        out_log_file = os.getenv(f"{self.name}_{self._abbreviation}_LOG_FILE")
        if out_log_file:
            return out_log_file
        else:
            out_log_folder = os.getenv(f"{self.name}_{self._abbreviation}_LOG_FOLDER")
            if out_log_folder:
                if not osp.isdir(out_log_folder):
                    try:
                        os.makedirs(out_log_folder)
                    except OSError:
                        print(f"Could not create directory {out_log_folder}.")
                        return None
                creation_time_str = strftime('%Y-%m-%d_%H.%M.%S', localtime(self._construction_time))
                out_log_file = osp.join(out_log_folder, f"{creation_time_str}_{self.name}.txt")
                return out_log_file
            else:
                return None


class StampedCollection(Collection):
    def __init__(name, print_results=None, header_to_str=None, observation_to_str=None):
        super().__init__(name, print_results=print_results,
            header_to_str=header_to_str, observation_to_str=observation_to_str)
    
    def add(self, observation):
        stamp = time()
        super().add((stamp, observation))
