import threading
from threading import Lock
from time import time


class TimeMeasurer:
    def __init__(self, name, end='', dump_on_deletion=True, skip_first_n=0):
        self.name = name
        self.end = end
        self.dump_on_deletion = dump_on_deletion
        self.skip_n = skip_first_n

        self.mutex = Lock()
        self.start_times = dict()  # key is thread id
        self.passed_times = list()

    def __del__(self):
        if self.dump_on_deletion:
            self.dump()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, type, value, traceback):
        self.stop()

    def start(self):
        with self.mutex:
            thread_id = threading.get_ident()
            self.start_times[thread_id] = time()
    
    def stop(self):
        with self.mutex:
            passed_time = self._passed_time_under_lock()
            if self.skip_n > 0:
                self.skip_n -= 1
            else:
                self.passed_times.append(passed_time)
        return passed_time
    
    def _passed_time_under_lock(self):
        assert self.mutex.locked()
        thread_id = threading.get_ident()
        passed_time = time() - self.start_times[thread_id]
        return passed_time
    
    def dump(self):
        with self.mutex:
            if len(self.passed_times) > 0:
                total = sum(self.passed_times)
                num = len(self.passed_times)
                print(f"{self.name}: {(total / num):.03f}{self.end}")
            else:
                print(f"{self.name}: no measurements{self.end}")
