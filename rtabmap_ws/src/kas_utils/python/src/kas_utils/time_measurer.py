import threading
from time import time, monotonic
from .collection import Collection


class TimeMeasurer(Collection):
    def __init__(self, name, skip_first_n=0):
        super().__init__(name,
            print_results=TimeMeasurer.print_results,
            observation_to_str=TimeMeasurer.observation_to_str)
        self._abbreviation = "TM"
        self.skip_first_n = skip_first_n

        self.start_stamps_times = dict()  # key is thread id

    def start(self):
        with self._mutex:
            thread_id = threading.get_ident()
            start_stamp = time()
            start_time = monotonic()
            self.start_stamps_times[thread_id] = (start_stamp, start_time)

    def stop(self):
        with self._mutex:
            if self.skip_first_n > 0:
                self.skip_first_n -= 1
                return
            stop_time = monotonic()
            thread_id = threading.get_ident()
            start_stamp, start_time = self.start_stamps_times[thread_id]
            passed_time = stop_time - start_time
            self._add_under_lock((start_stamp, passed_time))

    @staticmethod
    def print_results(name, observations):
        if len(observations) > 0:
            start_stamps, passed_times = zip(*observations)
            total = sum(passed_times)
            num = len(passed_times)
            print(f"{name}: {(total / num):.03f}")
        else:
            print(f"{name}: no measurements")

    @staticmethod
    def observation_to_str(observation):
        start_stamp, passed_time = observation
        out_str = f"{start_stamp:.06f} {passed_time:.06f}"
        return out_str

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, type, value, traceback):
        self.stop()

    def __del__(self):
        super().__del__()
