import time
import sys


class Timer:
    def __init__(self):
        self.running_now = False

    def start(self):
        if not self.running_now:
            self.start_time = time.perf_counter()
            self.running_now = True

    def stop(self):
        if self.running_now:
            self.stop_time = time.perf_counter()
            self.running_now = False

    def print_secs(self, time, file=sys.stdout):
        secs = time % 60
        mins = time // 60
        hrs = mins // 60
        mins = mins % 60

        print("% i h, %i min, % 2.1d sec" % (hrs, mins, secs), file=file)

    def elapsed_time(self):
        if self.running_now:
            elapsed_time = time.perf_counter - self.start_time
        else:
            elapsed_time = self.stop_time - self.start_time
        return elapsed_time

    def get_elapsed_time(self):
        return self.elapsed_time()

    def show_elapsed_time(self, file=sys.stdout):
        self.print_secs(self.elapsed_time(), file)

    def time(self):
        return time.perf_counter
