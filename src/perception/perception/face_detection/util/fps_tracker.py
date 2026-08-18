import time

class FPSTracker:
    def __init__(self):
        self.startTime = time.time()
        self.total_number_of_frames = 0
        self.counter = 0
        self.frameRate = 1  # The number of seconds to wait for each measurement.
        self.fps = 0
        
    def update(self):
        self.total_number_of_frames += 1
        self.counter += 1
        if self._elapsed_time() > self.frameRate:
            self.fps = self.counter / self._elapsed_time()
            self.counter = 0 
            self.startTime = time.time()

    def _elapsed_time(self):
        return time.time() - self.startTime
