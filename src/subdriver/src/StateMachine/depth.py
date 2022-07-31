import threading
import time

class DepthControl(object):
    
    def __init__(self):
        self.power = 0
        self.run = True
        self.thread = None
    def __run(threadname):
        global self
        while (self.run):
            time.sleep(2)
            self.power = -.15
            time.sleep(1)
            self.power = 0
    def run(self):
        self.thread = Thread(target = __run, args=("DepthRunThread", ) )
    
    def stop(self):
        self.run = False
    
    def get_power(self):
        return self.power