import numpy as np
from robot import *

CONTROLLER_CLASS_NAME = "StandUp"


class StandUp(BaseController):

    def initialize(self):
        self.jpos = np.zeros((4, 3), dtype=np.float32)
        self.target_jpos = np.array([
            [0.0, -45.0, 90.0],
            [0.0, -45.0, 90.0],
            [0.0, -55.0, 80.0],
            [0.0, -55.0, 80.0],
        ]) * 3.1415926 / 180.0
        self.counter = 0

    def run(self):
        self.counter += 1
        if self.counter < 10:
            self.update_jpos()
        else:
            a, b = 0.0, 1.0
            if self.counter < 250:
                b = self.counter / 250.0
                a = 1.0 - b
            inter_jpos = a*self.jpos + b*self.target_jpos
            for i in range(4):
                self.leg(i).desired_q = inter_jpos[i, :]
                self.leg(i).desired_dq = 0.0
                self.leg(i).kp_joint = 10.0
                self.leg(i).kd_joint = 1.0

    def update_jpos(self):
        for i in range(4):
            self.jpos[i, :] = self.leg(i).q
