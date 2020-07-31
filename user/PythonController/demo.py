import numpy as np
from robot import *

CONTROLLER_CLASS_NAME = "TestCtrl"


class TestCtrl(BaseController):
    def initialize(self):
        _ = self.leg(1).q
        _ = self.leg(2).dq
        _ = self.leg(0).p
        _ = self.leg(3).v
        _ = self.leg(3).tau

        self.leg(1).desired_q = 1.0
        self.leg(1).desired_dq = 2.0, 2.0, 2.0
        self.leg(1).desired_p = np.array([3.0, 3.0, 1.0])
        self.leg(1).desired_v = np.array([3.0, 3.0, 1.0])
        self.leg(1).tau_feed_forward = np.array([3.0, 3.0, 1.0])
        self.leg(2).force_feed_forward = 1.0
        self.leg(1).kp_joint = np.array([3.0, 3.0, 1.0])
        self.leg(1).kd_joint = 3.0, 3.0, 1.0
        self.leg(1).kp_cartesian = 3.0, 3.0, 1.0
        self.leg(1).kd_cartesian = 3.0, 3.0, 1.0

    def run(self):
        print("run")
