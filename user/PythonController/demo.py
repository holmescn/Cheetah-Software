import numpy as np
from robot import *

CONTROLLER_CLASS_NAME = "TestCtrl"


class TestCtrl(BaseController):
    def __init__(self):
        BaseController.__init__(self)
        self._instance = self

    def initialize(self):
        _ = self.leg[0].q
        _ = self.leg[1].dq
        _ = self.leg[2].p
        _ = self.leg[3].v
        _ = self.leg[2].tau

        self.leg[1].desired_q = 1.0
        self.leg[1].desired_dq = 2.0, 2.0, 2.0
        self.leg[1].desired_p = np.array([3.0, 3.0, 1.0])
        self.leg[1].desired_v = np.array([3.0, 3.0, 1.0])
        self.leg[1].tau_feed_forward = np.array([3.0, 3.0, 1.0])
        self.leg[2].force_feed_forward = 1.0
        self.leg[1].kp_joint = np.array([3.0, 3.0, 1.0])
        self.leg[1].kd_joint = 3.0, 3.0, 1.0
        self.leg[1].kp_cartesian = 3.0, 3.0, 1.0
        self.leg[1].kd_cartesian = 3.0, 3.0, 1.0

    def run(self):
        print("run")
        _ = self.state.contact
        _ = self.state.position
        _ = self.state.orientation
        _ = self.state.a_body
        _ = self.state.v_body
        _ = self.state.r_body
        _ = self.state.omega_body
        _ = self.state.a_world
        _ = self.state.v_world
        _ = self.state.omega_world
