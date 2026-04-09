import enum
import random
import numpy as np
import math
 
states = enum.Enum("states", "Front Turn_Right Turn_Left")
 
 
class WallFollower:
    MAX_VEL = 0.22
    MIN_RANGE_SENSOR = 0.16
    MAX_RANGE_SENSOR = 8
    TRACK = 0.16
    WHEEL_RADIUS = 0.033
    WHEEL_SPEED_MAX = MAX_VEL /WHEEL_RADIUS



    def init(self, dt):

        self.dt = dt
        self.maxfdist = 0.23
        self.maxtdist = self.dt * math.sqrt(2) *1.3
        self.state = states.Front
        self.v_vel = 0.15
        self.w_vel = 0
        self.front_dist = 0
        self.right_dist = 0
        self.left_dist = 0
        self._last_right_error = 0 #for the d on pd part
        self._last_left_error = 0 #for the d on pd part
        self._right_dist_error = 0 # for the p part
        self._left_dist_error = 0 # for the p part 
        self._wall_dist_target = 0.2 
        self._Kp = 4
        self._Kd = 5
        self.wall_followed = "right"
        

    def compute_commands(self,v, w, z_scan):

        self.front_rays = self.safe_min(z_scan[0:5]+z_scan[-5:0])
        self.left_rays = self.safe_min(z_scan[len(z_scan)*3/4:len(z_scan)*3/4 + 5]+z_scan[len(z_scan)*3/4-5:len((z_scan)*3/4])
        self.right_rays = self.safe_min(z_scan[len(z_scan)*1/4:len(z_scan)*3/4 + 5]+z_scan[len(z_scan)*1/4-5:len((z_scan)*1/4])

        if self.state == states.Front:
            self.handle_frotn()

        