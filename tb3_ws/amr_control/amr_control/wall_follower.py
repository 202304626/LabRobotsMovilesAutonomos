import enum
import random
import numpy as np
import math
 
states = enum.Enum("states", "Front Turn_Right Turn_Left")
 
 
class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""
 
    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]
 
    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        """Wall following class initializer.
 
        Args:
            dt: Sampling period [s].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.
 
        """
        self._dt: float = dt 
        self._logger = logger
        self._simulation: bool = simulation 
 
        self._front_distance_threshold = 0.23  # Distance threshold to obstacles in front [m]
        self.expected_turning_distance = self._front_distance_threshold * np.sqrt(2) * 1.3
 
        self._x_vel = 0.15
        self._w_vel = 0.0
  
        self._front_dist = 0
        self._right_dist = 0
        self._left_dist = 0
 
        self._last_right_error = 0
        # self._last_front_error = 0
        self._last_left_error = 0
 
        self._right_dist_error = 0
        self._left_dist_error = 0
        self._front_dist_error = 0
        self._wall_dist_target= 0.2
        self._whole_path_width = 0.4
 
        # Controller gains
        self._Kp = 4
        self._Kd = 5

        self._followed_wall = "right"
  
        self._state = states.Front
 
    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.
 
        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
 
        Returns:
            The linear and angular velocity commands for the robot. They must
                v: Linear velocity [m/s].
                w: Angular velocity [rad/s].
 
        """
 
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
 
        self._front_dist = self._safe_min(list(z_scan[0:5]) + list(z_scan[-5:]))  # Front distance
 
        self._right_dist = self._safe_min(
            z_scan[(3 * len(z_scan) // 4) - 5 : (3 * len(z_scan) // 4) + 5]
        )  # Right distance
 
        self._left_dist = self._safe_min(
            z_scan[(1 * len(z_scan) // 4) - 5 : (1 * len(z_scan) // 4) + 5]
        )  # Left distance
 
        try:
            self._right_dist_error = self._wall_dist_target - self._right_dist
            self._left_dist_error = - self._wall_dist_target + self._left_dist

        except Exception:
            self._right_dist_error = 0
            self._left_dist_error = 0
 
        if self._state == states.Front:
            self._x_vel, self._w_vel = self._handle_front_move()
 
        elif self._state == states.Turn_Right:
            self._x_vel, self._w_vel = self._handle_turn_right()
 
            if self._front_dist >= self.expected_turning_distance:

                self._state = states.Front
 
        elif self._state == states.Turn_Left:
            self._x_vel, self._w_vel = self._handle_turn_left()
 
            if self._front_dist >= self.expected_turning_distance:
                
                self._state = states.Front
 
        self._last_right_error = self._right_dist_error
        self._last_left_error = self._left_dist_error
 
        return self._x_vel, self._w_vel
 
    def _handle_front_move(self):

        if self._left_dist + self._right_dist >= self._whole_path_width:

            if self._right_dist <= self._left_dist:
                self._followed_wall = "right"

            else:
                self._followed_wall = "left"
            
        if self._front_dist <= self._front_distance_threshold:
            self._handle_turn()
            return 0.0, 0.0
 
        # Controller for angular velocity
        w_vel = self.get_w_vel()
        
        # Everytime front velocity (try to reach the max if posible)
        x_vel = (self.LINEAR_SPEED_MAX) - abs(w_vel)*(self.TRACK/2)  # Maybe we can calculate how much we can put here, and make it faster
 
        return x_vel, w_vel
 
    def _handle_turn(self):

        # Reset error
        self._right_dist_error = 0
        self._left_dist_error = 0

        diff = self._right_dist - self._left_dist  # Here maybe put a threshold
 
        if abs(diff) <= 0.05:
            self._state = random.choice([states.Turn_Left, states.Turn_Right])
 
        elif diff <= 0:  # The wall is at the right
            self._state = states.Turn_Left
            self._followed_wall = "right"
 
        elif diff > 0:  # The wall is at the left
            self._state = states.Turn_Right
            self._followed_wall = "left"
 
    def _handle_turn_right(self):
        # Just rotate condition and assign direction
        if self._state == states.Turn_Right:
            return (
                0.0,
                -0.5,
            )  # May we can see how much we can put here, like the max, and optimice time
 
    def _handle_turn_left(self):
        if self._state == states.Turn_Left:
            return (
                0.0,
                0.5,
            )  # May we can see how much we can put here, like the max, and optimice time
        
    def _safe_min(self, values, default=8.0):
        vals = [v for v in values if v is not None and not math.isinf(v) and not math.isnan(v)]
        return min(vals) if vals else default


    def get_w_vel(self):

        if self._followed_wall == "right":
            error, last_error = self._right_dist_error, self._last_right_error

        elif self._followed_wall == "left":
            error, last_error = self._left_dist_error, self._last_left_error

        else: 
            return 0.0

        return self._Kp * error + self._Kd * (
            (error - last_error) / self._dt
            
        )
    


























