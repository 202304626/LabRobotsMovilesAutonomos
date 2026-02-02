import enum
import random
import numpy as np
 
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
 
        self._front_distance_threshold = 0.25
 
        self._x_vel = 0.15
        self._w_vel = 0.0
 
        self._front_dist = 0
        self._right_dist = 0
        self._left_dist = 0
 
        self._last_right_error = 0
        # self._last_front_error = 0
        # self._last_left_error = 0
 
        self._right_dist_error = 0
        self._left_dist_error = 0
        self._front_dist_error = 0
        self.expected_turning_distance = self._front_distance_threshold  # * np.sqrt(2)
 
        self._right_dist_target = 0.2
        self._Kp_right = 3.0
        self._Kd_right = 5.0
 
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
 
        # FOR TOMORROW WATCH THE FORMULAS IN THE PDF
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
 
        self._front_dist = min(list(z_scan[0:5]) + list(z_scan[-5:]))  # Front distance
 
        self._right_dist = min(
            z_scan[(3 * len(z_scan) // 4) - 5 : (3 * len(z_scan) // 4) + 5]
        )  # Right distance
 
        self._left_dist = min(
            z_scan[(1 * len(z_scan) // 4) - 5 : (1 * len(z_scan) // 4) + 5]
        )  # Left distance
 
        try:
            self._right_dist_error = self._right_dist_target - self._right_dist
 
        except Exception:
            pass
 
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
 
        # Everytime front distance
        # self._x_vel = 0.15
        # Controller for angular velocity
        # self._w_vel = self._Kp_right * self._right_dist_error + self._Kd_right * (
        #    (self._right_dist_error - self._last_right_error) / self._dt
        # )
 
        self._last_right_error = self._right_dist_error
        # self._last_front_dist = self._front_dist
        # self._last_left_dist = self._left_dist
 
        return self._x_vel, self._w_vel
 
    def _handle_front_move(self):
        if self._front_dist <= self._front_distance_threshold:
            self._handle_turn()
            return 0.0, 0.0
 
        # Everytime front velocity
        x_vel = 0.15
        # Controller for angular velocity
        w_vel = self._Kp_right * self._right_dist_error + self._Kd_right * (
            (self._right_dist_error - self._last_right_error) / self._dt
        )
 
        return x_vel, w_vel
 
    def _handle_turn(self):
        diff = self._right_dist - self._left_dist  # Here maybe put a threshold
 
        if abs(diff) <= 0.05:
            self._state = random.choice([states.Turn_Left, states.Turn_Right])
 
        elif diff <= 0:  # The wall is at the right
            self._state = states.Turn_Right
 
        elif diff > 0:  # The wall is at the left
            self._state = states.Turn_Left
 
    def _handle_turn_right(self):
        # Just rotate condition and assign direction
        if self._state == states.Turn_Right:
            return 0.0, 0.5
 
    def _handle_turn_left(self):
        if self._state == states.Turn_Left:
            return 0.0, -0.5
            self._w_vel = -0.5
 