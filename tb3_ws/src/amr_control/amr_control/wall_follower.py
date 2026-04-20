import enum
import math
import random
import time

import numpy as np


# States: added Fixed_Front for real robot
states = enum.Enum("states", "Front Turn_Right Turn_Left Fixed_Front")


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the absence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0   # Maximum LiDAR sensor range [m]
    TRACK = 0.16             # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033     # Radius of the wheels [m]
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

        self._front_distance_threshold = 0.3  # Distance threshold to obstacles in front [m]
        self.expected_turning_distance = self._front_distance_threshold * np.sqrt(2) * 1.3
        self.expected_turning_distance_upper = 0.75

        self._x_vel = 0.15
        self._w_vel = 0.0

        self._front_dist = 0
        self._right_dist = 0
        self._left_dist = 0

        self._fixed_time = 0

        self._last_right_error = 0
        self._last_left_error = 0

        self._right_dist_error = 0
        self._left_dist_error = 0
        self._front_dist_error = 0
        self._wall_dist_target = 0.2
        self._whole_path_width = 0.4

        # Controller gains
        self._Kp = 7.6
        self._Kd = 6

        self._followed_wall = "right"  # We start with this default value

        self._state = states.Front  # We start with this default state

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
        self._front_dist = z_scan[-1]  # Front distance
        self._right_dist = z_scan[3 * len(z_scan) // 4]  # Right distance
        self._left_dist = z_scan[len(z_scan) // 4]  # Left distance

        try:
            self._right_dist_error = self._wall_dist_target - self._right_dist
            self._left_dist_error = -self._wall_dist_target + self._left_dist

        except Exception:
            self._right_dist_error = 0
            self._left_dist_error = 0

        if self._state == states.Front:
            self._x_vel, self._w_vel = self._handle_front_move(z_scan)

        elif self._state == states.Turn_Right:
            self._x_vel, self._w_vel = self._handle_turn_right()

            if (
                self._front_dist >= self.expected_turning_distance
                and self._front_dist <= self.expected_turning_distance_upper
            ):
                self._fixed_time = time.perf_counter()
                self._state = states.Fixed_Front
                self._logger.info(f"Front distance: {self._front_dist}")
                self._logger.info(f"state: {self._state}\n")

        elif self._state == states.Turn_Left:
            self._x_vel, self._w_vel = self._handle_turn_left()

            if (
                self._front_dist >= self.expected_turning_distance
                and self._front_dist <= self.expected_turning_distance_upper
            ):
                self._fixed_time = time.perf_counter()
                self._state = states.Fixed_Front
                self._logger.info(f"Front distance: {self._front_dist}")
                self._logger.info(f"state: {self._state}\n")

        elif self._state == states.Fixed_Front:
            self._x_vel, self._w_vel = self._handle_fixed_front()

            if (time.perf_counter() - self._fixed_time) > 0.35:
                self._state = states.Front
                self._logger.info(f"Front distance: {self._front_dist}")
                self._logger.info(f"state: {self._state}\n")

        self._last_right_error = self._right_dist_error
        self._last_left_error = self._left_dist_error

        return self._x_vel, self._w_vel

    def _handle_front_move(self, z_scan):
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

        x_vel = 0.1  # We start with a low vel, and will increase it while we improve our control
        return x_vel, w_vel

    def _handle_turn(self):
        # Reset error when start turning
        self._right_dist_error = 0
        self._left_dist_error = 0

        diff = self._right_dist - self._left_dist

        if abs(diff) <= 0.05:
            self._state = random.choice([states.Turn_Left, states.Turn_Right])
            self._logger.info("Intersection reached...")

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
                -0.37,
            )

    def _handle_turn_left(self):
        if self._state == states.Turn_Left:
            return (
                0.0,
                0.37,
            )

    def _safe_min(self, values, default=SENSOR_RANGE_MAX):
        vals = []
        for v in values:
            if v is not None and not math.isinf(v) and not math.isnan(v):
                vals.append(v)
            else:
                vals.append(default)
                self._logger.info(f"Value out of range: {v}")

        return min(vals) if vals else default

    def _safe_min2(self, values, default=None):
        if default is None:
            default = self.SENSOR_RANGE_MAX

        if values:
            ordered = [values[0]]
            for k in range(1, len(values)):
                if k < len(values):
                    ordered.append(values[k])
                if k < len(values):
                    ordered.append(values[-k])
                if len(ordered) >= len(values):
                    break
            values = ordered[:len(values)]

        vals = []
        for v in values:
            if v is not None and not math.isinf(v) and not math.isnan(v):
                vals.append(v)
            else:
                vals.append(default)
                if self._logger:
                    self._logger.info(f"Value out of range: {v}")

        return min(vals) if vals else default

    def get_w_vel(self):
        if self._followed_wall == "right":
            error, last_error = self._right_dist_error, self._last_right_error

        elif self._followed_wall == "left":
            error, last_error = self._left_dist_error, self._last_left_error

        else:
            return 0.0

        value = self._Kp * error + self._Kd * (
            (error - last_error) / self._dt
        )

        if value < 0:
            w = max(value, -0.34)
            return w

        else:
            w = min(value, 0.34)
            return w

    def _handle_fixed_front(self):
        return 0.10, 0.0
