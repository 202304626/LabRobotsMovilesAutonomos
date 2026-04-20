import math

import numpy as np


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.7,
        logger=None,
        simulation: bool = False,
    ):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._simulation: bool = simulation
        self._initial_alignment_done = False

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        if not self._path or len(self._path) == 0:
            return 0.0, 0.0

        # here we are going to apply the pure suit formula to get angular vel
        L = self._find_target_point((x, y), self._find_closest_point(x, y)[1])

        vector_to_target = np.array(L) - np.array((x, y))
        l = np.linalg.norm(vector_to_target)

        raw_alpha = np.arctan2(vector_to_target[1], vector_to_target[0]) - theta
        alpha = (raw_alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Config params
        v_min = 0.10  # constant velocity, we change it later but rn constant
        v_max = 0.22  # max v vel
        max_angle = math.radians(15)  # max angle

        if not self._initial_alignment_done:
            if abs(alpha) > max_angle:
                v_cmd = 0.0
                w_cmd = 0.5 * np.sign(alpha)
                return v_cmd, w_cmd
            else:
                self._initial_alignment_done = True

        # Calculate v
        v_desired = v_max * (
            1
            - abs(alpha)
            / max_angle
        )  # Reduce speed as alpha increases, control in curves

        v = max(v_min, min(v_desired, v_max))  # Clamp v to [v_min, v_max]
        v = 0.1

        w = (
            v * 2 * np.sin(alpha) / l  # substitute r in the formula
            if l > 0
            else 0.0
        )

        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        closest_xy = (0.0, 0.0)
        closest_idx = 0

        path = self._path
        closest_xy = min(
            path,
            key=lambda point: np.linalg.norm(np.array(point) - np.array((x, y))),
        )
        closest_idx = path.index(closest_xy)

        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        target_xy = (0.0, 0.0)
        path = self._path

        for i in range(origin_idx, len(path)):
            dist_act = np.linalg.norm(np.array(path[i]) - np.array(origin_xy))
            if dist_act >= self._lookahead_distance:
                return path[i]

        if path:
            return path[-1]

        return origin_xy
