import numpy as np

from amr_control import amr_control_cpp


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.5,
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

        self._is_aligned = False

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        if not self._path or len(self._path) == 0:
            return 0.0, 0.0
        if not hasattr(self, "_last_closest_idx"):
            self._last_closest_idx = 0
            self._is_aligned = False

        v, w, closest_idx, is_aligned = amr_control_cpp.compute_pure_pursuit(
            x,
            y,
            theta,
            self._path_arr,
            self._last_closest_idx,
            self._lookahead_distance,
            0.22,  # v_max
            0.10,  # v_min
            2.50,  # w_max
            self._is_aligned,
        )
        self._last_closest_idx = closest_idx
        self._is_aligned = is_aligned
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value
        self._path_arr = np.array(value)
        self._is_aligned = False
        self._last_closest_idx = 0

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        path = self._path_arr

        if not hasattr(self, "_last_closest_idx"):
            self._last_closest_idx = 0

        start_idx = max(0, self._last_closest_idx - 5)
        end_idx = min(len(path), self._last_closest_idx + 30)

        local_path = path[start_idx:end_idx]

        dx = local_path[:, 0] - x
        dy = local_path[:, 1] - y
        squared_distances = dx**2 + dy**2

        local_closest_idx = int(np.argmin(squared_distances))
        self._last_closest_idx = start_idx + local_closest_idx

        closest_xy = (path[self._last_closest_idx][0], path[self._last_closest_idx][1])
        return closest_xy, self._last_closest_idx

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
        path = self._path_arr

        future_path = path[origin_idx:]

        if len(future_path) == 0:
            return tuple(path[-1])

        dx = future_path[:, 0] - origin_xy[0]
        dy = future_path[:, 1] - origin_xy[1]
        squared_distances = dx**2 + dy**2

        lookhead_sqr = self._lookahead_distance**2
        idxs = np.where(squared_distances >= lookhead_sqr)[0]

        if len(idxs) > 0:
            return tuple(future_path[idxs[0]])

        return tuple(path[-1])
