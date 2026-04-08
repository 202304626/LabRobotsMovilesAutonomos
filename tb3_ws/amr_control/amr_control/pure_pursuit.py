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

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        if not self._path or len(self._path) == 0:
            return 0.0, 0.0

        _, closest_idx = self._find_closest_point(x, y)
        target = self._find_target_point((x, y), closest_idx)

        dx = target[0] - x
        dy = target[1] - y
        l = np.hypot(dx, dy)

        if l < 1e-6:
            return 0.0, 0.0

        raw_alpha = np.arctan2(dy, dx) - theta
        alpha = (raw_alpha + np.pi) % (2 * np.pi) - np.pi

        v_nominal = 0.08
        v_min = 0.05
        w_max = 0.5
        angle_stop = np.pi / 3

        

        v = max(v_min, v_nominal * (1.0 - abs(alpha) / angle_stop))

        v = 0.1

        w = 2.0 * v * np.sin(alpha) / max(l, self._lookahead_distance)
        # w = float(np.clip(w, -w_max, w_max))

        return float(v), +w

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
        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
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
        # TODO: 4.10. Complete the function body with your code (i.e., determine target_xy).
        target_xy = (0.0, 0.0)
        path = self._path

        # _, idx_closest_node_robot_path = self._find_closest_point(origin_xy[0], origin_xy[1])

        for i in range(origin_idx, len(path)):
            dist_act = np.linalg.norm(np.array(path[i]) - np.array(origin_xy))
            if (
                dist_act >= self._lookahead_distance
            ):  # When distance is greater than or equal to the lookahead distance, we have found the target point
                return path[i]

                """
                if i == origin_idx:
                    return path[i]

                point_before = path[i - 1]  # less than lookahed distance
                dist_before = np.linalg.norm(
                    np.array(point_before) - np.array(origin_xy)
                )  # distance to the point < lookahead distance
                point_after = path[i]  # actual point >= lookahead distance

                # Linear interpolation - % to go from point_before to point_after not all bcs we exceded, we need an intermediate point
                # diff (lookahead and distance before)     /   diff (distance after and distance before)

                weight = (self._lookahead_distance - dist_before) / (
                    dist_act - dist_before
                )  # < 1 bcs dist_act < lookahead distance | num < denom, 100%

                # form point before + weight * (vector)
                target_x = point_before[0] + weight * (point_after[0] - point_before[0])
                target_y = point_before[1] + weight * (point_after[1] - point_before[1])

                target_xy = (target_x, target_y)

                return target_xy
                """

        if path:
            return path[-1]

        return origin_xy
