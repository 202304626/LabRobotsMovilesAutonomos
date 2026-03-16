import numpy as np


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

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.11. Complete the function body with your code (i.e., compute v and w).
        v = 0.10  # constant velocity, we change it later but rn constant

        if not self._path or len(self._path) == 0:
            # Si no hay ruta, velocidad 0 y giro 0
            return 0.0, 0.0

        # here we are going to apply the pure suit formula to get angular vel
        L = self._find_target_point((x, y), self._find_closest_point(x, y)[1])

        vector_to_target = np.array(L) - np.array((x, y))
        l = np.linalg.norm(vector_to_target)
        alpha = np.arctan2(vector_to_target[1], vector_to_target[0]) - theta

        r = (
            l / (2 * np.sin(alpha)) if np.sin(alpha) != 0 else float("inf")
        )  # Avoid division by zero

        w = v / r if r != float("inf") else 0.0  # If r is infinite, set w to 0 (m/s) / m = s^-1

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

        if path:
            return path[-1]

        return origin_xy
