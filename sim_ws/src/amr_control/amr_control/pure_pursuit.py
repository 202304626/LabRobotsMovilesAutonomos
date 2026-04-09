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

        if not self._path or len(self._path) == 0:
            # Si no hay ruta, velocidad 0 y giro 0
            return 0.0, 0.0

        # here we are going to apply the pure suit formula to get angular vel
        L = self._find_target_point((x, y), self._find_closest_point(x, y)[1])

        vector_to_target = np.array(L) - np.array((x, y))
        l = np.linalg.norm(vector_to_target)

        raw_alpha = np.arctan2(vector_to_target[1], vector_to_target[0]) - theta
        alpha = (raw_alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        """if self._logger is not None:
            self._logger.warning(
                f"Robot pose: x = {x:.3f} m, y = {y:.3f} m, theta = {theta:.3f} rad"
            )"""

        ## OPTIMIZACION
        # Config params
        v_min = 0.10  # constant velocity, we change it later but rn constant
        v_max = 0.22  # max v vel
        max_angle = np.pi / 3.5  # max angle

        if (
            abs(alpha) > max_angle
        ):  # if the angle is too big, we can not try to go making the circle
            # Turn in place with max angular velocity, alpha = beta - theta, if alpha positive we need left turn, if negative right turn, this in sim, in real change dirs
            w = 0.5 * np.sign(alpha)
            return 0.0, w

        # Calculate v
        v_desired = v_max * (
            1
            - abs(alpha)
            / max_angle  # alpha lower than max_angle, then abs(alpha) / max_angle < 1, so we reduce the speed, if alpha is 0, we can go at max speed
        )  # Reduce speed as alpha increases, control in curves

        v = max(v_min, min(v_desired, v_max))  # Clamp v to [v_min, v_max]

        w = (
            v * 2 * np.sin(alpha) / l  # subsitute r in the formula
            if l > 0
            else 0.0  # if sin alpha is 0, we can go straight, if l is 0, we are at the target.
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

    def _find_closest_point(
        self, x: float, y: float
    ) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        """closest_xy = (0.0, 0.0)
        closest_idx = 0

        path = self._path

        closest_xy = min(
            path,
            key=lambda point: np.linalg.norm(np.array(point) - np.array((x, y))),
        )
        closest_idx = path.index(closest_xy)"""

        path = np.array(self._path)

        dx = path[:, 0] - x
        dy = path[:, 1] - y
        squared_distances = dx**2 + dy**2
        closest_idx = int(np.argmin(squared_distances))
        closest_xy = (path[closest_idx][0], path[closest_idx][1])

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
        path = np.array(self._path)
        np_origin_xy = np.array(origin_xy)

        future_path = path[
            origin_idx:
        ]  # Get the future path points starting from the closest point

        if len(future_path) == 0:
            return tuple(
                path[-1]
            )  # If no future points, return the last point in the path

        dx = future_path[:, 0] - origin_xy[0]
        dy = future_path[:, 1] - origin_xy[1]
        squared_distances = dx**2 + dy**2

        lookhead_sqr = self._lookahead_distance**2
        idxs = np.where(squared_distances >= lookhead_sqr)[0]

        if len(idxs) > 0:
            return tuple(
                future_path[idxs[0]]
            )  # Return the first point that is at least lookahead distance away

        return tuple(
            path[-1]
        )  # If no point is far enough, return the last point in the path

        # _, idx_closest_node_robot_path = self._find_closest_point(origin_xy[0], origin_xy[1])

        for i in range(origin_idx, len(path)):
            dist_act = np.linalg.norm(path[i] - np_origin_xy)
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

        if len(path) > 0:
            return path[-1]

        return origin_xy
