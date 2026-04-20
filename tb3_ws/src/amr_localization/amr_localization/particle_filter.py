import datetime
import math
import os
import random

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pytz
from matplotlib import pyplot as plt
from scipy.stats import norm
from sklearn.cluster import DBSCAN

from amr_localization.maps import Map


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (
            float("nan"),
            float("nan"),
            float("nan"),
        ),
        logger=None,
        simulation: bool = False,
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._logger = logger
        self._particle_count: int = particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._simulation: bool = simulation
        self._iteration: int = 0

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        localized = False
        pose = (float("nan"), float("nan"), float("nan"))

        particles_projected = np.array(self._particles, dtype=float).copy()

        theta = particles_projected[:, -1]

        particles_projected = np.hstack([
            particles_projected[:, :-1],
            np.cos(theta)[:, None],
            np.sin(theta)[:, None],
        ])

        clustering = DBSCAN(
            eps=0.2, min_samples=10, n_jobs=-1, algorithm="kd_tree"
        ).fit(particles_projected)
        labels = clustering.labels_
        n_clusters = len(set(labels) - {-1})
        indexes = clustering.core_sample_indices_
        self._logger.warning(f"Number of clusters: {n_clusters}")

        for cluster_id in set(labels):
            if cluster_id == -1:
                continue

            cluster_points = np.array(self._particles)[labels == cluster_id]

            centroid_x = np.mean(cluster_points[:, 0])
            centroid_y = np.mean(cluster_points[:, 1])

            self._logger.warning(
                f"Cluster {cluster_id} -> centroid: ({centroid_x:.3f}, {centroid_y:.3f})"
            )

            self._logger.warning(
                f"Cluster {cluster_id} -> particle count: {len(cluster_points)}"
            )

        if n_clusters == 1:
            localized = True

            cluster_particles = np.array(self._particles[indexes], dtype=float)

            x_mean = np.mean(cluster_particles[:, 0])
            y_mean = np.mean(cluster_particles[:, 1])

            theta_mean = math.atan2(
                np.mean(np.sin(cluster_particles[:, -1])),
                np.mean(np.cos(cluster_particles[:, -1])),
            ) % (2 * math.pi)

            pose = (x_mean, y_mean, theta_mean)

            self._particle_count = 50

        elif n_clusters > 1:
            if n_clusters > 3:
                self._particle_count = max(int(100 * n_clusters), 70)
            else:
                self._particle_count = max(int(70 * n_clusters), 70)

        if self._logger:
            if localized:
                pass
            else:
                self._logger.warning(
                    f"[DBSCAN] Searching... Clusters: {n_clusters}. "
                    f"Particles: {self._particle_count}"
                )

        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # We assume that v and w are measured with respect of the robot

        for particle in self._particles:
            # Extract the components of this particle, to update them
            x_o, y_o, theta_o = particle

            # Generate noise
            noise_v = np.random.normal(loc=0, scale=self._sigma_v)
            noise_w = np.random.normal(loc=0, scale=self._sigma_w)

            # Project the velocities to the x,y axis (global world axis)
            x_vel = np.cos(theta_o) * (v + noise_v)
            y_vel = np.sin(theta_o) * (v + noise_v)

            # Update x,y,theta with noise
            particle[0] += x_vel * self._dt
            particle[1] += y_vel * self._dt
            particle[2] += (w + noise_w) * self._dt

            # Normalize to make sure that the angle is in range [0, 2*pi]
            particle[2] %= 2 * np.pi

            # Compute the intersection with the walls
            colission_segment = [(x_o, y_o), (particle[0], particle[1])]
            colissions, _ = self._map.check_collision(
                segment=colission_segment, compute_distance=False
            )

            if colissions:
                # Readjust the position of the particle, to avoid trespassing the wall
                particle[0] = colissions[0]
                particle[1] = colissions[1]

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        probabilities = np.array(
            [self._measurement_probability(measurements, particle) for particle in self._particles],
            dtype=float,
        )

        probabilities = np.nan_to_num(probabilities, nan=0.0, posinf=0.0, neginf=0.0)

        total = np.sum(probabilities)

        # If all weights are 0 or invalid, use a uniform distribution
        if total <= 0.0 or not np.isfinite(total):
            probabilities = np.ones(len(self._particles), dtype=float) / len(self._particles)
        else:
            probabilities = probabilities / total

        n = self._particle_count
        rand_numbers = np.random.uniform(0, 1 / n) + np.arange(n) / n  # array of strata
        # we sum the weights in order to apply the stratified samples
        weight_circle = np.cumsum(probabilities)
        # we choose the largest weight whose cumulative value does not exceed the strat.
        prominent_weights = np.digitize(rand_numbers, weight_circle)
        prominent_weights = np.clip(prominent_weights, 0, len(self._particles) - 1)

        self._particles = self._particles[prominent_weights]

        if self._logger is not None:
            self._logger.warning(f"Running resample. Particle count: {len(self._particles)}.")

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
        self,
        particle_count: int,
        global_localization: bool,
        initial_pose: tuple[float, float, float],
        initial_pose_sigma: tuple[float, float, float],
    ) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=float)

        # Extract the bounds of the map
        x_min, y_min, x_max, y_max = self._map.bounds()

        # Extract the values needed to create particles
        x_o, y_o, theta_o = initial_pose
        x_o_std, y_o_std, theta_o_std = initial_pose_sigma

        for i in range(particle_count):
            valid = False

            while not valid:
                if global_localization:  # First localization mode
                    # We create the particle in the bounds of the map using a uniform distribution
                    particle_x = np.random.uniform(low=x_min, high=x_max)
                    particle_y = np.random.uniform(low=y_min, high=y_max)
                    orientation = np.random.choice([0, np.pi / 2, np.pi, 3 * np.pi / 2])

                else:  # Pose tracking mode
                    # We create the particle near to its position using a normal distribution
                    particle_x = np.random.normal(loc=x_o, scale=x_o_std)
                    particle_y = np.random.normal(loc=y_o, scale=y_o_std)
                    orientation = np.random.normal(loc=theta_o, scale=theta_o_std)

                # If it is in an invalid place, we generate it again
                if self._map.contains(
                    (particle_x, particle_y)
                ):  # If the particle is valid, we store it
                    particles[i] = [
                        particle_x,
                        particle_y,
                        orientation % (2 * math.pi),
                    ]  # To normalize and just have values in range [0, 2*pi]
                    valid = True

        return particles

    def _sense(self, pose: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            pose: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
        rays = np.arange(0, 240, 240 // 8).tolist()

        segments = self._lidar_rays(pose=pose, indices=rays)

        z_hat: list[float] = [
            self._map.check_collision(segment=pair, compute_distance=True)[1] for pair in segments
        ]

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        return math.exp(-0.5 * ((x - mu) / sigma) ** 2) / (sigma * math.sqrt(2 * math.pi))

    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0
        predicted_measurements = self._sense(pose=particle)

        n = len(measurements)
        step = max(n // 8, 1)
        rays = list(range(0, n, step))

        for i, (ray_idx, predicted_measurement) in enumerate(zip(rays, predicted_measurements)):
            measurement = measurements[ray_idx]

            # Fix measurement
            if math.isinf(measurement) or math.isnan(measurement) or measurement <= 0.0:
                neighbor_indices = [
                    j for j in range(ray_idx - 2, ray_idx + 3)
                    if 0 <= j < n and j != ray_idx
                ]

                valid_neighbors = [
                    measurements[j]
                    for j in neighbor_indices
                    if not math.isinf(measurements[j])
                    and not math.isnan(measurements[j])
                    and measurements[j] > 0.0
                ]

                if valid_neighbors:
                    measurement = max(valid_neighbors)
                else:
                    continue

            # Fix predicted_measurement
            if (
                math.isinf(predicted_measurement)
                or math.isnan(predicted_measurement)
                or predicted_measurement <= 0.0
            ):
                pred_neighbor_indices = [
                    j for j in range(i - 2, i + 3)
                    if 0 <= j < len(predicted_measurements) and j != i
                ]

                valid_pred_neighbors = [
                    predicted_measurements[j]
                    for j in pred_neighbor_indices
                    if not math.isinf(predicted_measurements[j])
                    and not math.isnan(predicted_measurements[j])
                    and predicted_measurements[j] > 0.0
                ]

                if valid_pred_neighbors:
                    predicted_measurement = max(valid_pred_neighbors)
                else:
                    continue

            probability *= self._gaussian(
                mu=measurement,
                sigma=self._sigma_z,
                x=predicted_measurement,
            )

        return probability
