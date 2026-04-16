#!/usr/bin/env python3
import datetime
import math
import numpy as np
import os
import pytz
import random
from scipy.stats import norm
from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN
import time

from amr_localization import amr_localization_cpp

# added
from ament_index_python.packages import get_package_share_directory


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        min_particles: int = 100,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        logger=None,
        simulation: bool = False,
        use_kld_sampling: bool = True,
        kld_epsilon: float = 0.05,
        kld_delta: float = 0.01,
        kld_bin_size: float = 0.2,
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
            use_kld_sampling: Enable KLD-Sampling for adaptive particle count.
            kld_epsilon: Maximum error allowed in particle approximation (default: 0.05).
            kld_delta: Upper bound on probability that error exceeds epsilon (default: 0.01).
            kld_bin_size: Spatial bin size for discretization [m] (default: 0.2m).

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._logger = logger
        self._particle_count: int = particle_count
        self._min_particles: int = min_particles
        self._max_particles: int = particle_count  # Max for KLD-Sampling

        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._simulation: bool = simulation
        self._iteration: int = 0

        # KLD-Sampling parameters
        self._use_kld_sampling: bool = use_kld_sampling
        self._kld_epsilon: float = kld_epsilon  # Max error allowed
        self._kld_delta: float = kld_delta  # Probability bound
        self._kld_bin_size: float = kld_bin_size  # Spatial discretization

        # self._localized_particle_count: int = 50
        self._localized = False

        #######################################
        # Obtenemos la ruta absoluta al archivo del mapa de forma segura
        pkg_dir = get_package_share_directory("amr_localization")

        # OJO: Asumimos que map_path viene solo con el nombre del archivo (ej: "project.json")
        # Si map_path ya trae "maps/project.json", puedes usar os.path.basename(map_path)
        # para quedarte solo con el nombre y que el join no falle.
        # Por seguridad, usaremos os.path.basename:
        map_filename = os.path.basename(map_path)
        absolute_map_path = os.path.join(pkg_dir, "maps", map_filename)

        self._map = Map(
            absolute_map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )

        map_data = []
        for segment in self._map._map_segments:
            map_data.append([segment[0][0], segment[0][1], segment[1][0], segment[1][1]])

        map_array = np.array(map_data, dtype=np.float64)
        if not map_array.flags["C_CONTIGUOUS"]:
            map_array = np.ascontiguousarray(map_array)

        amr_localization_cpp.init_map_segments(map_array)

        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate with a fast-track for tracking mode."""
        # Si ya estamos localizados, usamos la "vía rápida"
        if self._localized:
            # 1. Calculamos la dispersión (Desviación estándar de las posiciones)
            std_x = np.std(self._particles[:, 0])
            std_y = np.std(self._particles[:, 1])
            # Umbral de seguridad: si las partículas se separan más de 30cm, nos hemos perdido
            if std_x < 0.3 and std_y < 0.3:
                x_mean = np.mean(self._particles[:, 0])
                y_mean = np.mean(self._particles[:, 1])
                theta_mean = math.atan2(
                    np.mean(np.sin(self._particles[:, 2])), np.mean(np.cos(self._particles[:, 2]))
                ) % (2 * math.pi)
                return True, (x_mean, y_mean, theta_mean)
            else:
                # Nos hemos dispersado, perdemos el estado de "localizado"
                self._localized = False
                if self._logger:
                    self._logger.warn(
                        "Pérdida de convergencia. Re-inundando el mapa con partículas..."
                    )

                # --- EXPANSIÓN (Up-sampling): Volvemos a llenar el mapa ---
                self._particles = self._init_particles(
                    self._initial_particle_count,
                    global_localization=True,
                    initial_pose=(0.0, 0.0, 0.0),
                    initial_pose_sigma=(0.0, 0.0, 0.0),
                )
                self._particle_count = self._initial_particle_count

                # Como acabamos de re-esparcir, no devolvemos una pose válida todavía
                return False, (float("nan"), float("nan"), float("nan"))
        # --- MODO LOCALIZACIÓN (DBSCAN) ---
        # Solo llegamos aquí si self._localized es False
        particles_projected = np.array(self._particles, dtype=float)
        theta = particles_projected[:, -1]
        particles_projected = np.hstack(
            [particles_projected[:, :-1], np.cos(theta)[:, None], np.sin(theta)[:, None]]
        )
        clustering = DBSCAN(eps=0.2, min_samples=10).fit(particles_projected)
        labels = clustering.labels_
        n_clusters = len(set(labels) - {-1})
        indexes = clustering.core_sample_indices_
        if n_clusters == 1:
            self._localized = True  # ¡Lo encontramos!
            cluster_particles = self._particles[indexes]
            # --- REDUCCIÓN DE PARTÍCULAS (Down-sampling final) ---
            if len(cluster_particles) > self._min_particles:
                indices_elegidos = np.random.choice(
                    len(cluster_particles), self._min_particles, replace=False
                )
                # IMPORTANTE: ascontiguousarray evita el crasheo en C++
                self._particles = np.ascontiguousarray(cluster_particles[indices_elegidos])
            else:
                self._particles = np.ascontiguousarray(cluster_particles)
            self._particle_count = len(self._particles)
            # Calculamos la media usando el array ya reducido
            x_mean = np.mean(self._particles[:, 0])
            y_mean = np.mean(self._particles[:, 1])
            theta_mean = math.atan2(
                np.mean(np.sin(self._particles[:, -1])), np.mean(np.cos(self._particles[:, -1]))
            ) % (2 * math.pi)
            return True, (x_mean, y_mean, theta_mean)
        elif n_clusters > 1:
            # --- REDUCCIÓN PROGRESIVA (KLD-Sampling aproximado) ---
            particulas_validas = self._particles[labels != -1]
            cantidad_deseada = max(int(200 * n_clusters), self._min_particles)
            if len(particulas_validas) > cantidad_deseada:
                indices = np.random.choice(len(particulas_validas), cantidad_deseada, replace=False)
                self._particles = np.ascontiguousarray(particulas_validas[indices])
            elif len(particulas_validas) > 0:
                self._particles = np.ascontiguousarray(particulas_validas)
            self._particle_count = len(self._particles)
        return False, (float("nan"), float("nan"), float("nan"))

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles using C++ core."""
        self._iteration += 1

        if not self._particles.flags["C_CONTIGUOUS"]:
            self._particles = np.ascontiguousarray(self._particles)

        # Esto hace el ruido, la trigonometría Y el bucle de colisiones a máxima velocidad
        amr_localization_cpp.move_and_collide_particles(
            self._particles, v, w, self._dt, self._sigma_v, self._sigma_w
        )

    def _kld_particle_count(self, particles: np.ndarray) -> int:
        """Computes adaptive particle count using KLD-Sampling algorithm.

        Based on: Dieter Fox, "Adapting the Sample Size in Particle Filters
        Through KLD-Sampling", IJRR 2003.

        Args:
            particles: Current particle set (N, 3) with [x, y, theta].

        Returns:
            Optimal number of particles to maintain given the current distribution.
        """
        if not self._use_kld_sampling:
            return len(particles)

        # Discretize particles into spatial bins
        bin_size = self._kld_bin_size
        bins_set = set()

        for particle in particles:
            x, y, theta = particle
            # Create bin indices (spatial + angular)
            bin_x = int(np.floor(x / bin_size))
            bin_y = int(np.floor(y / bin_size))
            # Angular discretization: 8 bins (45° each)
            bin_theta = int(np.floor(theta / (2 * np.pi / 8)))

            bins_set.add((bin_x, bin_y, bin_theta))

        k = len(bins_set)  # Number of occupied bins

        if k <= 1:
            # Very concentrated → use minimum particles
            return self._min_particles

        # KLD-Sampling formula (Fox 2003)
        # n = (k / (2*epsilon)) * (1 - 2/(9*k) + sqrt(2/(9*k)) * z)^3
        # where z = quantile of standard normal for probability (1 - delta)

        z = norm.ppf(1.0 - self._kld_delta)  # Typically ~2.33 for delta=0.01

        # Compute particle count
        k_minus_1 = k - 1
        term = 1.0 - 2.0 / (9.0 * k_minus_1) + np.sqrt(2.0 / (9.0 * k_minus_1)) * z
        n = (k_minus_1 / (2.0 * self._kld_epsilon)) * (term ** 3)

        # Clamp to valid range
        n_clamped = int(np.clip(n, self._min_particles, self._max_particles))

        # Log if significant change
        if self._logger and abs(n_clamped - len(particles)) > 100:
            self._logger.info(
                f"KLD-Sampling: {len(particles)} → {n_clamped} particles "
                f"(bins={k}, localized={self._localized})"
            )

        return n_clamped

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles using full C++ acceleration with KLD-Sampling."""
        self._iteration += 1
        n = self._particles.shape[0]

        # 1. Raycasting for all particles
        ray_indices = list(range(0, 240, 240 // 8))
        all_z_hat = amr_localization_cpp.batch_raycasting(
            self._particles, ray_indices, 1.5, -0.035, self._sensor_range_max
        )

        # 2. Compute weights
        z_real = np.array([measurements[i] for i in ray_indices], dtype=np.float64)
        probabilities = np.zeros(n, dtype=np.float64)

        amr_localization_cpp.compute_weights(
            probabilities, z_real, all_z_hat, self._sigma_z, self._sensor_range_min
        )

        if np.sum(probabilities) == 0:
            self._localized = False

        # 3. Standard resampling
        amr_localization_cpp.resample_particles(self._particles, probabilities)

        # 4. KLD-Sampling: Adaptive particle count adjustment
        if self._use_kld_sampling:
            optimal_count = self._kld_particle_count(self._particles)
            current_count = len(self._particles)

            if optimal_count < current_count:
                # Down-sampling: Reduce particles
                indices = np.random.choice(current_count, optimal_count, replace=False)
                self._particles = np.ascontiguousarray(self._particles[indices])
                self._particle_count = optimal_count

            elif optimal_count > current_count:
                # Up-sampling: Duplicate particles with noise
                additional_needed = optimal_count - current_count
                # Randomly select particles to duplicate
                duplicate_indices = np.random.choice(current_count, additional_needed, replace=True)
                duplicates = self._particles[duplicate_indices].copy()

                # Add small Gaussian noise to avoid identical particles
                noise_x = np.random.normal(0, 0.05, additional_needed)
                noise_y = np.random.normal(0, 0.05, additional_needed)
                noise_theta = np.random.normal(0, 0.1, additional_needed)

                duplicates[:, 0] += noise_x
                duplicates[:, 1] += noise_y
                duplicates[:, 2] = (duplicates[:, 2] + noise_theta) % (2 * np.pi)

                # Concatenate original + duplicates
                self._particles = np.ascontiguousarray(
                    np.vstack([self._particles, duplicates])
                )
                self._particle_count = optimal_count

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
        """Draws N random valid particles using MASSIVE C++ BATCHING."""
        particles = np.empty((particle_count, 3), dtype=float)
        x_min, y_min, x_max, y_max = self._map.bounds()
        x_o, y_o, theta_o = initial_pose
        x_o_std, y_o_std, theta_o_std = initial_pose_sigma
        # Generamos el triple de partículas necesarias para asegurar que filtramos suficientes
        batch_size = particle_count * 5
        valid_count = 0
        while valid_count < particle_count:
            if global_localization:
                batch_x = np.random.uniform(low=x_min, high=x_max, size=batch_size)
                batch_y = np.random.uniform(low=y_min, high=y_max, size=batch_size)
                batch_theta = np.random.choice(
                    [0, np.pi / 2, np.pi, 3 * np.pi / 2], size=batch_size
                )
            else:
                batch_x = np.random.normal(loc=x_o, scale=x_o_std, size=batch_size)
                batch_y = np.random.normal(loc=y_o, scale=y_o_std, size=batch_size)
                batch_theta = np.random.normal(loc=theta_o, scale=theta_o_std, size=batch_size)
            # Empaquetamos en una matriz Nx2 para C++
            points_to_check = np.column_stack((batch_x, batch_y))

            # ¡MAGIA C++! Devuelve una máscara booleana instantánea
            valid_mask = self._map.batch_contains(points_to_check)

            # Filtramos con NumPy (Ultrarrápido, cero bucles Python)
            valid_x = batch_x[valid_mask]
            valid_y = batch_y[valid_mask]
            valid_theta = batch_theta[valid_mask]

            # Cuántas partículas válidas hemos conseguido en esta tanda
            new_valid = len(valid_x)
            if new_valid > 0:
                # Cuántas nos faltan para llenar el array
                needed = particle_count - valid_count
                take = min(new_valid, needed)

                particles[valid_count : valid_count + take, 0] = valid_x[:take]
                particles[valid_count : valid_count + take, 1] = valid_y[:take]
                particles[valid_count : valid_count + take, 2] = valid_theta[:take] % (2 * math.pi)

                valid_count += take
        return particles
