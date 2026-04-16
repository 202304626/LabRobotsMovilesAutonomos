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

        self._localized_particle_count: int = 400
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

        if self._localized:
            std_x = np.std(self._particles[:, 0])
            std_y = np.std(self._particles[:, 1])
            # ¡Importante! No re-inundamos el mapa, solo volvemos a usar DBSCAN
            if std_x < 0.4 and std_y < 0.4:
                x_mean = np.mean(self._particles[:, 0])
                y_mean = np.mean(self._particles[:, 1])
                theta_mean = math.atan2(
                    np.mean(np.sin(self._particles[:, 2])), np.mean(np.cos(self._particles[:, 2]))
                ) % (2 * math.pi)
                return True, (x_mean, y_mean, theta_mean)
            else:
                self._localized = False
                if self._logger:
                    self._logger.warn(
                        "Pérdida de convergencia, re-activando DBSCAN (Sin re-esparcir)"
                    )

        # --- MODO LOCALIZACIÓN (DBSCAN) ---
        particles_projected = np.array(self._particles, dtype=float)
        theta = particles_projected[:, -1]
        particles_projected = np.hstack(
            [particles_projected[:, :-1], np.cos(theta)[:, None], np.sin(theta)[:, None]]
        )
        clustering = DBSCAN(eps=0.2, min_samples=5).fit(particles_projected)
        labels = clustering.labels_
        n_clusters = len(set(labels) - {-1})
        indexes = clustering.core_sample_indices_
        if n_clusters == 1:
            self._localized = True
            cluster_particles = self._particles[indexes]
            # Reducción SEGURA
            if len(cluster_particles) > self._localized_particle_count:
                indices_elegidos = np.random.choice(
                    len(cluster_particles), self._localized_particle_count, replace=False
                )
                self._particles = np.ascontiguousarray(cluster_particles[indices_elegidos])
            else:
                self._particles = np.ascontiguousarray(cluster_particles)
            self._particle_count = len(self._particles)
            x_mean = np.mean(self._particles[:, 0])
            y_mean = np.mean(self._particles[:, 1])
            theta_mean = math.atan2(
                np.mean(np.sin(self._particles[:, -1])), np.mean(np.cos(self._particles[:, -1]))
            ) % (2 * math.pi)
            return True, (x_mean, y_mean, theta_mean)
        elif n_clusters > 1:
            # 1. Decidir cuántas partículas queremos mantener en esta iteración.
            # Por ejemplo, 100 por cada clúster, con un mínimo de 400 y un máximo de 3000.
            target_count = min(
                self._initial_particle_count,
                max(int(100 * n_clusters), self._localized_particle_count),
            )

            # 2. Si el array actual es más grande que nuestro objetivo, lo reducimos físicamente.
            if len(self._particles) > target_count:
                # Extraemos todas las partículas que el DBSCAN consideró parte de algún clúster (labels != -1)
                core_particles = self._particles[labels != -1]

                if len(core_particles) > target_count:
                    # Si hay demasiadas partículas "buenas", nos quedamos con una muestra aleatoria
                    indices_elegidos = np.random.choice(
                        len(core_particles), target_count, replace=False
                    )
                    self._particles = np.ascontiguousarray(core_particles[indices_elegidos])
                elif len(core_particles) > 0:
                    # Si hay menos partículas "buenas" que el target, nos quedamos con todas ellas
                    self._particles = np.ascontiguousarray(core_particles)

                # Actualizamos el contador oficial
                self._particle_count = len(self._particles)

                if self._logger:
                    self._logger.info(
                        f"Reduciendo partículas gradualmente: {n_clusters} clústeres -> {self._particle_count} partículas"
                    )
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

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles using full C++ acceleration."""
        self._iteration += 1
        n = self._particles.shape[0]
        ray_indices = list(range(0, 240, 240 // 8))
        all_z_hat = amr_localization_cpp.batch_raycasting(
            self._particles, ray_indices, 1.5, -0.035, self._sensor_range_max
        )
        z_real = np.array([measurements[i] for i in ray_indices], dtype=np.float64)
        probabilities = np.zeros(n, dtype=np.float64)
        amr_localization_cpp.compute_weights(
            probabilities, z_real, all_z_hat, self._sigma_z, self._sensor_range_min
        )
        if np.sum(probabilities) == 0:
            self._localized = False

        amr_localization_cpp.resample_particles(self._particles, probabilities)

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
