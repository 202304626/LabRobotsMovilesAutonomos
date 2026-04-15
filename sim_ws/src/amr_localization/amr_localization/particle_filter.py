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

        self._localized_particle_count: int = 25
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

    # def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
    #     """Computes the pose estimate when the particles form a single DBSCAN cluster.

    #     Adapts the amount of particles depending on the number of clusters during localization.
    #     100 particles are kept for pose tracking.

    #     Returns:
    #         localized: True if the pose estimate is valid.
    #         pose: Robot pose estimate (x, y, theta) [m, m, rad].

    #     """
    #     # TODO: 3.10. Complete the missing function body with your code.

    #     localized = False
    #     pose = (float("nan"), float("nan"), float("nan"))

    #     particles_projected = np.array(self._particles, dtype=float)
    #     theta = particles_projected[:, -1]

    #     particles_projected = np.hstack(
    #         [particles_projected[:, :-1], np.cos(theta)[:, None], np.sin(theta)[:, None]]
    #     )

    #     if not self._localized:
    #         clustering = DBSCAN(eps=0.2, min_samples=5).fit(particles_projected)
    #         labels = clustering.labels_
    #         n_clusters = len(set(labels) - {-1})
    #         indexes = clustering.core_sample_indices_

    #     if n_clusters == 1:
    #         localized = True
    #         cluster_particles = np.asarray(self._particles[indexes], dtype=np.float64)

    #         x_mean = np.mean(cluster_particles[:, 0])
    #         y_mean = np.mean(cluster_particles[:, 1])

    #         theta_mean = math.atan2(
    #             np.mean(np.sin(cluster_particles[:, -1])), np.mean(np.cos(cluster_particles[:, -1]))
    #         ) % (2 * math.pi)

    #         pose = (x_mean, y_mean, theta_mean)

    #         self._particle_count = self._localized_particle_count
    #         self._localized = True

    #     elif n_clusters > 1:
    #         self._particle_count = max(int(100 * n_clusters), 100)

    #     return localized, pose

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate with a fast-track for tracking mode."""

        # Si ya estamos localizados, usamos la "vía rápida"
        if self._localized:
            # 1. Calculamos la dispersión (Desviación estándar de las posiciones)
            # Si el std es bajo, es que el grupo está unido y no necesitamos DBSCAN
            std_x = np.std(self._particles[:, 0])
            std_y = np.std(self._particles[:, 1])

            # Umbral de seguridad: si las partículas se separan más de 30cm,
            # es que nos hemos perdido y hay que volver a usar DBSCAN
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
                    self._logger.warn("Pérdida de convergencia, re-activando DBSCAN")

        # --- MODO LOCALIZACIÓN (DBSCAN) ---
        # Solo llegamos aquí si self._localized es False
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
            self._localized = True  # ¡Lo encontramos!
            cluster_particles = np.asarray(self._particles[indexes], dtype=np.float64)

            x_mean = np.mean(cluster_particles[:, 0])
            y_mean = np.mean(cluster_particles[:, 1])
            theta_mean = math.atan2(
                np.mean(np.sin(cluster_particles[:, -1])), np.mean(np.cos(cluster_particles[:, -1]))
            ) % (2 * math.pi)

            self._particle_count = self._localized_particle_count
            return True, (x_mean, y_mean, theta_mean)

        elif n_clusters > 1:
            self._particle_count = max(int(100 * n_clusters), 100)

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
        """Samples a new set of particles using Batch C++ Raycasting."""
        self._iteration += 1
        n = self._particles.shape[0]

        # 1. Definimos los índices de los rayos que queremos usar
        ray_indices = list(range(0, 240, 240 // 8))

        # 2. LANZAMIENTO MASIVO: C++ calcula los láseres de TODAS las partículas a la vez
        # all_z_hat es una matriz (N_partículas, 8_rayos)
        all_z_hat = amr_localization_cpp.batch_raycasting(
            self._particles, ray_indices, 1.5, -0.035, self._sensor_range_max
        )

        # 3. Vectorizamos el cálculo de probabilidad
        z_real = np.array([measurements[i] for i in ray_indices])
        z_real = np.where(np.isnan(z_real), self._sensor_range_min, z_real)

        # Convertimos all_z_hat NaNs a sensor_range_min
        all_z_hat = np.where(np.isnan(all_z_hat), self._sensor_range_min, all_z_hat)

        # Calculamos la probabilidad gaussiana para todos a la vez (vectorizado con NumPy)
        # Prob = exp(-0.5 * ((z_pred - z_obs) / sigma)^2)
        diff = all_z_hat - z_real  # Diferencia entre lo que ve cada partícula y el robot real
        exponent = -0.5 * (diff / self._sigma_z) ** 2
        probs_per_ray = np.exp(exponent) / (self._sigma_z * np.sqrt(2 * np.pi))

        # La probabilidad de la partícula es el producto de las probabilidades de sus rayos
        probabilities = np.prod(probs_per_ray, axis=1)

        # --- El resto del resample sigue igual ---
        total = np.sum(probabilities)
        if total > 0:
            probabilities /= total
        else:
            probabilities = np.ones(n) / n
            self._localized = False

        n_new = self._particle_count

        rand_numbers = np.random.uniform(0, 1 / n_new) + np.arange(n_new) / n_new
        weight_circle = np.cumsum(probabilities)
        prominent_weights = np.digitize(rand_numbers, weight_circle)
        prominent_weights = np.clip(prominent_weights, 0, len(self._particles) - 1)
        self._particles = self._particles[prominent_weights]

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
        x_min, y_min, x_max, y_max = self._map.bounds()
        x_o, y_o, theta_o = initial_pose
        x_o_std, y_o_std, theta_o_std = initial_pose_sigma
        # BATCH GENERATION: Generamos muchísimas partículas de golpe (5x para compensar las que caen fuera del mapa)
        batch_size = particle_count * 5
        valid_count = 0
        while valid_count < particle_count:
            if global_localization:
                # Vectorizado: Generamos miles de coordenadas X, Y y orientaciones en 1 microsegundo
                batch_x = np.random.uniform(low=x_min, high=x_max, size=batch_size)
                batch_y = np.random.uniform(low=y_min, high=y_max, size=batch_size)
                batch_theta = np.random.choice(
                    [0, np.pi / 2, np.pi, 3 * np.pi / 2], size=batch_size
                )
            else:
                # Vectorizado: Seguimiento Gaussiano
                batch_x = np.random.normal(loc=x_o, scale=x_o_std, size=batch_size)
                batch_y = np.random.normal(loc=y_o, scale=y_o_std, size=batch_size)
                batch_theta = np.random.normal(loc=theta_o, scale=theta_o_std, size=batch_size)

            # Filtramos rápidamente las partículas válidas que caen en el mapa libre
            for i in range(batch_size):
                if self._map.contains((batch_x[i], batch_y[i])):
                    particles[valid_count] = [
                        batch_x[i],
                        batch_y[i],
                        batch_theta[i] % (2 * math.pi),
                    ]
                    valid_count += 1
                    if valid_count == particle_count:
                        return particles
        return particles
