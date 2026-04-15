import datetime
import numpy as np
import os
import pytz
import random
import time
from scipy.spatial import cKDTree

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


from ament_index_python.packages import get_package_share_directory

import amr_localization.maps_cpp as maps_cpp

import heapq  # Importamos la cola de prioridad de alto rendimiento
import math


class PRM:
    """Class to plan a path to a given destination using probabilistic roadmaps (PRM)."""

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
        logger=None,
        simulation: bool = False,
    ):
        """Probabilistic roadmap (PRM) class initializer.

        Args:
            map_path: Path to the map of the environment.
            obstacle_safety_distance: Distance to grow the obstacles by [m].
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].
            sensor_range_max: Sensor measurement range [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        #################################################################
        # Obtenemos la ruta absoluta al archivo del mapa de forma segura
        pkg_dir = get_package_share_directory("amr_localization")

        # OJO: Asumimos que map_path viene solo con el nombre del archivo (ej: "project.json")
        # Si map_path ya trae "maps/project.json", puedes usar os.path.basename(map_path)
        # para quedarte solo con el nombre y que el join no falle.
        # Por seguridad, usaremos os.path.basename:
        map_filename = os.path.basename(map_path)
        absolute_map_path = os.path.join(pkg_dir, "maps", map_filename)

        self._map = Map(
            absolute_map_path,  # <--- Pasamos la ruta absoluta
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=obstacle_safety_distance,
        )

        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = self._create_graph(
            use_grid,
            node_count,
            grid_size,
            connection_distance,
        )

        self._logger = logger
        self._simulation: bool = simulation

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """Computes the shortest path from a start to a goal location using the A* algorithm.
        Args:
            start: Initial location in (x, y) [m] format.
            goal: Destination in (x, y) [m] format.
        Returns:
            Path to the destination. The first value corresponds to the initial location.
        """

        # Check if the goal is valid
        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")
        ancestors: dict[tuple[float, float], tuple[float, float]] = {}
        # Encontrar los nodos más cercanos de forma vectorizada/nativa rápida
        closest_start_node = min(
            self._graph.keys(),
            key=lambda node: math.hypot(node[0] - start[0], node[1] - start[1]),
        )
        closest_goal_node = min(
            self._graph.keys(),
            key=lambda node: math.hypot(node[0] - goal[0], node[1] - goal[1]),
        )
        ancestors[goal] = closest_goal_node
        ancestors[closest_start_node] = start
        # Diccionario para rastrear el coste real (G) más barato a cada nodo
        g_scores = {closest_start_node: 0.0}

        # open_list será nuestro Min-Heap (Cola de prioridad)
        # Guardaremos tuplas de la forma: (F_score, G_score, nodo_tupla)
        # heapq extrae SIEMPRE el de menor F_score en O(1)
        open_list = []
        initial_h = math.hypot(closest_start_node[0] - goal[0], closest_start_node[1] - goal[1])
        heapq.heappush(open_list, (initial_h, 0.0, closest_start_node))

        closed_list = set()
        while open_list:
            # Extraemos el nodo con el menor F_score de forma instantánea
            current_f, current_g, current_node = heapq.heappop(open_list)
            # Si hemos llegado al objetivo, reconstruimos el camino
            if current_node == closest_goal_node:
                return self._reconstruct_path(start, goal, ancestors)
            # Si ya expandimos este nodo con un coste mejor o igual, lo ignoramos
            if current_node in closed_list:
                continue
            closed_list.add(current_node)
            for neighbor in self._graph[current_node]:
                if neighbor in closed_list:
                    continue
                # Distancia euclídea ultrarrápida usando math.hypot
                dist = math.hypot(current_node[0] - neighbor[0], current_node[1] - neighbor[1])
                tentative_g = current_g + dist
                # Si descubrimos un camino mejor hacia el vecino
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    ancestors[neighbor] = current_node
                    g_scores[neighbor] = tentative_g

                    # Heurística al objetivo
                    h = math.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1])
                    f_score = tentative_g + h

                    # Empujamos el vecino a la cola de prioridad en O(log N)
                    heapq.heappush(open_list, (f_score, tentative_g, neighbor))
        raise ValueError("No path found from start to goal.")

    @staticmethod
    def smooth_path(
        path: list[tuple[float, float]],
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
    ) -> list[tuple[float, float]]:
        """Computes a smooth path from a piecewise linear path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            additional_smoothing_points: Number of equally spaced intermediate points to add
                between two nodes of the original path.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                less than this value.

        Returns: Smoothed path (initial location first) in (x, y) [m] format.

        """
        # TODO: 4.5. Complete the function body (i.e., load smoothed_path).

        ### Add additional points
        if additional_smoothing_points > 0:
            new_path = []
            for i in range(len(path) - 1):
                p_i = path[i]
                p_next = path[i + 1]

                new_path.append(p_i)

                vector = np.array(p_next) - np.array(p_i)

                for j in range(1, additional_smoothing_points + 1):
                    intermediate_point = (
                        p_i[0] + vector[0] * j / (additional_smoothing_points + 1),
                        p_i[1] + vector[1] * j / (additional_smoothing_points + 1),
                    )
                    new_path.append(intermediate_point)

            new_path.append(path[-1])

            path = new_path

        path_arr = np.array(path)
        s = np.copy(path_arr)
        while True:
            s_anterior = np.copy(s)

            # ¡Vectorización de NumPy! Actualizamos todos los puntos interiores de golpe
            # s[1:-1] representa todos los puntos menos el primero y el último
            s[1:-1] = (
                s[1:-1]
                + data_weight * (path_arr[1:-1] - s[1:-1])
                + smooth_weight * (s[2:] + s[:-2] - 2 * s[1:-1])
            )
            # Comprobación de convergencia
            if np.sum(np.abs(s_anterior - s)) < tolerance:
                return [tuple(p) for p in s]

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """Draws particles.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        # Plot the nodes
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)

        # Plot the edges
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot([x_start, x_end], [y_start, y_end], "c-", linewidth=0.25)

        # Plot the path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "bo", markersize=4)  # Draw nodes as blue circles

        # Plot the smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "yo", markersize=2)  # Draw nodes as yellow circles

        if path or smoothed_path:
            axes.plot(
                x_val[0], y_val[0], "rs", markersize=7
            )  # Draw a red square at the start location
            axes.plot(
                x_val[-1], y_val[-1], "g*", markersize=12
            )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
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
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if display:
            plt.show(block=block)

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Connects nodes using a KDTree and MASSIVE C++ checking."""

        nodes = list(graph.keys())
        if not nodes:
            return graph

        tree = cKDTree(nodes)
        pairs = tree.query_pairs(connection_distance)

        # ¡ESTA ES LA LÍNEA QUE FALTABA! Convierte el set en lista
        pairs_list = list(pairs)
        if not pairs_list:
            return graph
        # Preparamos TODOS los pares en un solo Array NumPy Nx4
        segments_array = np.empty((len(pairs_list), 4), dtype=np.float64)
        for idx, (i, j) in enumerate(pairs_list):
            segments_array[idx] = [nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1]]

        # ¡¡EL GRAN LLAMADO A C++!! Verifica miles de segmentos en menos de 1ms
        crosses_mask = self._map.batch_crosses(segments_array)

        for idx, (i, j) in enumerate(pairs_list):
            if not crosses_mask[idx]:  # Si C++ dice que no cruza, lo añadimos
                node1 = nodes[i]
                node2 = nodes[j]
                graph[node1].append(node2)
                graph[node2].append(node1)
        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a roadmap as a graph with edges connecting the closest nodes.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A dictionary with (x, y) [m] tuples as keys and lists of connected nodes as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph = self._generate_nodes(use_grid, node_count, grid_size)
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a set of valid nodes to build a roadmap with.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.

        Returns: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        # TODO: 4.1. Complete the missing function body with your code.
        x_min, y_min, x_max, y_max = self._map.bounds()

        if use_grid:
            grid = np.mgrid[
                x_min : x_max + grid_size : grid_size,
                y_min : y_max + grid_size : grid_size,
            ]
            for x in grid[0].flat:
                for y in grid[1].flat:
                    if self._map.contains((x, y)):
                        graph[(x, y)] = []
        else:
            # BATCH GENERATION: Generamos nodos en masa para aniquilar el bucle lento de Python
            batch_size = node_count * 5
            valid_count = 0
            while valid_count < node_count:
                # Array de NumPy con las posiciones aleatorias de golpe
                batch_x = np.random.uniform(low=x_min, high=x_max, size=batch_size)
                batch_y = np.random.uniform(low=y_min, high=y_max, size=batch_size)
                for i in range(batch_size):
                    particle_x = batch_x[i]
                    particle_y = batch_y[i]
                    # Si no colisiona, lo añadimos al grafo
                    if self._map.contains((particle_x, particle_y)):
                        # Usar tuplas evita problemas de hasheo con las keys del diccionario
                        graph[(particle_x, particle_y)] = []
                        valid_count += 1
                        if valid_count == node_count:
                            return graph

        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Goal location in (x, y) [m] format.
            ancestors: Dictionary with (x, y) [m] tuples as keys and the node (x_prev, y_prev) [m]
                from which it was added to the open list as values.

        Returns: Path to the goal (start location first) in (x, y) [m] format.

        """
        path: list[tuple[float, float]] = []

        # TODO: 4.4. Complete the missing function body with your code.

        current_node = goal
        while current_node != start:
            path.append(current_node)
            current_node = ancestors[current_node]
        path.append(start)
        path.reverse()

        return path


if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    prm = PRM(
        map_path,
        use_grid=True,
        node_count=500,
        grid_size=0.1,
        connection_distance=0.20,
        obstacle_safety_distance=0.12,
    )
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(1.0, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.1, additional_smoothing_points=4
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
