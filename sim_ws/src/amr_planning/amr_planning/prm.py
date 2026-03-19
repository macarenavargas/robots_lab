import datetime
import numpy as np
import os
import pytz
import random
import time
import math

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


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
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
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

    def _find_closest(self, point) -> tuple[float, float]:
        """
            (auxiliar function for the "find_path" method)
            Finds the closest point on the graph 

            Args : 
                point : The start or end (x,y)

            Returns: 
                The closest node that belongs to the graph 
        """
        smallest_distance = np.inf
        closest_node = None

        for value in self._graph.keys():
            dist = math.dist(value, point)
            if dist < smallest_distance:
                smallest_distance = dist
                closest_node = value

        return closest_node

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


        ancestors: dict[tuple[float, float], tuple[float, float]] = {}  # {(x, y: (x_prev, y_prev)}

        # TODO: 4.3. Complete the function body (i.e., replace the code below).
        path: list[tuple[float, float]] = []

        # 1. Get the start and end nodes 
        closest_node_start = self._find_closest(start)
        closest_node_goal = self._find_closest(goal)

        if closest_node_start is None: 
            print(" The start point is not valid, no node in the graph")

        if closest_node_goal is None:
            print(" The goal point is not valid, no node in the graph")


        # 2. Initilize the open and close list. 
        open_list: dict[tuple[float, float], tuple[float, float]] = {}
        closed_list = set()


        # 3. Principal loop 
        # initial node is the start node. add it to the list to initialize the loop 
        g = 0
        h = math.dist(closest_node_start, closest_node_goal)
        f = h + g
        open_list[closest_node_start] = (f, g)

        goal_reached = False

        while open_list: 

            # extract the node that has the minimum value from the open list 
            node = min(open_list, key=lambda k: open_list.get(k)[0])

            #extract their values and delete it from the open list 
            _ , g_node = open_list[node]
            del open_list[node]

            if node == closest_node_goal:
                goal_reached = True
                break

            # extraet their neighbours 
            list_closest_neighbours = self._graph[node]

            for neighbour in list_closest_neighbours:
                # calculate their values 
                #g_now = g_node + 1
                g_now = g_node + math.dist(node, neighbour)
                h_now = math.dist(neighbour, closest_node_goal)
                f_now = g_now + h_now

                # if its not in either lists, add it to the open list 
                if neighbour not in closed_list and neighbour not in open_list:
                    open_list[neighbour] = (f_now, g_now)
                    ancestors[neighbour] = node
                
                # if it was already in the open list, only add it if it has less g
                elif neighbour in open_list:
                    if open_list[neighbour][1] > g_now:
                        open_list[neighbour] = (f_now, g_now)
                        ancestors[neighbour] = node

            

            # add the original node to the close list 
            closed_list.add(node)

        # if we have not got to the goal node, cannot compute the path 
        if not goal_reached:
            raise RuntimeError("No path found")

        # add the real starting and ending points 
        if start != closest_node_start:
            ancestors[closest_node_start] = start
        if goal != closest_node_goal:
            ancestors[goal] = closest_node_goal
        
        # reconstruct the path : 
        path = self._reconstruct_path(closest_node_start, closest_node_goal, ancestors)

        return path

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
     
        extended_path: list[tuple[float, float]] = []

        # -----add the new intermediate nodes -----
        for i in range(len(path) - 1): 

      
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            extended_path.append((x1, y1))

            for j in range(additional_smoothing_points): 
                # we make the linear interpolation 
                step = (j + 1)/ (additional_smoothing_points+1)

                x = x1 + step * (x2 - x1)
                y = y1 + step * (y2 - y1)
                extended_path.append((x,y))
        
        extended_path.append(path[-1])
        original_path = extended_path.copy()
        smoothed_path :list[tuple[float, float]] = []
        smoothed_path = extended_path.copy()

        # -----gradient descent-----

        total_change = np.inf 

        while total_change > tolerance: 
            
            total_change = 0 
            for i in range(1, len(smoothed_path) -1 ): # not iterate in the start and end 
                
                # we get the node of the smoothed path and the original node 
                x, y = smoothed_path[i]
                x_old, y_old = x, y
                x_original, y_original = original_path[i]
                
                # CRITERIA 1 : it has to be near the original value 
                #criteria1 = node - original_node 

                x+= data_weight * (x_original - x)
                y+= data_weight * (y_original - y)

                
                # CRITERIA 2 : it has to be near its neighbours 
                # criteria 2 = prev_node + next_node - 2 * node 
                
                x_prev, y_prev = smoothed_path[i-1]
                x_next, y_next = smoothed_path[i+1]

                x+= smooth_weight * (x_prev + x_next - 2 * x)
                y+= smooth_weight * (y_prev + y_next - 2 * y)

                # update the modified x and y of this node 
                smoothed_path[i] = (x, y)

                total_change += abs (x - x_old) + abs (y - y_old)



        return smoothed_path

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
        """Connects every generated node with all the nodes that are closer than a given threshold.

        Args:
            graph: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A modified graph with lists of connected nodes as values.

        """
        # TODO: 4.2. Complete the missing function body with your code.

        nodes = list(graph.keys())
        num_nodes = len(nodes)

       
        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                node_1 = nodes[i]
                node_2 = nodes[j]

                dist = math.dist(node_1, node_2)
                if dist <= connection_distance:
                    
                    # Check if the line does not step into an obstacle
                    # Create "crosses" (faster then check_collision)
                    #print(self._map.crosses([node_1, node_2]))
                    if not self._map.crosses([node_1, node_2]):
                
                        # Add both as the conexion is for both sides 
                        graph[node_1].append(node_2)
                        graph[node_2].append(node_1)
         #----checks----
        total_edges = sum(len(v) for v in graph.values())
        print("nodes:", len(graph))
        print("total neighbor refs:", total_edges)
        isolated = sum(1 for v in graph.values() if len(v) == 0)
        print("isolated nodes:", isolated)

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
        #print(len(graph))
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

        if not use_grid:
            # cumpute random points with a uniform distributuion 
            while len(graph) < node_count:
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)

                point = (round(x, 3), round(y, 3))

                if self._map.contains(point):
                    graph[point] = []

        else:
            # compute points that are equally spaced along the grid
            for x in np.arange(x_min, x_max, grid_size):
                for y in np.arange(y_min, y_max, grid_size):
                    point = (round(x, 3), round(y, 3))

                    if self._map.contains(point):
                        graph[point] = []

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

        node = goal
        node = goal
        path = [node]

        while node in ancestors:
            node = ancestors[node]
            path.append(node)

        path.reverse()

        return path


if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    # node_count = 250 when use_grid = True 
    prm = PRM(map_path, use_grid=True, node_count=250, grid_size   =0.1, connection_distance=0.15)
    #prm = PRM(map_path, use_grid=False, node_count=300, grid_size   =0.1, connection_distance=0.3)
    
    #prm = PRM(map_path, use_grid=False, node_count= 300, grid_size   =0.1, connection_distance=0.3)
    
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(-0.6, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.3, additional_smoothing_points=3
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
