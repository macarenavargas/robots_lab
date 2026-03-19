import numpy as np 
class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.3,
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
        self._v : float = 0.1
        self._alpha_threshold:float = 0.8


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
        if not self._path:
            return 0.0, 0.0
        
        v= self._v
        

        # get the closest and target points 
        closest_xy, closest_idx = self._find_closest_point(x,y)
        target_xy = self._find_target_point(closest_xy,closest_idx )
        #target_xy = self._find_target_point((x,y),closest_idx )
        x_target,y_target = target_xy

        # follow the formulas :
        # l = distance between target and current location
        l = np.linalg.norm(np.array((x,y)) - np.array( target_xy))
        if l < 1e-6:
            return 0.0, 0.0

        # calculate angle B 
        beta = np.arctan2(y_target - y,x_target - x)
        alpha = beta - theta 
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # normalize

        w = 2 * v * np.sin(alpha) / l

        # if alpha is large -> make the robot rotate over itself
    
        if abs(alpha) > self._alpha_threshold: 

            v = 0.0
            #v = 0.05
            #w = np.sin(alpha) * 0.8
            #w = 1.5 * alpha
            w = np.clip(1.5 * alpha, -1.0, 1.0)

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

        closest_distance = np.inf

        for i in range(len(self.path)):
            x_node, y_node = self.path[i]
            distance = np.linalg.norm(np.array((x,y) ) - np.array( (x_node,y_node )))
            if distance < closest_distance: 
                closest_xy = (x_node, y_node)
                closest_idx = i 
                closest_distance = distance 

                                
        
        return closest_xy, closest_idx

    def _find_target_point_1(
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

        for i in range(origin_idx, len(self.path)): 

            distance = np.linalg.norm(np.array(self.path[i]) - np.array(origin_xy))
            if distance >= self._lookahead_distance: 
                target_xy = self.path[i]
                return target_xy

        
        if target_xy == (0.0, 0.0): 
            #target_xy = self.path[-1] # return goal 
            return target_xy

        return target_xy


    def _find_target_point(self, origin_xy: tuple[float, float], origin_idx: int) -> tuple[float, float]:
        target_xy = self.path[-1]
        accumulated = 0.0
        prev = origin_xy
        for i in range(origin_idx + 1, len(self.path)):
            curr = self.path[i]
            accumulated += np.linalg.norm(np.array(curr) - np.array(prev))

            if accumulated >= self._lookahead_distance:

                return curr

            prev = curr

        return target_xy
 
    
