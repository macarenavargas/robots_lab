import numpy as np 
import math 
class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.2,
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
        self._v : float = 0.15 # 0.15 
        self._alpha_threshold:float = 0.5
        self._last_closest_idx: int = 0 


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
        
        

        # get the closest and target points 
        closest_xy, closest_idx = self._find_closest_point(x,y)
   
        target_xy = self._find_target_point(closest_xy ,closest_idx )
        #target_xy = self._find_target_point((x,y),closest_idx )
        x_target,y_target = target_xy
        

        # calculate angle alpha and beta with the formulas
        # values between 0 and 2pi 
        beta = np.arctan2(y_target - y, x_target - x)
        alpha = beta - theta 

        if self._simulation: 
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        else: 
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))

       
        # check if alpha is too high
        if abs(alpha) > self._alpha_threshold:
            v = 0.0 
            w = - 0.4 * np.sign(alpha) # use the sign to rotate in the direction of the target point.
            
            if self._logger: 
                self._logger.info(f" LARGE ALPHA !! alpha={alpha:.2f}, setting v=0 and w={w:.2f}")
            return v, w
        
        else: 
            # calculate the control commands
            v = self._v #v is constant in pure pursuit algorithm. 
            w = - 2* v * np.sin(alpha) /  self._lookahead_distance
            #w = np.clip(w, -0.3, 0.3) # limit the value so its not so big. 
            
            
        
            # limit v if w is too big so that it behaves better in the curves 
            # if abs(w)> 0.3: 
            #     v = 0.1
            
            if self._logger:
                self._logger.info(f"PURE PERSUIT: v = {v:+.3f} m/s, w = {w:+.3f} rad/s, {alpha:.2f} rad")
                        
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
        """
        
        Find the closest path point to the current robot pose.
        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.
        """

         # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).

        if not self.path:
            return (0.0, 0.0), 0
        
        
        start_idx = self._last_closest_idx
        closest_xy = self.path[start_idx]
        closest_idx = start_idx
        closest_distance = np.inf

        for i in range(start_idx, len(self.path)):
            # for every node, get the distance to the robot 
            x_node, y_node = self.path[i]
            distance = np.linalg.norm(np.array((x, y)) - np.array((x_node, y_node)))
            # if it is the closest one, save its parameters. 

            if distance < closest_distance:
                closest_xy = (x_node, y_node)
                closest_idx = i
                closest_distance = distance


        self._last_closest_idx = closest_idx
        
        return closest_xy, closest_idx
    

    #-----------find target point -----------------

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """
        DISTANCE METHOD: choose the first point that is at least lookahead_distance away from the current robot pose.
        """
        

         # TODO: 4.10. Complete the function body 
        target_xy = (0.0, 0.0) # default value if no point is found.
        
        #cdist of numpy (?)
        for i in range(origin_idx, len(self.path)): 

            distance = np.linalg.norm(np.array(self.path[i]) - np.array(origin_xy))
            if distance > self._lookahead_distance: #>= (?)
                # strategy : we choose the first point that passes the distance 
                target_xy = self.path[i]
                return target_xy

        
        # if the target has not changed, then we assume we reached the goal 
        if target_xy == (0.0, 0.0): 
            target_xy = self.path[-1] # return goal 
            return target_xy
    
      
        return target_xy