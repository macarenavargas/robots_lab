import numpy as np 
import math 
class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

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
        self._lookahead_distance: float = lookahead_distance # 0.25 in real robot 
        self._path: list[tuple[float, float]] = []
        self._simulation: bool = simulation
        self._v : float = 0.1 # 0.s15 
        self._Ki : float = 0.15
        self._error_integral : float= 0.0

        
        if self._simulation:
            self._alpha_threshold:float = 0.5
        else:
            self._alpha_threshold:float = 0.5
        self._last_closest_idx: int = 0 
        # Robot limits (using exact same values as wall_follower for consistency)
    

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
            # if self._logger:
            #     self._logger.warn("No path available in Pure Pursuit")
            return 0.0, 0.0
        
        

        # get the closest and target points 
        closest_xy, closest_idx = self._find_closest_point(x,y)
        # if self._logger:
        #     self._logger.debug(
        #         f"[CLOSEST] idx={closest_idx}, point={closest_xy}"
        #     )

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

        # if self._logger:
        #     self._logger.info(f"PURE PERSUIT:lookeahead distance = {self._lookahead_distance:.2f}, target point = {target_xy}, alpha = {alpha:.2f} rad")
             
        
        # check if alpha is too high
        if abs(alpha) > self._alpha_threshold:
            v = 0.0 
            w = - 0.2 * np.sign(alpha) # use the sign to rotate in the direction of the target point.
            
            # if self._logger: 
            #     self._logger.info(f" LARGE ALPHA !! alpha={alpha:.2f}, setting v=0 and w={w:.2f}")
            

        else: 

            # calculate the distance error to the closest path point and accumulate it
            distance_error = np.linalg.norm(np.array((x, y)) - np.array(closest_xy))
            self._error_integral += distance_error * self._dt

            # calculate the control commands
            v = self._v # v is constant in pure pursuit algorithm.
            w = - 2 * v * np.sin(alpha) / self._lookahead_distance
            # v, w = self._saturate_commands(v, w, alpha)
            if self._Ki != 0.0:
              w += self._Ki * self._error_integral * np.sign(alpha)
                        
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _saturate_commands(self, v: float, w: float, alpha: float) -> tuple[float, float]:
        """Applies kinematic constraints to keep velocity commands within robot limits.

        Args:
            v: linear velocity [m/s].
            w: angular velocity [rad/s].
            alpha: angle between robot heading and target point [rad].

        Returns:
            A tuple containing the possible velocity commands:
                v: limited linear velocity [m/s].
                w: limited angular velocity [rad/s].
        """
        b = self.TRACK / 2.0
        v_max = self.LINEAR_SPEED_MAX

        # Ecuación (5) de la imagen
        # w_max_permitida = (v_max - abs(v)) / b
        
        # 1. Basándose en la w calculada, recalculamos la v permitida (Ecuación 6)
        # Esto reduce v si es necesario, o la incrementa para aprovechar el margen disponible.
        v_permitida = v_max - abs(w) * b
        v_permitida = max(0.0, v_permitida)

        # 2. Ahora, usando esta nueva velocidad v_permitida, recalculamos la w.
        # Esto garantiza que el robot mantenga la trayectoria y la curvatura del Pure Pursuit.
        w_nueva = -2 * v_permitida * np.sin(alpha) / self._lookahead_distance

        return v_permitida, w_nueva

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
        # if self._logger:
        #     self._logger.info(f"Closest point: {closest_xy} at index {closest_idx} with distance {closest_distance:.2f}")
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
    
        # if self._logger:
        #     self._logger.info(f"Target point: {target_xy} at index {origin_idx}")
        return target_xy