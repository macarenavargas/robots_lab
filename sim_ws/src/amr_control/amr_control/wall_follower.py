from enum import Enum, auto
import math
import numpy as np

class State(Enum):
    FIND_WALL = auto()    
    FOLLOW_WALL = auto()  
    ROTATE_CORNER = auto()

class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    
    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._simulation: bool = simulation

        self._state = State.FIND_WALL
       
        self.K_p = 2.0
        self.K_d = 0
        self._prev_error = 0.0  # memory for de derivative term

        self._dist_ref = 0.5  # distance that the robot has too keep from the wall
        self._vel_ref = 0.15

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            The linear and angular velocity commands for the robot. They must
                v: Linear velocity [m/s].
                w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).

        z_scan = [r if not math.isnan(r) and not math.isinf(r) else self.SENSOR_RANGE_MAX for r in z_scan]
       
        d_front = min(z_scan[-5 :]+z_scan[:5])
       
        d_wall = min(z_scan[160 : 200])

        # State machine: criteria for state changes
        if d_front < 0.2: 
            self._state = State.ROTATE_CORNER
        elif d_wall < 1.0: 
            self._state = State.FOLLOW_WALL
        else:
            self._state = State.FIND_WALL

        v = 0.0
        w = 0.0
        
        if self._state == State.ROTATE_CORNER:
            v = 0.0
            w = -0.6  
            self.prev_error = 0.0 # reset the pd error so it doesnt acummulate

        elif self._state == State.FOLLOW_WALL:
           
            v = self._vel_ref
            
            error = self._dist_ref - d_wall
            derivative = (error - self._prev_error) / self._dt
    
            w = (self.K_p * error + self.K_d * derivative)
            
            self._prev_error = error

        elif self._state == State.FIND_WALL:
            # go foward and turn slightly to find the wall
            v = self._vel_ref
            w = 0.15 # turn right slightly to look for the waññ
            self._prev_error = 0.0

        
        # cinematic restricction.
        # dealing with saturation
        
        b = self.TRACK / 2.0
        r = self.WHEEL_RADIUS
        

        w_limit = (self.WHEEL_SPEED_MAX * r - abs(v)) / b # |w_max| = (|phi_max|*r- v) / b
        
        # if estimated w is greater than the limit
        if abs(w) > w_limit:
            # we priorize w, and so limit v
            
            v_altered = (self.WHEEL_SPEED_MAX * r) - (abs(w) * b) # v_max = |phi_max|*r - |w|*b
            
            if v_altered < 0: 
                # we make sure that the w demand doesn't ask for a negative v
                v = 0.0
                # fix w to its rotation maximium (v=0)
                w = np.sign(w) * (self.WHEEL_SPEED_MAX * r / b) # |w_max| = (|phi_max|*r- 0) / b
            else:
                v = v_altered
        
        # Log (opcional) para depurar estados
        self._logger.info(f"State: {self._state.name} | Front: {d_front:.2f} | Wall: {d_wall:.2f} | v: {v:.2f}, w: {w:.2f}")

        return v, w
