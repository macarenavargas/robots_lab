import math
import numpy as np
from enum import Enum, auto

class State(Enum):
    FIND_WALL = auto()    
    FOLLOW_WALL = auto()  
    CORNER = auto()



class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""
git 
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
       
        self.K_p = 1.2
        self.K_d = 0.8 
        self._prev_error = None 

        self._dist_ref = 0.2
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
        
        # clean de z_scan array to deal with nan and inf values     
        clean_scan = []
        for r in z_scan:
    
            if math.isnan(r) or math.isinf(r) or r == 0.0:
                # if we get any value that doesnt make sense we set it to the senor max
                clean_scan.append(self.SENSOR_RANGE_MAX)
            else:
                clean_scan.append(r)
        
        # extract from the scan array the set of values we need
        d_right_wall = min(clean_scan[160:200]) # values we use to detect how far the right wall is
        d_front = min(clean_scan[-5:] + clean_scan[:5]) # values to detect the front
        

        # state machine logic: transitions

        if self._state == State.CORNER:
            # in case we are already rotating, we keep going until the front of the robot is far enough from the wall
            if d_front > 1.0:
                self._state = State.FOLLOW_WALL
                self._prev_error = None
            else:
                
                self._state = State.CORNER


        else:
            
            if d_front < 0.20: # corner detection threashold
                self._state = State.CORNER
            
            elif d_right_wall < 1.0: 
                self._state = State.FOLLOW_WALL
            
            else:
                self._state = State.FIND_WALL

        v = 0.0
        w = 0.0

        # state machine logic: definition of each state
        
        if self._state == State.CORNER:
            v = 0.0
            # negative angular velocity turns the robot to the left
            w = -0.5 
            self._prev_error = None # error reset

        elif self._state == State.FOLLOW_WALL:

            # control pd logic
            v = self._vel_ref            
            error = self._dist_ref - d_right_wall
            if self._prev_error is None:
                derivative = 0.0
            else:
                derivative = (error - self._prev_error) / self._dt
        
            w = - (self.K_p * error + self.K_d * derivative)
            
            self._prev_error = error

        elif self._state == State.FIND_WALL:
            # in case the robot is turned away, it rotates until it finds it
            v = self._vel_ref
            w = 0.20 
            self._prev_error = None

        # cosntrains to respect the phisical limits of the robot
        b = self.TRACK / 2.0
        r = self.WHEEL_RADIUS
        w_limit = (self.WHEEL_SPEED_MAX * r - abs(v)) / b
        
        if abs(w) > w_limit:
            # we choose to sacrifice some linear velocity so we can satisfy the angular velocity demand
            v_altered = (self.WHEEL_SPEED_MAX * r) - (abs(w) * b) # v = vmax − |ω|b

            # if angular velocity demanded requires a negative linear velocity value -> can't satisfy the requtement
            if v_altered < 0: 
                v = 0.0
                w = np.sign(w) * (self.WHEEL_SPEED_MAX * r / b)  # set w to its limit
            else:
                v = v_altered
        
     
        if self._logger is not None:
             self._logger.info(f"[{self._state.name}] F:{d_front:.2f} | R:{d_right_wall:.2f} | Err:{self._dist_ref - d_right_wall:.2f} | w:{w:.2f}")
        return v, w