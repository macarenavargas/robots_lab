import math
import numpy as np
from enum import Enum, auto


class State(Enum):
    DECIDE_SIDE = auto()
    FIND_WALL = auto()
    FOLLOW_WALL = auto()
    CORNER = auto()


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    # Navigation Threshold
    DIST_REF = 0.2  # Target distance to wall [m]
    VEL_REF = 0.15  # Target linear velocity [m/s]
    MAX_FRONT_DISTANCE = 0.25  # Front distance to trigger corner [m]
    WALL_LOST_VAL = 1.2  # Distance to consider wall lost [m]
    WALL_FOUND_VAL = 0.8  # Distance to consider new wall found [m]
    TURN_ANGLE_TARGET = (
        math.pi / 2
    )   # Angle the rpbot has to tu acomploish when in enters corner state

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

        self._state = State.DECIDE_SIDE
        self._side_sign = 1  # 1 = Right, -1 = Left
        self._angle_turned = 0.0  # used to acumulate angle to know how much the robot has turned

        if simulation:
            self.K_p = 1.2    
            self.K_d = 0.9
        else:
            # too high!! try Kp 3 to 5
            self.K_p = 2 #6 # 2
            self.K_d = 1#5# 1

        self._prev_error = 0.0

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
        clean_scan = self._clean_lidar_data(z_scan)

        # extract from the scan array the set of values we need

        d_front, d_left, d_right = self._get_sensor_readings(clean_scan)

        # Update control variables
        distance_to_current_wall, distance_to_other_wall = self._get_wall_distances(d_left, d_right)

        # state machine logic: transitions
        self._update_state_machine(
            d_front, distance_to_current_wall, distance_to_other_wall, d_left, d_right, z_w
        )

        # state machine logic: definition of each state
        v_cmd, w_cmd= self._compute_actions_based_on_state(distance_to_current_wall)
        # contrains to respect the phisical limits of the robot
        v, w = self._saturate_commands(v_cmd, w_cmd)

        if self._logger is not None:
            self._logger.info(
                f"[{self._state.name}] Side:{'R' if self._side_sign == 1 else 'L'} | Front:{d_front:.2f} | Right:{d_right:.2f} |  Left: {d_left:.2f} | w:{w:.2f}"
            )


        if not self._simulation:
            w = -w

        return v, w

    def _clean_lidar_data(self, z_scan: list[float]) -> list[float]:
        """Filters invalid LiDAR readings (NaN, Inf, 0.0) by replacing them with max range.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].

        Returns:
            A clean list of distances.
        """

        if self._simulation:
            clean_scan = []
            for r in z_scan:
                if math.isnan(r) or math.isinf(r) or r == 0.0:
                    # if we get any value that doesnt make sense we set it to the senor max
                    clean_scan.append(self.SENSOR_RANGE_MAX)
                else:
                    clean_scan.append(r)
            return clean_scan
        else:
            return list(z_scan)
        
    

    def _get_robust_min(self, values: list[float]) -> float:
       
        if self._simulation:
            valid_values = values
        else:
            valid_values = [v for v in values if not math.isnan(v) and not math.isinf(v) and v > 0.0]
        
  
        if not valid_values: 
            return self.SENSOR_RANGE_MAX
        
        k = min(len(valid_values), 5) 
        sorted_vals = sorted(valid_values)
        return sum(sorted_vals[:k]) / k

    def _get_sensor_readings(self, scan: list[float]) -> tuple[float, float, float]:
        """Extracts specific ranges of distance from the Lidar scan.

        Args:
            scan: The cleaned lidar distance from every LiDAR ray to the closest obstacle [m].
        Returns:
            A tuple containing the minimum distances for three main directions:
                d_front: Distance to the closest obstacle in front [m].
                d_left: Distance to the closest obstacle on the left [m].
                d_right: Distance to the closest obstacle on the right [m].
        """

    
        if self._simulation:
            d_right = min(scan[160:200])
            d_front = min(scan[-5:] + scan[:5])
            d_left = min(scan[40:80])
            return d_front, d_left, d_right

        else:
            n = len(scan)
            if n == 0: 
                return self.SENSOR_RANGE_MAX, self.SENSOR_RANGE_MAX, self.SENSOR_RANGE_MAX 

            
            #  +/- 20 degrees range for left and right
            SIDE_APERTURE = 20
            #  +/- 10 degree range for front
            FRONT_APERTURE = 5 

            # convert degrees into number of indexes/ lidar rays"
            rays_per_degree = n / 360.0
            
            side_width = int(SIDE_APERTURE * rays_per_degree)
            front_width = int(FRONT_APERTURE * rays_per_degree)

            fw = max(1, front_width)
            d_front = self._get_robust_min(scan[-fw:] + scan[:fw])

            idx_left = int(n / 4)
            d_left = self._get_robust_min(scan[idx_left - side_width : idx_left + side_width])

            idx_right = int(3 * n / 4)
            d_right = self._get_robust_min(scan[idx_right - side_width : idx_right + side_width])
            return d_front, d_left, d_right

    def _get_wall_distances(self, d_left: float, d_right: float) -> tuple[float, float]:
        """Determines distances to the followed wall and the opposite wall.

        Args:
            d_left: Minimum distance to the left wall [m].
            d_right: Minimum distance to the right wall [m].

        Returns:
            A tuple containing:
                distance_to_current_wall: The distance to the wall currently being followed [m].
                distance_to_other_wall: The distance to the wall on the opposite side [m].
        """
        if self._side_sign == 1:
            return d_right, d_left
        return d_left, d_right

    def _update_state_machine(
        self,
        d_front: float,
        distance_to_current_wall: float,
        distance_to_other_wall: float,
        d_left: float,
        d_right: float,
        z_w: float,
    ) -> None:
        """Updates the robot's state and side-following logic based on inputs from de sensor.

        Args:
            d_front: Distance to the closest obstacle in front [m].
            distance_to_current_wall: Distance to the wall currently being followed [m].
            distance_to_other_wall: Distance to the opposite wall [m].
            d_left: Distance to the left wall [m].
            d_right: Distance to the right wall [m].
            z_w: Current angular velocity [rad/s].
        """

        # --- Initial Decision ---
        if self._state == State.DECIDE_SIDE:
            if d_left < d_right:
                self._side_sign = -1
            else:
                self._side_sign = 1
            self._state = State.FIND_WALL
            return

        # ---  Side Switching ---
        # block change if the robot is turning
        if self._state != State.CORNER:
            should_switch = False
            if self._simulation:
                if distance_to_current_wall > distance_to_other_wall:
                    should_switch = True
            else:
                # only switches wall if the other wall is clearly a better option to follow
                HYSTERESIS = 0.1 # 
                
                if distance_to_other_wall < (distance_to_current_wall - HYSTERESIS):
                    should_switch = True

            if should_switch:
                self._side_sign *= -1 

                
                self._prev_error = 0.0
                self._state = State.FOLLOW_WALL
                return

        # --- State Transitions ---
        if self._state == State.CORNER:
            # robot has tu turn 90º before beign allowed to exir this state
            if self._simulation:
                self._angle_turned += abs(z_w) * self._dt
            else:
                self._angle_turned += 0.5 * self._dt
            
            if self._angle_turned >= self.TURN_ANGLE_TARGET:
                self._state = State.FIND_WALL
                self._prev_error = 0.0
                self._angle_turned = 0.0
        else:
            if d_front < self.MAX_FRONT_DISTANCE:
                # last check before turning?
                # Before turning blindly, loook which side has more space IN REALIYY
               
                if not self._simulation:
                    if d_left < d_right:
                        # less space left , then wall at left -> follow left -> continue right 
                        # Si hay menos hueco a la izq, la pared está a la izq -> sigo izq -> giro derecha
                        self._side_sign = -1 
                    elif d_left > d_right:
                        # the same but with the right wall 
                        self._side_sign = 1
                self._state = State.CORNER
                self._angle_turned = 0.0  # Reset integrator
                self._prev_error = 0.0
                return

            elif distance_to_current_wall < 1.0:
                # if the robot is not near a front wall, and it can detect a side wall
                self._state = State.FOLLOW_WALL
            else:
                self._state = State.FIND_WALL
        return 
    def _compute_actions_based_on_state(
        self, distance_to_current_wall: float
    ) -> tuple[float, float]:
        """Calculates the target linear and angular velocities based on the current state.

        Args:
            distance_to_current_wall: Distance to the wall currently being followed [m].

        Returns:
            A tuple containing the velocity commands:
                v: Linear velocity command [m/s].
                w: Angular velocity command [rad/s].
        """
        v = 0.0
        w = 0.0

        if self._state == State.DECIDE_SIDE:
            return 0.0, 0.0

        elif self._state == State.CORNER:
            v = 0.0
            # negative angular velocity turns the robot to the left
            # if we are following the right wall = 1 -> -0.5*1 = -0.5 the robot will make a turn to the left
            # if we are following the left wall = -1 -> -0.5*-1=0.5 the robot will make a turn to the right
            w = -0.5 * self._side_sign
            self._prev_error = 0.0

        elif self._state == State.FOLLOW_WALL:
            v = self.VEL_REF
            error = self.DIST_REF - distance_to_current_wall

            derivative = (error - self._prev_error) / self._dt
            # w = -self._side_sign * (self.K_p * error + self.K_d * derivative)
            pid_output = -self._side_sign * (self.K_p * error + self.K_d * derivative)


            PID_LIMIT = 0.5 
            
            if pid_output > PID_LIMIT:
                w = PID_LIMIT
            elif pid_output < -PID_LIMIT:
                w = -PID_LIMIT
            else:
                w = pid_output

            self._prev_error = error

        elif self._state == State.FIND_WALL:
            # in case the robot can't detect wall
            v = 0.08
            w = 0.45 * self._side_sign
            self._prev_error = 0.0
        

        return v, w

    def _saturate_commands(self, v: float, w: float) -> tuple[float, float]:
        """Applies kinematic constraints to keep velocity commands within robot limits.

        Args:
            v: linear velocity [m/s].
            w: angular velocity [rad/s].

        Returns:
            A tuple containing the possible velocity commands:
                v: limited linear velocity [m/s].
                w: limited angular velocity [rad/s].
        """
        b = self.TRACK / 2.0
        r = self.WHEEL_RADIUS

        w_limit = (self.WHEEL_SPEED_MAX * r - abs(v)) / b

        if abs(w) > w_limit:
            # prioritize angular velo   city, reduce linear velocity
            v_altered = (self.WHEEL_SPEED_MAX * r) - (abs(w) * b)

            if v_altered < 0:
                v = 0.0
                w = np.sign(w) * (self.WHEEL_SPEED_MAX * r / b)
            else:
                v = v_altered

        return v, w
