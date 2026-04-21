import math
import numpy as np
from enum import Enum, auto


class State(Enum):
    MOVE_AHEAD = auto()
    FOLLOW_WALL = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()


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

        self._state = State.MOVE_AHEAD
        self._side_sign = 1  # 1 = Right, -1 = Left

        if simulation:
            self.K_p = 2  # 1.7 2
            self.K_d = 1  # 2.2
        else:
            # too high!! try Kp 3 to 5
            self.K_p = 2  # 6 # 2
            self.K_d = 1  # 5# 1

        self._prev_error = 0.0

        self._dist_ref = 0.2
        self._vel_ref = 0.15
        self._d_current = None

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

        # STEP 1: clean de z_scan array to deal with nan and inf values
        clean_scan = self._clean_lidar_data(z_scan)

        # STEP 2: extract from the scan array the set of values we need
        d_front, d_left, d_right = self._get_sensor_readings(clean_scan)

        # STEP 3: update the distance to wall we are currently following
        if self._side_sign == 1:
            self._d_current = d_right
        else:
            self._d_current = d_left

        # STEP 3: update the state machine
        self._update_state_machine(d_front, d_left, d_right)

        # STEP 4: determine what action to take based ont he current state
        v_cmd, w_cmd = self._compute_actions_based_on_state()

        # STEP 5: apply contrains to respect the physical limits of the robot
        v, w = self._saturate_commands(v_cmd, w_cmd)

        if self._logger is not None:
            self._logger.info(
                f"[{self._state.name}] Side:{'R' if self._side_sign == 1 else 'L'} "
                f"| Front:{d_front:.2f} | Right:{d_right:.3f} |  Left: {d_left:.3f} | w:{w:.2f}"
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
        safe_value = 0.9 * self.SENSOR_RANGE_MIN

        clean_scan = [
            r if not (math.isnan(r) or math.isinf(r) or r <= 0.0) else safe_value
            for r in z_scan
        ]
        return clean_scan

    def _get_robust_min(self, values: list[float]) -> float:
        k = min(len(values), 5)
        sorted_vals = sorted(values)
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
        SIM_TOTAL_RAYS = 240.0
        SIM_DEGREES_PER_RAY = 360.0 / SIM_TOTAL_RAYS

        if self._simulation:
            sim_front_rays_half = 20  # +/- 20 rays from the front center of the robot
            sim_side_rays_half = 10  # +/- 10 rays from the center of each side (90º y 270º)
        else:
            sim_front_rays_half = 15  # +/- 20 rays from the front center of the robot
            sim_side_rays_half = 10  # +/- 10 rays from the center of each side (90º y 270º)

        if self._simulation:
            # STEP 1: extract minimum distance from the front section
            d_front = min(scan[-sim_front_rays_half:] + scan[:sim_front_rays_half])

            # STEP 2: extract minimum distance from the left section (center at 90º)
            idx_left = int(SIM_TOTAL_RAYS / 4)  # index 60
            d_left = min(scan[idx_left - sim_side_rays_half : idx_left + sim_side_rays_half + 1])

            # STEP 3: extract minimum distance from the right section (center at 270º)
            idx_right = int(3 * SIM_TOTAL_RAYS / 4)  # index 180
            d_right = min(scan[idx_right - sim_side_rays_half : idx_right + sim_side_rays_half + 1])

            return d_front, d_left, d_right

        else:
            n = len(scan)
            if n == 0:
                return (
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                )

            # STEP 1: transform simualtion rays into aperture degrees
            front_aperture_deg = sim_front_rays_half * SIM_DEGREES_PER_RAY
            side_aperture_deg = sim_side_rays_half * SIM_DEGREES_PER_RAY

            # STEP 2. Calculate how many rays in the real robot are equivalent to those degrees
            real_rays_per_degree = n / 360.0
            real_front_width = int(front_aperture_deg * real_rays_per_degree)
            real_side_width = int(side_aperture_deg * real_rays_per_degree)

            # STEP 3: extract minimum distance from each section
            fw = max(1, real_front_width)
            front = scan[-fw:] + scan[:fw]
            valid_front = [v for v in front if v > 0.145]
            d_front = min(valid_front) if valid_front else 0.145

            idx_left = int(n / 4)
            left = scan[idx_left - real_side_width : idx_left + real_side_width + 1]
            valid_left = [v for v in left if v > 0.145]
            d_left = min(valid_left) if valid_left else 0.145

            idx_right = int(3 * n / 4)
            right = scan[idx_right - real_side_width : idx_right + real_side_width + 1]
            valid_right = [v for v in right if v > 0.145]
            d_right = min(valid_right) if valid_right else 0.145

            return d_front, d_left, d_right

    def _update_state_machine(self, d_front, d_left, d_right) -> None:
        enter_turn_distance = 0.25
        exit_turn_distance = 0.4
        exit_move_ahead_distance = 0.25
        enter_move_ahead_distance = 0.4

        # STATE MOVE AHEAD:
        # -> enter (from FOLLOW_WALL): don't have a wall thats close enigh to follow
        # <- leave (TO FOLLOW_WALL): found a wall to follow
        if self._state == State.MOVE_AHEAD:
            if self._simulation:
                # is there a wall close enough to follow?
                if d_left < exit_move_ahead_distance or d_right < exit_move_ahead_distance:
                    # check which wall is closer
                    if d_left > d_right:
                        self._side_sign = 1
                        if self._logger:
                            self._logger.info("Wall found on the RIGHT. Switching to FOLLOW_WALL.")
                    else:
                        self._side_sign = -1
                        if self._logger:
                            self._logger.info("Wall found on the LEFT. Switching to FOLLOW_WALL.")
                    self._state = State.FOLLOW_WALL
                    self._prev_error = 0.0
            else:
                if d_front < enter_turn_distance:
                    # decide which side to turn to
                    if self._side_sign == 1:
                        self._state = State.TURN_LEFT
                    elif self._side_sign == -1:
                        self._state = State.TURN_RIGHT

                    if self._logger:
                        self._logger.info(
                            f"Frontal obstacle detected at {d_front:.2f}m. Executing CORNER maneuver."
                        )

                # is there a wall close enough to follow?
                elif d_left < exit_move_ahead_distance or d_right < exit_move_ahead_distance:
                    # check which wall is closer
                    if d_left > d_right:
                        self._side_sign = 1
                        if self._logger:
                            self._logger.info("Wall found on the RIGHT. Switching to FOLLOW_WALL.")
                    else:
                        self._side_sign = -1
                        if self._logger:
                            self._logger.info("Wall found on the LEFT. Switching to FOLLOW_WALL.")
                    self._state = State.FOLLOW_WALL
                    self._prev_error = 0.0

        # STATE FOLLOW_WALL:
        # -> enter (from MOVE_AHEAD): there is a wall close enough to follow
        # <- leave (to TURN_LEFT or TURN_RIGHT): frontal obstacle detected
        # <- leave (to MOVE_AHEAD): current wall the robot was following is lost
        elif self._state == State.FOLLOW_WALL:
            # is there an obstacle close enough?
            if d_front < enter_turn_distance:
                # decide which side to turn to
                if d_left > d_right:
                    self._side_sign = 1
                    self._state = State.TURN_LEFT
                else:
                    self._side_sign = -1
                    self._state = State.TURN_RIGHT

                if self._logger:
                    self._logger.info(
                        f"Frontal obstacle detected at {d_front:.2f}m. Executing CORNER maneuver."
                    )
            # have we lost the current wlal we where following?
            elif (self._side_sign == 1 and d_right > enter_move_ahead_distance) or (
                self._side_sign == -1 and d_left > enter_move_ahead_distance
            ):
                if self._logger:
                    self._logger.info("WALL LOST. Switching to MOVE_AHEAD.")
                self._state = State.MOVE_AHEAD

        # STATE TURN_RIGHT or TURN_LEFT:
        # -> enter (from FOLLOW_WALL): frontal obstacle detected
        # <- leave (to MOVE_AHEAD): frontal obstacled cleared.
        elif self._state == State.TURN_RIGHT:
            # Is the robot's front clear?
            if d_front > exit_turn_distance:
                self._state = State.MOVE_AHEAD
                if self._logger:
                    self._logger.info(
                        f"Front cleared ({d_front:.2f}m). Maneuver complete. Switching to MOVE_AHEAD."
                    )
        elif self._state == State.TURN_LEFT:
            # Is the robot's front clear?
            if d_front > exit_turn_distance:
                self._state = State.MOVE_AHEAD
                if self._logger:
                    self._logger.info(
                        f"Front cleared ({d_front:.2f}m). Maneuver complete. Switching to MOVE_AHEAD."
                    )

    def _compute_actions_based_on_state(self) -> tuple[float, float]:

        v = 0.0
        w = 0.0

        turn_angular_velocity = 0.4

        # STATE MOVE_AHEAD: move in a straight line
        if self._state == State.MOVE_AHEAD:
            v = self._vel_ref
            w = 0.0

        # STATE FOLLOW_WALL: apply PD control to follow the wall
        elif self._state == State.FOLLOW_WALL:
            v = self._vel_ref

            error = self._dist_ref - self._d_current

            if self._prev_error == 0.0:
                self._prev_error = error

            derivative = (error - self._prev_error) / self._dt

            pid_output = -self._side_sign * (self.K_p * error + self.K_d * derivative)
            # w = np.clip(pid_output, -0.5, 0.5)
            w = pid_output
            self._prev_error = error

        # STATE TURN LEFT, TURN_RIGHT: stop and turn on the spot.
        elif self._state == State.TURN_LEFT:
            v = 0.0
            w = -turn_angular_velocity

        elif self._state == State.TURN_RIGHT:
            v = 0.0
            w = turn_angular_velocity
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
            # prioritize angular velocity, reduce linear velocity
            v_altered = (self.WHEEL_SPEED_MAX * r) - (abs(w) * b)

            if v_altered < 0:
                v = 0.0
                w = np.sign(w) * (self.WHEEL_SPEED_MAX * r / b)
            else:
                v = v_altered

        return v, w
