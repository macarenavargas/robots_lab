from enum import Enum, auto
import math


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    class State(Enum):
        STOPPED = auto()
        MOVE_STRAIGHT = auto()
        TURN = auto()

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

        self._state = self.State.STOPPED
        self._turn_direction = 0.0
        self.K_p = 2.0
        self.K_d = 1.5
        self._dist_ref = 0.5  # distance that the robot has too keep from the wall
        self._prev_error = 0.0  # memory for de derivative term

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

        # scan = [self.SENSOR_RANGE_MAX if math.isinf(d) else d for d in z_scan]

        # front_values = scan[-5:] + scan[:5]
        # left_side_values = scan[55:65]
        # right_side_values = scan[175:185]

        # valid_front = [v for v in front_values if not math.isnan(v)]
        # valid_left = [v for v in left_side_values if not math.isnan(v)]
        # valid_right = [v for v in right_side_values if not math.isnan(v)]

        # min_front = min(valid_front) if len(valid_front) > 0 else 0.0

        # min_left = min(valid_left) if len(valid_left) > 0 else 0.0

        # min_right = min(valid_right) if len(valid_right) > 0 else 0.0

        # 1. Limpieza de Infinitos
        scan = [self.SENSOR_RANGE_MAX if math.isinf(d) else d for d in z_scan]

        # 2. CÁLCULO DINÁMICO DE ÍNDICES (A prueba de fallos)
        n = len(scan)  # ¿Son 360? ¿Son 720? Da igual, esto se ajusta solo.

        # En ROS estándar: 0=Frente, n/4=Izq, n/2=Atrás, 3n/4=Derecha
        idx_front = 0
        idx_left = int(n * 0.25)  # Aprox 90 grados
        idx_right = int(n * 0.75)  # Aprox 270 grados

        # Definimos sectores de 10 grados aprox (5 rayos a cada lado)
        margin = 5

        # Ojo con el slice en Python si idx_right se sale, pero en medio del array es seguro
        # Frente (cruzando el cero)
        front_values = scan[-margin:] + scan[:margin]
        left_side_values = scan[idx_left - margin : idx_left + margin]
        right_side_values = scan[idx_right - margin : idx_right + margin]

        # 3. Filtrado de NaNs
        valid_front = [v for v in front_values if not math.isnan(v)]
        valid_left = [v for v in left_side_values if not math.isnan(v)]
        valid_right = [v for v in right_side_values if not math.isnan(v)]

        # 4. Mínimos seguros
        min_front = min(valid_front) if valid_front else 0.0
        min_left = min(valid_left) if valid_left else 0.0
        min_right = min(valid_right) if valid_right else 0.0

        v = 0.0
        w = 0.0

        if self._state == self.State.STOPPED:
            v = 0.0
            w = 0.0
            self._state = self.State.MOVE_STRAIGHT

        elif self._state == self.State.MOVE_STRAIGHT:
            v = 0.15
            # control PD formula to calculate w
            distance_to_right_wall = min_right
            error = self.DIST_REF - distance_to_right_wall
            derivative = (error - self._prev_error) / self._dt
            w = (self.K_p * error) + (self.K_d * derivative)

            self._prev_error = error

            # cinematic restrictions
            b = self.TRACK / 2
            w_max_admissible = (self.LINEAR_SPEED_MAX - abs(v)) / b  # maximum value w can take
            w_filtered = min(w, w_max_admissible)  # in case w exceeds the allowed fisical limit
            w = max(
                w_filtered, -w_max_admissible
            )  # apply the absolute value logic from the formula

            if min_front < 0.45:
                self._state = self.State.TURN
                if min_right < min_left:
                    self._turn_direction = -0.5
                else:
                    self._turn_direction = 0.5

        elif self._state == self.State.TURN:
            v = 0.0
            w = self._turn_direction

            if min_front > 0.65:
                self._state = self.State.MOVE_STRAIGHT
                self._prev_error = 0.0  # reset the error

        return v, w
