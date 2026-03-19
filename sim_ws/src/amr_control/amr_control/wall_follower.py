import math
import numpy as np
from enum import Enum, auto


class State(Enum):
    DECIDE_SIDE = auto()
    FIND_WALL = auto()
    ALIGN_WALL = auto()
    FOLLOW_WALL = auto()
    INNER_CORNER = auto()
    OUTER_CORNER = auto()


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
    )  # Angle the rpbot has to tu acomploish when in enters corner state

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
            self.K_p = 2
            self.K_d = 0.9
        else:
            # too high!! try Kp 3 to 5
            self.K_p = 2  # 6 # 2
            self.K_d = 1  # 5# 1

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

        # d_front, d_left, d_right = self._get_sensor_readings(clean_scan)
        d_front, d_left, d_right, d_front_left, d_front_right = self._get_sensor_readings(
            clean_scan
        )

        # Update control variables
        distance_to_current_wall, distance_to_other_wall = self._get_wall_distances(d_left, d_right)

        # state machine logic: transitions
        # self._update_state_machine(
        # d_front, distance_to_current_wall, distance_to_other_wall, d_left, d_right, z_w
        # )
        self._update_state_machine(
            d_front,
            distance_to_current_wall,
            distance_to_other_wall,
            d_left,
            d_right,
            d_front_left,
            d_front_right,
            z_w,
        )

        # state machine logic: definition of each state
        v_cmd, w_cmd = self._compute_actions_based_on_state(distance_to_current_wall)
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
            valid_values = [
                v for v in values if not math.isnan(v) and not math.isinf(v) and v > 0.0
            ]

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
            # El LiDAR tiene 241 valores. 1 índice = 1.5 grados.

            # FRENTE (Índice 0 y 240) -> +/- 7.5 grados
            d_front = min(scan[-5:] + scan[:5])

            # LATERAL IZQUIERDO (90º) -> Índice 60 (+/- 15 grados)
            d_left = min(scan[50:70])

            # LATERAL DERECHO (270º) -> Índice 180 (+/- 15 grados)
            d_right = min(scan[170:190])

            # DIAGONAL FRONT-IZQ (45º) -> Índice 30 (+/- 15 grados)
            d_front_left = min(scan[20:40])

            # DIAGONAL FRONT-DER (315º) -> Índice 210 (+/- 15 grados)
            d_front_right = min(scan[200:220])

            return d_front, d_left, d_right, d_front_left, d_front_right

        else:
            n = len(scan)
            if n == 0:
                return (
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                    self.SENSOR_RANGE_MAX,
                )

            #  +/- 20 degrees range for left and right
            SIDE_APERTURE = 20
            #  +/- 10 degree range for front
            FRONT_APERTURE = 5
            # +/- 15 degree for diagonals
            DIAG_APERTURE = 15

            rays_per_degree = n / 360.0

            side_width = int(SIDE_APERTURE * rays_per_degree)
            front_width = int(FRONT_APERTURE * rays_per_degree)
            diag_width = int(DIAG_APERTURE * rays_per_degree)

            fw = max(1, front_width)
            d_front = self._get_robust_min(scan[-fw:] + scan[:fw])

            idx_left = int(n / 4)  # 90 grados
            d_left = self._get_robust_min(scan[idx_left - side_width : idx_left + side_width])

            idx_right = int(3 * n / 4)  # 270 grados
            d_right = self._get_robust_min(scan[idx_right - side_width : idx_right + side_width])

            idx_front_left = int(n / 8)  # 45 grados
            d_front_left = self._get_robust_min(
                scan[idx_front_left - diag_width : idx_front_left + diag_width]
            )

            idx_front_right = int(7 * n / 8)  # 315 grados
            d_front_right = self._get_robust_min(
                scan[idx_front_right - diag_width : idx_front_right + diag_width]
            )

            return d_front, d_left, d_right, d_front_left, d_front_right

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
        d_front_left: float,  # <--- NUEVO
        d_front_right: float,  # <--- NUEVO
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

            if self._logger:
                self._logger.info(
                    f"DECISION: Seguimos la pared {'DERECHA' if self._side_sign == 1 else 'IZQUIERDA'}"
                )
            return

        # --- SeCURITY -> stop robot if wall to close ---
        if self._state not in [State.INNER_CORNER, State.ALIGN_WALL]:
            if d_front < self.MAX_FRONT_DISTANCE:
                # ¡LA MAGIA DE LA INTERSECCIÓN ESTÁ AQUÍ!
                # Antes de girar, decidimos qué lado nos conviene seguir
                # para asegurarnos de girar hacia el lado abierto.
                if d_left > d_right:
                    # Hay más espacio a la izq. Queremos girar a la izq.
                    # Para que INNER_CORNER gire a la izq, le decimos que la pared está a la der.
                    self._side_sign = 1
                else:
                    self._side_sign = -1

                self._state = State.INNER_CORNER
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info(
                        f"EMERGENCIA: Frente a {d_front:.2f}m. "
                        f"Espacio: L={d_left:.2f}, R={d_right:.2f}. "
                        f"Cambio forzado a Side: {'R' if self._side_sign == 1 else 'L'}"
                    )
                return

        # --- SIDE SWITCHING (Exploración dinámica) ---
        # Solo permitimos cambiar de lado si estamos en modos "tranquilos"
        if self._state in [State.FOLLOW_WALL, State.FIND_WALL]:
            should_switch = False

            if self._simulation:
                # if distance_to_other_wall < distance_to_current_wall:
                #     should_switch = True
                HYSTERESIS = 0.15  # 15 cm de margen evita que oscile en pasillos
                if distance_to_other_wall < (distance_to_current_wall - HYSTERESIS):
                    should_switch = True
            else:
                HYSTERESIS = 0.15  # 15 cm de margen evita que oscile en pasillos
                if distance_to_other_wall < (distance_to_current_wall - HYSTERESIS):
                    should_switch = True

            if should_switch:
                self._side_sign *= -1
                self._prev_error = 0.0

                # Si cambiamos de lado, lo más seguro es forzar una alineación
                # con la nueva pared antes de correr
                self._state = State.ALIGN_WALL

                if self._logger:
                    self._logger.info(
                        f"CAMBIO DE LADO: La pared {'derecha' if self._side_sign == 1 else 'izquierda'} es mejor. Alineando..."
                    )
                return

        # --- Find wall ---
        if self._state == State.FIND_WALL:
            # Si encontramos CUALQUIER pared cerca, empezamos a alinearnos
            WALL_FOUND_VAL = 0.3
            if distance_to_current_wall < WALL_FOUND_VAL or d_front < WALL_FOUND_VAL:
                self._state = State.ALIGN_WALL
                if self._logger:
                    self._logger.info(
                        f"PARED ENCONTRADA a {distance_to_current_wall:.2f}m. Iniciando alineación."
                    )
            return

        # ---- align with wall ---

        if self._state == State.ALIGN_WALL:
            # Seleccionamos qué diagonal vigilar en base a la pared que seguimos
            d_diagonal = d_front_right if self._side_sign == 1 else d_front_left

            # MAGIA GEOMÉTRICA:
            # 1. El frente debe estar limpio (> 0.5)
            # 2. La pared no se nos ha escapado (< 0.6)
            # 3. La diagonal es al menos un 20% mayor que el lateral puro.
            #    Esto garantiza matemáticamente que el robot ya no mira hacia la pared.
            is_parallel = d_diagonal > (distance_to_current_wall * 1.2)

            if d_front > 0.5 and distance_to_current_wall < 0.6 and is_parallel:
                self._state = State.FOLLOW_WALL
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info(
                        f"ALINEADO PERFECTO: Frente={d_front:.2f}, Lateral={distance_to_current_wall:.2f}, Diagonal={d_diagonal:.2f}"
                    )
            else:
                if self._logger:
                    self._logger.info(
                        f"Alineando... Lat={distance_to_current_wall:.2f}, Diag={d_diagonal:.2f}"
                    )
            return
        # --- Follow wall ---

        if self._state == State.FOLLOW_WALL:
            # Condición A: La pared lateral desaparece repentinamente (Intersección abierta)
            if distance_to_current_wall > 0.6:
                self._state = State.OUTER_CORNER
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info(
                        f"PARED PERDIDA: Distancia lateral {distance_to_current_wall:.2f}m. Envolviendo esquina exterior..."
                    )
            # (La Condición B de chocar de frente ya está cubierta por la lógica de Seguridad Global arriba)
            return

        # ---- inner corner ---
        if self._state == State.INNER_CORNER:
            # Condición de salida: Hemos girado lo suficiente como para que el frente vuelva a estar despejado
            if d_front > 0.45:
                # Ya estamos alineados con la nueva pared
                # self._state = State.FOLLOW_WALL
                self._state = State.ALIGN_WALL
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info("ESQUINA INTERIOR: Frente despejado. Retomando seguimiento.")
            return

        # ---- outer corner ----
        if self._state == State.OUTER_CORNER:
            # Condición de salida: Al trazar el arco, volvemos a detectar la pared en el lateral
            if distance_to_current_wall < 0.8:
                self._state = State.FOLLOW_WALL
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info(
                        f"ESQUINA exterior: Pared recuperada a {distance_to_current_wall:.2f}m. Retomando seguimiento."
                    )
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

        elif self._state == State.FIND_WALL:
            # Avanza lento, giro leve hacia el lado elegido para "buscar" la pared
            v = 0.1
            w = 0.3 * self._side_sign

        elif self._state == State.ALIGN_WALL:
            # Giro sobre el eje central (pivotar). v=0 garantiza seguridad.
            # Giramos alejándonos de la pared
            v = 0.0
            w = -0.5 * self._side_sign

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

        elif self._state == State.INNER_CORNER:
            # Pivotar bruscamente para evitar la colisión frontal (rotación pura)
            v = 0.0
            w = -0.5 * self._side_sign

        elif self._state == State.OUTER_CORNER:
            # Trazar un arco ciego envolviendo la esquina.
            # Avanza moderadamente y gira hacia la pared "perdida"
            v = 0.12  # Velocidad algo reducida para no abrirse demasiado
            w = 0.5 * self._side_sign  # Gira HACIA el lado de la pared

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
