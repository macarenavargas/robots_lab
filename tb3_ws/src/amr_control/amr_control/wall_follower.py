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
            self.K_d = 1.3
        else:
            # too high!! try Kp 3 to 5
            self.K_p = 2  # 6 # 2
            self.K_d = 1  # 5# 1

        self._prev_error = 0.0

        self._dist_ref = 0.2
        self._vel_ref = 0.15

        # Distancias de seguridad y transición (m)
        self.DIST_FRONT_LIMIT = 0.25  # Umbral de colisión inminente
        self.DIST_FRONT_SAFE = 0.6  # Espacio necesario para dejar de pivotar
        self.DIST_SIDE_REF = 0.20  # Distancia objetivo (tu DIST_REF)
        self.DIST_WALL_LOST = 0.50  # Umbral para considerar que la pared ha terminado
        self.DIST_WALL_CAPTURE = 0.35  # Distancia para "enganchar" una nueva pared
        self.DIST_SIDE_MAX = 0.35  # Límite para seguir una pared
        self.PARALLEL_MARGIN = 1.2  # Tolerancia de paralelismo (10%)
        self.SIDE_HYSTERESIS = 0.1  # Evita oscilaciones entre paredes
        # Parámetros de comportamiento (m/s y rad/s)
        self.V_SEARCH = 0.10
        self.W_SEARCH = 0.30

        self.W_ALIGN = 0.30  # Reducido de 0.5 para mayor estabilidad del sensor
        self.W_INNER = 0.50
        self.V_OUTER = 0.12
        self.W_OUTER = 0.50

        self.W_LIMIT_PID = 0.05  # 0.05

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
        v_cmd, w_cmd = self._compute_actions_based_on_state(distance_to_current_wall, d_front)
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
        self, d_front, d_current, d_other, d_left, d_right, d_diag_l, d_diag_r, z_w
    ) -> None:
        """
        Estructura jerarquica:
        1. Seguridad Critica
        2. Gestion de Transiciones segun Estado Actual
        """

        # --- 0. DECIDE SIDE ---
        if self._state == State.DECIDE_SIDE:
            self._side_sign = -1 if d_left < d_right else 1
            self._state = State.FIND_WALL
            if self._logger:
                side_str = "DERECHA" if self._side_sign == 1 else "IZQUIERDA"
                self._logger.info(f"MODO INICIAL: Referencia pared {side_str}")
            return

        # --- A. SEGURIDAD CRITICA (Pre-emptive) ---
        if self._state not in [State.INNER_CORNER]:
            if d_front < self.DIST_FRONT_LIMIT:
                # Decidimos lado de escape basado en el espacio disponible actual
                self._side_sign = 1 if d_left > d_right else -1
                self._state = State.INNER_CORNER
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info(
                        f"SEGURIDAD: Obstaculo a {d_front:.2f}m. Pivotando hacia {'DERECHA' if self._side_sign == 1 else 'IZQUIERDA'}"
                    )
                return

        # --- B. TRANSICIONES DE ESTADO (Consolidadas) ---

        # 1. MODO BUSQUEDA
        if self._state == State.FIND_WALL:
            if d_current < self.DIST_WALL_CAPTURE or d_front < self.DIST_WALL_CAPTURE:
                self._state = State.ALIGN_WALL
            # Si estamos buscando y la pared aparece por el lado contrario, la agarramos
            elif d_other < self.DIST_WALL_CAPTURE:
                self._side_sign *= -1
                self._state = State.ALIGN_WALL
                if self._logger:
                    self._logger.info("BUSQUEDA: Pared encontrada en el lado opuesto. Cambiando.")

        # 2. MODO SEGUIMIENTO NOMINAL
        elif self._state == State.FOLLOW_WALL:
            # Caso: Perdemos la pared que seguiamos
            if d_current > self.DIST_WALL_LOST:
                if d_other < self.DIST_WALL_CAPTURE:
                    self._side_sign *= -1
                    self._state = State.ALIGN_WALL
                    if self._logger:
                        self._logger.info(
                            "TOPOLOGIA: Cambiando a pared opuesta por perdida de actual"
                        )
                else:
                    self._state = State.OUTER_CORNER
                    if self._logger:
                        self._logger.info(
                            "TOPOLOGIA: Pared perdida sin alternativa. Rodeando esquina"
                        )

            # Caso: Existe una pared mucho mejor (Side Switching)
            elif d_other < (d_current - self.SIDE_HYSTERESIS) and d_other < self.DIST_SIDE_MAX:
                self._side_sign *= -1
                self._state = State.FOLLOW_WALL
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info("OPTIMIZACION: Cambio de lado por proximidad")

        # 3. MODO ALINEACION (Trigonometria Lidar)
        elif self._state == State.ALIGN_WALL:
            d_diag = d_diag_r if self._side_sign == 1 else d_diag_l
            # is_parallel = d_diag > (d_current * self.PARALLEL_MARGIN)
            is_parallel = (d_current * 1.1) < d_diag < (d_current * 1.6)

            # Escape 1: ¡NUEVO! Abortar si la pared desaparece en mitad del giro
            if d_current > self.DIST_WALL_LOST:
                self._state = State.FIND_WALL
                if self._logger:
                    self._logger.info("ABORTAR ALINEACION: La pared desapareció durante el giro")
                return

            # Salida con Éxito (Ya lo tenías)
            if (
                d_front > self.DIST_FRONT_SAFE
                and d_current < (self.DIST_WALL_LOST + 0.05)
                and is_parallel
            ):
                self._state = State.FOLLOW_WALL
                self._prev_error = 0.0
                if self._logger:
                    self._logger.info("ESTADO: FOLLOW_WALL. Alineacion correcta")

        # 4. MODO ESQUINA INTERIOR
        elif self._state == State.INNER_CORNER:
            if d_front > self.DIST_FRONT_SAFE:
                self._state = State.ALIGN_WALL
                if self._logger:
                    self._logger.info("ESTADO: ALIGN_WALL. Frente libre")

        # 5. MODO ESQUINA EXTERIOR
        elif self._state == State.OUTER_CORNER:
            # Si el frente se bloquea mientras rodeamos, prioridad a la seguridad
            if d_front < self.DIST_FRONT_LIMIT:
                self._state = State.INNER_CORNER
                return

            if d_current < self.DIST_WALL_CAPTURE:
                self._state = State.FOLLOW_WALL
                if self._logger:
                    self._logger.info("ESTADO: FOLLOW_WALL. Esquina envuelta")

    def _compute_actions_based_on_state(
        self,
        distance_to_current_wall: float,
        d_front: float,  # <-- Añadir aquí
    ) -> tuple[float, float]:
        """
        Calcula las consignas de velocidad v y w basándose exclusivamente en el estado actual.
        Implementa leyes de control diferenciadas para navegación, alineación y emergencia.
        """
        v = 0.0
        w = 0.0

        # MODO: Reposo
        if self._state == State.DECIDE_SIDE:
            return 0.0, 0.0

        # MODO: Búsqueda (Arco amplio para cubrir área)
        elif self._state == State.FIND_WALL:
            v = self.V_SEARCH
            w = self.W_SEARCH * self._side_sign

        # MODO: Alineación (Rotación pura para precisión del LiDAR)
        elif self._state == State.ALIGN_WALL:
            v = 0.0
            # Reducimos velocidad angular para que el LiDAR no sufra de motion blur
            w = -self.W_ALIGN * self._side_sign

        # MODO: Seguimiento (Control PD sobre el error de distancia)
        elif self._state == State.FOLLOW_WALL:
            v = self.VEL_REF

            # PID normal (el código que ya tenías)
            error = self.DIST_REF - distance_to_current_wall

            derivative = (error - self._prev_error) / self._dt
            pid_output = -self._side_sign * (self.K_p * error + self.K_d * derivative)
            w = np.clip(pid_output, -self.W_LIMIT_PID, self.W_LIMIT_PID)
            self._prev_error = error

        # MODO: Esquina Interior (Evitación de colisión frontal)
        elif self._state == State.INNER_CORNER:
            v = 0.0
            w = -self.W_INNER * self._side_sign

        # MODO: Esquina Exterior (Maniobra de envolvimiento)
        elif self._state == State.OUTER_CORNER:
            v = self.V_OUTER
            w = self.W_OUTER * self._side_sign

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
