import math
import numpy as np


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22 
    SENSOR_RANGE_MIN = 0.16 
    SENSOR_RANGE_MAX = 8.0 
    TRACK = 0.16 
    WHEEL_RADIUS = 0.033 
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS 

    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        self._dt: float = dt
        self._logger = logger
        self._simulation: bool = simulation

       
        self.K_p = 1.2
        self.K_d = 0.6 
        

        self._prev_error = None 

        self._dist_ref = 0.2
        self._vel_ref = 0.15

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        
        # 1. LIMPIEZA DE DATOS (CRÍTICO)
      
        clean_scan = []
        for r in z_scan:
    
            if math.isnan(r) or math.isinf(r) or r == 0.0:
                    clean_scan.append(self.SENSOR_RANGE_MAX)
            else:
                  
                  
                    clean_scan.append(r)
        
        # 2. LECTURA (Solo nos importa la pared derecha)
        d_right_wall = min(clean_scan[160:200])

        # 3. CONTROL PD PURO (Sin máquina de estados)

        v = self._vel_ref
        
  
        error = self._dist_ref - d_right_wall
        

        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / self._dt
 
        # FÓRMULA ( robot invertido: Right=+, Left=-)
        # Si Error es Negativo (Lejos) -> Queremos ir derecha (+) -> (-) * (-) = (+) OK.
        # Si Error es Positivo (Cerca) -> Queremos ir izquierda (-) -> (-) * (+) = (-) OK.
        w = - (self.K_p * error + self.K_d * derivative)

        self._prev_error = error
        
        # 4. SATURACIÓN (Física del robot)

        b = self.TRACK / 2.0
        r = self.WHEEL_RADIUS
        
        w_limit = (self.WHEEL_SPEED_MAX * r - abs(v)) / b
        
        if abs(w) > w_limit:
            v_altered = (self.WHEEL_SPEED_MAX * r) - (abs(w) * b)
            if v_altered < 0: 
                v = 0.0
                w = np.sign(w) * (self.WHEEL_SPEED_MAX * r / b) 
            else:
                v = v_altered
        

        if self._logger is not None:
             self._logger.info(f"MODO TEST PD | Wall: {d_right_wall:.2f} | Err: {error:.2f} | v: {v:.2f}, w: {w:.2f}")

        return v, w