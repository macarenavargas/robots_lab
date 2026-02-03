from amr_simulation.robot import Robot
from typing import Any


class TurtleBot3Burger(Robot):
    """Class to control the Turtlebot3 Burger robot."""

    # Constants
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity [m/s]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    def __init__(self, sim: Any, dt: float) -> None:
        """Turtlebot3 Burger robot class initializer.

        Args:
            sim: CoppeliaSim simulation handle.
            dt: Sampling period [s].

        """
        Robot.__init__(self, sim=sim, track=self.TRACK, wheel_radius=self.WHEEL_RADIUS)
        self._dt: float = dt
        self._motors: dict[str, int] = self._init_motors()

        self.prev_right_wheel_angular_vel: float = 0.0
        self.prev_left_wheel_angular_vel: float = 0.0

    def move(self, v: float, w: float) -> None:
        """Solve inverse differential kinematics and send commands to the motors.

        If the target angular speed of any of the wheels is larger than the maximum admissible,
        both commands are ignored and the last valid set of wheel velocities, which are stored in
        class attributes, are sent instead.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        """
        # TODO: 2.1. Complete the function body with your code (i.e., replace the pass statement).

        b = self.TRACK / 2
        r = self.WHEEL_RADIUS
        right_wheel_angular_velocity = (v - b * w) / r  # phi r
        left_wheel_angular_velocity = (v + b * w) / r  # phi l

        if (
            right_wheel_angular_velocity > self.WHEEL_SPEED_MAX
            or left_wheel_angular_velocity > self.WHEEL_SPEED_MAX
        ):
            right_wheel_angular_velocity = self.prev_right_wheel_angular_vel
            left_wheel_angular_velocity = self.prev_left_wheel_angular_vel
        else:
            self.prev_right_wheel_angular_vel = right_wheel_angular_velocity
            self.prev_left_wheel_angular_vel = left_wheel_angular_velocity

        self._sim.setJointTargetVelocity(self._motors["right"], right_wheel_angular_velocity)
        self._sim.setJointTargetVelocity(self._motors["left"], left_wheel_angular_velocity)

        pass

    def sense(self) -> tuple[list[float], float, float]:
        """Read the LiDAR and the encoders.

        Returns:
            z_scan: Distance from every LiDAR ray to the closest obstacle in 1.5º increments [m].
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        # Read LiDAR
        packed_data: str = self._sim.getBufferProperty(self._sim.handle_scene, "signal.lidar")
        z_scan: list[float] = self._sim.unpackFloatTable(packed_data)

        # Return nan if the measurement failed
        z_scan = [z if z >= 0.0 else float("nan") for z in z_scan]

        # Read encoders
        z_v, z_w = self._sense_encoders()

        return z_scan, z_v, z_w

    def _init_motors(self) -> dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """
        motors: dict[str, int] = {}

        motors["left"] = self._sim.getObject("/leftMotor")
        motors["right"] = self._sim.getObject("/rightMotor")

        return motors

    def _sense_encoders(self) -> tuple[float, float]:
        """Solve forward differential kinematics from encoder readings.

        Returns:
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        # Read the angular position increment in the last sampling period [rad]
        encoders: dict[str, float] = {}

        encoders["left"] = self._sim.getFloatProperty(self._sim.handle_scene, "signal.leftEncoder")
        encoders["right"] = self._sim.getFloatProperty(
            self._sim.handle_scene, "signal.rightEncoder"
        )

        # TODO: 2.2. Compute the derivatives of the angular positions to obtain velocities [rad/s].

        right_wheel_angular_velocity = encoders["left"] / self._dt  # phi . r
        left_wheel_angular_velocity = encoders["right"] / self._dt  # phi . l

        # TODO: 2.3. Solve forward differential kinematics (i.e., calculate z_v and z_w).
        b = self.TRACK / 2
        r = self.WHEEL_RADIUS
        z_v = ((right_wheel_angular_velocity + left_wheel_angular_velocity) * r) / 2  # x. R
        z_w = ((right_wheel_angular_velocity - left_wheel_angular_velocity) * r) / (
            2 * b
        )  # theta . R

        return z_v, z_w
