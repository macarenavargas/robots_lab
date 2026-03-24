import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from amr_msgs.msg import MotionControl

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat

from amr_localization.particle_filter import ParticleFilter


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("global_localization", True)
        self.declare_parameter("initial_pose", (0.0, 0.0, math.radians(0)))
        self.declare_parameter("initial_pose_sigma", (0.05, 0.05, math.radians(5)))
        self.declare_parameter("particles", 1000)
        self.declare_parameter("sigma_v", 0.1)
        self.declare_parameter("sigma_w", 0.1)
        self.declare_parameter("sigma_z", 0.1)
        self.declare_parameter("simulation", False)
        self.declare_parameter("steps_btw_sense_updates", 10)
        self.declare_parameter("world", "lab03")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            global_localization = (
                self.get_parameter("global_localization").get_parameter_value().bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose").get_parameter_value().double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = self.get_parameter("particles").get_parameter_value().integer_value
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            self._steps_btw_sense_updates = (
                self.get_parameter("steps_btw_sense_updates").get_parameter_value().integer_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._localized = False
            self._x_pred = 0.0
            self._y_pred = 0.0
            self._theta_pred = 0.0


            self._steps = 0

            self._last_z_scan = None
            self._odom_measurements = []

            self._timer_period = 5

            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
                simulation=self._simulation,
                logger=self.get_logger(),  # Replace None with self.get_logger() to enable logging in the class
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            # TODO: 3.1. Create the /pose publisher (PoseStamped message).

            self._publisher_pose = self.create_publisher(
                msg_type=PoseStamped, topic="/pose", qos_profile=10
            )

            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            if self._simulation:
                # synchronized subscribers with join callback
                self._subscribers: list[message_filters.Subscriber] = []
                self._subscribers.append(
                    message_filters.Subscriber(
                        self, Odometry, "/odometry", qos_profile=scan_qos_profile
                    )
                )
                self._subscribers.append(
                    message_filters.Subscriber(
                        self, LaserScan, "/scan", qos_profile=scan_qos_profile
                    )
                )

                ts = message_filters.ApproximateTimeSynchronizer(
                    self._subscribers, queue_size=10, slop=9
                )
                ts.registerCallback(self._compute_pose_callback)

            else:
                # separate subscribers with individual call backs
                self._subscriber_odom = self.create_subscription(
                    Odometry,
                    "/odometry",
                    callback=self._odometry_callback,
                    qos_profile=scan_qos_profile,
                )
                self._subscriber_scan = self.create_subscription(
                    LaserScan, "/scan", callback=self._scan_callback, qos_profile=scan_qos_profile
                )

               
                # Motion Control publisher
                self._publisher_motion_control = self.create_publisher(
                    MotionControl, "/motion_control", qos_profile=10
                )
                # Timer
                self._timer = self.create_timer(self._timer_period, self._timer_callback)

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def _timer_callback(self):

        if self._last_z_scan is None:
            return
        # a) STOP THE ROBOT: publish the motionControl message to indicate the robot that it needs to stop
        motion_msg = MotionControl()
        motion_msg.allow_motion = False
        self._publisher_motion_control.publish(motion_msg)

        # b) MOVEMENT: execute as many motion steps as measurementes acumulated in odometry

        if len(self._odom_measurements) > 0:
            for z_v, z_w in self._odom_measurements:
                self._execute_motion_step(z_v, z_w)

        self._odom_measurements.clear()

        # c) MEASUREMENT: execute one single correction phase
        x_h, y_h, theta_h = self._execute_measurement_step(self._last_z_scan)

        # d) START THE ROBOT: publish the motion control message again to tell the robot to move
        motion_msg2 = MotionControl()
        motion_msg2.allow_motion = True
        self._publisher_motion_control.publish(motion_msg2)

        # e) PUBLISH POSE: publish the estimated pose
        self._publish_pose_estimate(x_h, y_h, theta_h)

        # RESTART THE TIMER: so that the robot moves the whole timer period
        self._timer.reset()

    def _scan_callback(self, scan_msg: LaserScan):
        self._last_z_scan = scan_msg.ranges

    def _odometry_callback(self, odom_msg: Odometry):
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z

        noise_threshold = 1e-3
        if abs(z_v) > noise_threshold or abs(z_w) > noise_threshold:
            self._odom_measurements.append((z_v, z_w))

        # lab 4 -> make a prediction with Euler 
        dt = self.get_parameter("dt").value
        self._x_pred += z_v * math.cos(self._theta_pred) * dt
        self._y_pred += z_v * math.sin(self._theta_pred) * dt
        self._theta_pred += z_w * dt
        self._theta_pred %= 2 * math.pi

        self._localized = False
        self._publish_pose_estimate(self._x_pred, self._y_pred, self._theta_pred)



    def _compute_pose_callback(self, odom_msg: Odometry, scan_msg: LaserScan):
        
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR sensor readings.

        """
        # Parse measurements
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z
        z_scan: list[float] = scan_msg.ranges

        # Execute particle filter
        self._execute_motion_step(z_v, z_w)
        x_h, y_h, theta_h = self._execute_measurement_step(z_scan)
        self._steps += 1

        # Publish
        self._publish_pose_estimate(x_h, y_h, theta_h)

    def _execute_measurement_step(self, z_scan: list[float]) -> tuple[float, float, float]:
        """Executes and monitors the measurement step (sense) of the particle filter.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].

        Returns:
            Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.
        """
        pose = (float("inf"), float("inf"), float("inf"))

        if self._localized or not self._steps % self._steps_btw_sense_updates:
            start_time = time.perf_counter()
            self._particle_filter.resample(z_scan)
            sense_time = time.perf_counter() - start_time

            self.get_logger().info(f"Sense step time: {sense_time:6.3f} s")

            if self._enable_plot:
                self._particle_filter.show("Sense", save_figure=True)

            start_time = time.perf_counter()
            self._localized, pose = self._particle_filter.compute_pose()
            clustering_time = time.perf_counter() - start_time

            self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        start_time = time.perf_counter()
        self._particle_filter.move(z_v, z_w)
        move_time = time.perf_counter() - start_time

        # self.get_logger().info(f"Move step time: {move_time:7.3f} s")

        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # TODO: 3.2. Complete the function body with your code (i.e., replace the pass statement).

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.localized = self._localized

        #if self._localized:
        msg.pose.position.x = x_h
        msg.pose.position.y = y_h
        msg.pose.position.z = 0.0

        qw, qx, qy, qz = euler2quat(0.0, 0.0, float(theta_h))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._publisher_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
