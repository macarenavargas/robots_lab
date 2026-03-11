import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped
from amr_msgs.msg import MotionControl
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback

from amr_control.wall_follower import WallFollower


class WallFollowerNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.1)  # simulation 0.05
        self.declare_parameter("enable_localization", False)
        self.declare_parameter("simulation", False)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization = (
                self.get_parameter("enable_localization").get_parameter_value().bool_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value

            # Attribute and object initializations
            self._wall_follower = WallFollower(
                dt,
                simulation=self._simulation,
                logger=self.get_logger(),  # Replace None with self.get_logger() to enable logging in the class
            )

            # --- Variables to register for non-simulation mode---
            self._last_z_v = None
            self._last_z_w = None
            self._last_z_scan = None
            self._allow_motion = True

            # Publishers
            # TODO: 2.10. Create the /cmd_vel velocity commands publisher (TwistStamped message).

            # Publishers
            if self._simulation:
                self._publisher_cmd_vel = self.create_publisher(
                    msg_type=TwistStamped, topic="/cmd_vel", qos_profile=10
                )

            else:
                self._publisher_cmd_vel = self.create_publisher(
                    msg_type=Twist, topic="/cmd_vel", qos_profile=10
                )

            # Subscribers
            # TODO: 2.7. Synchronize _compute_commands_callback with /odometry and /scan.
            qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._subscribers: list[message_filters.Subscriber] = []
            # Append as many topics as needed

            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "/odometry", qos_profile=qos_profile)
            )

            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "/scan", qos_profile=qos_profile)
            )

            ts = None

            if self._simulation:
                ts = message_filters.ApproximateTimeSynchronizer(
                    self._subscribers, queue_size=10, slop=9
                )  # we will have to change slop to a lower value for the real robot

                ts.registerCallback(self._compute_commands_callback)

            else:
                ts = message_filters.ApproximateTimeSynchronizer(
                    self._subscribers, queue_size=10, slop=0.15
                )  # much smaller value
                ts.registerCallback(self._capture_measurements_callback)
                self._timer = self.create_timer(dt, self._compute_commands_timer)

                # suscriber
                self._subscriber_motion_control = self.create_subscription(
                    MotionControl,
                    "/motion_control",
                    callback=self._motion_control_callback,
                    qos_profile=10,
                )

            # TODO: 4.12. Add /pose to the synced subscriptions only if localization is enabled.

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

    def _motion_control_callback(self, motion_control_msg: MotionControl):
        self._allow_motion = motion_control_msg.allow_motion

    def _capture_measurements_callback(
        self, odom_msg: Odometry, scan_msg: LaserScan, pose_msg: PoseStamped = PoseStamped()
    ):

        # the first callback in non-simulation
        if not pose_msg.localized:
            self._last_z_v: float = odom_msg.twist.twist.linear.x
            self._last_z_w: float = odom_msg.twist.twist.angular.z

            self._last_z_scan: list[float] = scan_msg.ranges

    def _compute_commands_timer(
        self,
    ):

        if self._last_z_v is None or self._last_z_w is None or self._last_z_scan is None:
            return

        z_v: float = self._last_z_v
        z_w: float = self._last_z_w
        z_scan: list[float] = self._last_z_scan

        # Execute wall follower
        v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
        # self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")

        # Publish
        self._publish_velocity_commands(v, w)

    def _compute_commands_callback(
        self, odom_msg: Odometry, scan_msg: LaserScan, pose_msg: PoseStamped = PoseStamped()
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        if not self._simulation and not self._allow_motion:
            self._publish_velocity_commands(0.0, 0.0)
            return

        if not pose_msg.localized:
            # TODO: 2.8. Parse the odometry from the Odometry message (i.e., read z_v and z_w).
            z_v: float = odom_msg.twist.twist.linear.x
            z_w: float = odom_msg.twist.twist.angular.z

            # TODO: 2.9. Parse LiDAR measurements from the LaserScan message (i.e., read z_scan).
            z_scan: list[float] = scan_msg.ranges

            # Execute wall follower
            v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
            # self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")

            # Publish
            self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """
        # TODO: 2.11. Complete the function body with your code (i.e., replace the pass statement).

        msg = None
        # we will have to modify this when transfering it to the real robot
        if self._simulation:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = v
            msg.twist.angular.z = w

        else:
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = -w  # (-) because the robot is dextr.

        self._publisher_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
