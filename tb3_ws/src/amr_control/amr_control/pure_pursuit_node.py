
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from amr_msgs.msg import PoseStamped, MotionControl
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path

import math
import traceback
from transforms3d.euler import quat2euler

from amr_control.pure_pursuit import PurePursuit


class PurePursuitNode(LifecycleNode):
    def __init__(self):
        """Pure pursuit node initializer."""
        super().__init__("pure_pursuit")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("lookahead_distance", 0.2)
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
            lookahead_distance = (
                self.get_parameter("lookahead_distance").get_parameter_value().double_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value
            self._allow_motion = True 
            # Attribute and object initializations
            self._pure_pursuit = PurePursuit(
                dt,
                lookahead_distance,
                simulation=self._simulation,
                logger=self.get_logger(),  # Replace None with self.get_logger() to enable logging in the class
            )

            # Publishers
            #self._publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)
            if self._simulation:
                self._publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)

    
            else:

                self._publisher = self.create_publisher(Twist, "cmd_vel", 10)
                self.subscriber_motion = self.create_subscription(
                    MotionControl, "/motion_control", self._motion_control_callback, qos_profile=10
                )

            # Subscribers
            self._subscriber_pose = self.create_subscription(
                PoseStamped, "pose", self._compute_commands_callback, 10)

            
            #self._subscriber_path = self.create_subscription(Path, "path", self._path_callback, 10)
            
            #qos = QoSProfile(depth=10)
            #qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            self._subscriber_path = self.create_subscription(
                Path, "path", self._path_callback, 10
            )

            

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
        self._logger.info(f" ---allow motion = {self._allow_motion}----")
        if not self._allow_motion:
            
            self._logger.info(f" --i publish 0, 0!-- ")
            self._publish_velocity_commands(0.0, 0.0)
            return



    def _compute_commands_callback(self, pose_msg: PoseStamped):
        """Subscriber callback. Executes a pure pursuit controller and publishes v and w commands.

        Starts to operate once the robot is localized.

        Args:
            pose_msg: Message containing the estimated robot pose.

        """
        
        
        if self._allow_motion: 
            self._logger.info(f" entro en compute commands callback -> ALLOW MOTION = TRUE")
        
            if pose_msg.localized:
                # Parse pose
                x = pose_msg.pose.position.x
                y = pose_msg.pose.position.y
                quat_w = pose_msg.pose.orientation.w

                quat_x = pose_msg.pose.orientation.x
                quat_y = pose_msg.pose.orientation.y
                quat_z = pose_msg.pose.orientation.z
                _, _, theta = quat2euler((quat_w, quat_x, quat_y, quat_z))
                theta %= 2 * math.pi

                # Execute pure pursuit
                # self.get_logger().info(
                #     f"[PP] USING POSE → x={x:.2f}, y={y:.2f}, localized={pose_msg.localized}"
                # )
                v, w = self._pure_pursuit.compute_commands(x, y, theta)

            
                # Publish
                self._publish_velocity_commands(v, w)
            
            # else: 
            #     self._publish_velocity_commands(0.0, 0.0)

    def _path_callback(self, path_msg: Path):
        """Subscriber callback. Saves the path the pure pursuit controller has to follow.

        Args:
            path_msg: Message containing the (smoothed) path.

        """
        # TODO: 4.8. Complete the function body with your code (i.e., replace the pass statement).
        
        # iterate the path_msg and get the (x,y) of all the points
        path = []
        for pose in path_msg.poses: 

            x = pose.pose.position.x
            y = pose.pose.position.y

            path.append((x, y))
        
        # keep the path in the class 
        self._pure_pursuit.path = path
        self._pure_pursuit._last_closest_idx = 0
        
        self.get_logger().info(
            f"Path received with {len(path)} points | "
            f"start={path[0]} | end={path[-1]}"
            f"First 3 points: {path[:3]}"
        )
         

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """

        if self._simulation:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = v
            msg.twist.angular.z = w
        else:
            
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            if v == 0.0 and w == 0.0: 
                self._logger.info(" FROM PURE PURSUIT NODE I STOP THE ROBOT -> v = 0 and w = 0 ")

        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()

    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass

    pure_pursuit_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()