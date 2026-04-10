import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.time import Time
from transforms3d.euler import quat2euler
import math

# Message types
from nav_msgs.msg import Odometry

class OdometryNode(LifecycleNode):  # Inherit from LifecycleNode
    def __init__(self) -> None:
        super().__init__("odometry")
        
        # Initialize state variables
        self.prev_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        
        # Placeholders for ROS 2 entities
        self._subscriber = None
        self._publisher = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        # Reset variables on configuration
        self.prev_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        # Subscribers are standard, but created here
        self._subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self._estimate_velocities_callback,
            qos_profile=10,
        )

        # Publishers MUST be lifecycle publishers
        self._publisher = self.create_lifecycle_publisher(
            msg_type=Odometry, topic="/odometry", qos_profile=10
        )

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a deactivating transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a cleaning up transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'unconfigured' state.")
        self.destroy_subscription(self._subscriber)
        self.destroy_publisher(self._publisher)
        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a shutdown transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'shutting down' state.")
        self.destroy_subscription(self._subscriber)
        self.destroy_publisher(self._publisher)
        return super().on_shutdown(state)

    def _estimate_velocities_callback(self, msg: Odometry) -> None:
        # Ignore callback if the node is not active
        if not self._publisher or not self._publisher.is_activated:
            return

        current_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # obtain current position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # obtain orientation (translate from quaternion to Euler/Yaw)
        q = msg.pose.pose.orientation
        quart_list = [q.w, q.x, q.y, q.z]
        (roll, pitch, yaw) = quat2euler(quart_list)
        current_theta = yaw

        # If its the first message, only keep and exit (cannot derivate)
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_theta = current_theta
            return

        # calculate delta 
        dt = current_time - self.prev_time
        
        if dt == 0.0: # avoid collision at 0.0
            return

        dx = current_x - self.prev_x
        dy = current_y - self.prev_y
        d_theta = current_theta - self.prev_theta

        # normalize delta theta (from pi to -pi)
        if d_theta > math.pi:
            d_theta -= 2 * math.pi
        elif d_theta < -math.pi:
            d_theta += 2 * math.pi
  
        # linear velocity projection
        dist_projected = dx * math.cos(current_theta) + dy * math.sin(current_theta)
        v = dist_projected / dt

        # Angular velocity
        w = d_theta / dt

        # update for next iteration
        self.prev_time = current_time
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_theta = current_theta

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
        
    odometry_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()