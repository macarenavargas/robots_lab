import rclpy
from rclpy.node import Node

# Message types
from amr_msgs.msg import Key  # Example message (deprecated)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class MinimalSubscriber(Node):  # Nodes inherit from the base class Node
    def __init__(self) -> None:
        super().__init__("teleop")  # Set the node name
        # Subscribers (beware self._subscriptionsis reserved)

        self._subscriber_key = self.create_subscription(
            msg_type=Key,
            topic="key",
            callback=self._key_callback,
            qos_profile=10,
        )

        self._subscriber_lidar = self.create_subscription(
                    msg_type=LaserScan,
                    topic="scan",
                    callback=self._lidar_callback,
                    qos_profile=10,
                )


        self._publisher = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

    def _key_callback(self, msg: Key) -> None:
        # Log to the terminal with information (info) level
        self.get_logger().info(f"I heard: {msg.key_input}")

        cmd_vel = Twist()

        if msg.key_input == "w":
            cmd_vel.linear.x = 0.1
        elif msg.key_input == "s":
            cmd_vel.linear.x = -0.1
        elif msg.key_input == "a":
            cmd_vel.angular.z = 0.1
        elif msg.key_input == "d":
            cmd_vel.angular.z = -0.1
        elif msg.key_input == " ":
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = 0
        self._publisher.publish(cmd_vel)
        self.get_logger().info(f"Publishing: {cmd_vel.linear.x}, {cmd_vel.angular.z}")


def main(args=None) -> None:
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly.
    # Optional. Otherwise, it will be done automatically
    # when the garbage collector destroys the node object.
    minimal_subscriber.destroy_node()
    rclpy.shutdown()  # Or rclpy.try_shutdown()


if __name__ == "__main__":
    main()
