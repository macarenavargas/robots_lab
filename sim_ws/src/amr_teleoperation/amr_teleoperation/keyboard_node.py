import rclpy
from rclpy.node import Node
from amr_msgs.msg import Key
from sshkeyboard import listen_keyboard


class KeyboardNode(Node):
    """
    Publisher node that sends a message "Publishing : {key_input}" when a key is pressed. 
    """

    def __init__(self) -> None:
        # initilize the constructor 
        super().__init__("keyboard")

        # configutation of publisher 
        self._publisher = self.create_publisher(msg_type=Key, topic="key", qos_profile=10)

        # we receive if a key is pressed 
        listen_keyboard(on_press=self._listen_keyboard_callback, sequential=True)

    def _listen_keyboard_callback(self, key) -> None:
        """
        Callback that is executed to publish the string of the pressed key 
        """
        msg = Key()
        msg.key_input = key

  
        # publish the message in the topic 
        self._publisher.publish(msg)

        # shows the message in terminal also 
        self.get_logger().info(f"Publishing: {msg.key_input}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
