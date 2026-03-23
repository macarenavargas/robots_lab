import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from amr_msgs.msg import PoseStamped as AmrPoseStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import os
import time
import traceback

from amr_planning.prm import PRM


class PRMNode(LifecycleNode):
    def __init__(self):
        """Probabilistic roadmap (PRM) node initializer."""
        super().__init__("probabilistic_roadmap")

        # Parameters
        self.declare_parameter("connection_distance", 0.3)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("goal", (0.0, 0.0))
        self.declare_parameter("grid_size", 0.05)
        self.declare_parameter("node_count", 250)
        self.declare_parameter("obstacle_safety_distance", 0.08)
        self.declare_parameter("simulation", False)
        self.declare_parameter("smoothing_additional_points", 3)
        self.declare_parameter("smoothing_data_weight", 0.1)
        self.declare_parameter("smoothing_smooth_weight", 0.3)
        self.declare_parameter("use_grid", False)
        self.declare_parameter("world", "project")


    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            connection_distance = (
                self.get_parameter("connection_distance").get_parameter_value().double_value
            )
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            self._goal = tuple(
                self.get_parameter("goal").get_parameter_value().double_array_value.tolist()
            )
            grid_size = self.get_parameter("grid_size").get_parameter_value().double_value
            node_count = self.get_parameter("node_count").get_parameter_value().integer_value
            obstacle_safety_distance = (
                self.get_parameter("obstacle_safety_distance").get_parameter_value().double_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value
            self._smoothing_additional_points = (
                self.get_parameter("smoothing_additional_points")
                .get_parameter_value()
                .integer_value
            )
            self._smoothing_data_weight = (
                self.get_parameter("smoothing_data_weight").get_parameter_value().double_value
            )
            self._smoothing_smooth_weight = (
                self.get_parameter("smoothing_smooth_weight").get_parameter_value().double_value
            )
            use_grid = self.get_parameter("use_grid").get_parameter_value().bool_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._localized = False

            start_time = time.perf_counter()
            self._planning = PRM(
                map_path,
                obstacle_safety_distance,
                use_grid,
                node_count,
                grid_size,
                connection_distance,
                simulation=self._simulation,
                logger=None,  # Replace None with self.get_logger() to enable logging in the class
            )
            roadmap_creation_time = time.perf_counter() - start_time

            self.get_logger().info(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

            # Publishers
            # TODO: 4.6. Create the /path publisher (Path message).
            #self._publisher_path = self.create_publisher(msg_type= Path, topic="/path", qos_profile=10)
            #qos = QoSProfile(depth=10)
            #qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            self._publisher_path = self.create_publisher(
                msg_type=Path,
                topic="/path",
                #qos_profile=qos,
                qos_profile=10,
            )
                        
            # Subscribers
            self._subscriber_pose = self.create_subscription(
                AmrPoseStamped, "pose", self._path_callback, 10
            )


            self.get_logger().info(
                f"PRM params | goal={self._goal} | "
                f"node_count={node_count} | grid_size={grid_size:.3f} | "
                f"conn_dist={connection_distance:.3f} | "
                f"obs_safe={obstacle_safety_distance:.3f} | "
                f"use_grid={use_grid} | world={world}"
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

    def _path_callback(self, pose_msg: AmrPoseStamped):
        """Subscriber callback. Finds a path using A* and publishes the smoothed path to the goal.

        Args:
            pose_msg: Message containing the robot pose estimate.

        """

        if pose_msg.localized and not self._localized:
            
            # localize the robot for the first tiem  
            start = (pose_msg.pose.position.x, pose_msg.pose.position.y)
            self.get_logger().info("Robot localized for the first time. Computing path.")
            start_time = time.perf_counter()
            # find the path from the start to the goal using A* algorithm.
            path = self._planning.find_path(start, self._goal)
            pathfinding_time = time.perf_counter() - start_time

            self.get_logger().info(f"Pathfinding time: {pathfinding_time:1.3f} s")

            # smooth the path 
            smoothed_path = PRM.smooth_path(
                path,
                data_weight=self._smoothing_data_weight,
                smooth_weight=self._smoothing_smooth_weight,
                additional_smoothing_points=self._smoothing_additional_points,
            )
            smoothing_time = time.perf_counter() - start_time

            self.get_logger().info(f"Smoothing time: {smoothing_time:1.3f} s")

            # see the plot 
            if self._enable_plot:
                self._planning.show(path=path, smoothed_path=smoothed_path, save_figure=True)

            self._publish_path(smoothed_path)
        
        self._localized = pose_msg.localized

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        """Publishes the robot's path to the goal in a nav_msgs.msg.Path message.

        Args:
            path: Smoothed path (initial location first) in (x, y) format.

        """
        # TODO: 4.7. Complete the function body with your code (i.e., replace the pass statement).

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for x,y in path: 
            # for each point in the path list, we make a PoseStamp message 
            # and add it to the "poses" field. 
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation.w = 1.0

            msg.poses.append(pose_msg)
        

        self._publisher_path.publish(msg)
        self.get_logger().info(
            f"Published /path with {len(path)} points | "
            f"first=({path[0][0]:.2f}, {path[0][1]:.2f}) | "
            f"last=({path[-1][0]:.2f}, {path[-1][1]:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    prm_node = PRMNode()

    try:
        rclpy.spin(prm_node)
    except KeyboardInterrupt:
        pass

    prm_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
