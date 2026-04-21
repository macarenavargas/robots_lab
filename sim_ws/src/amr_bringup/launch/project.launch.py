from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    simulation = True
    world = "project"
    # POINT 1
    a = (-1.0, -1.0, math.radians(90))
    d = (1.0, 1.0, math.radians(-90))
    e = (-0.6, 1.0, math.radians(0))
    c = (1.0, -1.0, math.radians(90))
    b = (0.2, -0.6, math.radians(0))
    f = (-1, 0.6, math.radians(0))
    # start = (-1.0, -1.0, math.radians(90))
    # goal = (-0.6, 1.0)
    start = e
    goal = (f[0], f[1])

    particle_filter_node = LifecycleNode(
        package="amr_localization",
        executable="particle_filter",
        name="particle_filter",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "enable_plot": False,
                "global_localization": True,
                "particles": 3000,  # 500,
                "sigma_v": 0.05,
                "sigma_w": 0.1,
                "sigma_z": 0.3,
                "simulation": simulation,
                "world": world,
            }
        ],
    )

    probabilistic_roadmap_node = LifecycleNode(
        package="amr_planning",
        executable="probabilistic_roadmap",
        name="probabilistic_roadmap",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "connection_distance": 0.15,
                "enable_plot": True,
                "goal": goal,
                "grid_size": 0.1,
                "node_count": 250,
                "obstacle_safety_distance": 0.15,  # 0.12,
                "simulation": simulation,
                "smoothing_additional_points": 4,
                "smoothing_data_weight": 0.1,
                "smoothing_smooth_weight": 0.25,
                "use_grid": True,
                "world": world,
            }
        ],
    )

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "enable_localization": True,
                "simulation": simulation,
            }
        ],
    )

    pure_pursuit_node = LifecycleNode(
        package="amr_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "lookahead_distance": 0.2,
                "simulation": simulation,
            }
        ],
    )

    coppeliasim_node = LifecycleNode(
        package="amr_simulation",
        executable="coppeliasim",
        name="coppeliasim",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "enable_localization": True,
                "goal": goal,
                "pose_tolerance": (0.1, 10.0),
                "start": start,
            }
        ],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "particle_filter",
                    "probabilistic_roadmap",
                    "wall_follower",
                    "pure_pursuit",
                    "coppeliasim",  # Must be started last
                )
            }
        ],
    )

    return LaunchDescription(
        [
            particle_filter_node,
            probabilistic_roadmap_node,
            wall_follower_node,
            pure_pursuit_node,
            coppeliasim_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
