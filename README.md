terminal 1: 
pip install nanobind
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup project.launch.py

terminal 2: 
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
 
