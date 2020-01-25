WRITE-UP OF PROJECT

The source-files consists of seven packages, split between the official ROS packages in one folder and my own two packages in another folder. These two folders are pick_objects and using_markers.

pick_objects sends goal poses to the robot.
using_markers publishes visual markers in rviz.

In addition, there are multiple bash scripts in the scripts folder. These launch a multiple of ros-packages in the appropiate order.

Robot spawned with pose by modifying /officialROS/turtlebot_simulator/turtlebot_gazebo/launch/includes/kobuki.launch.xml (-Y 1.5708)