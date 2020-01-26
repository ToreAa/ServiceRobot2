WRITE-UP OF PROJECT

The source-files consists of seven packages, split between the official ROS packages in one folder and my own two packages in another folder. These two folders are pick_objects and using_markers.

pick_objects sends goal poses to the robot.
using_markers publishes visual markers in rviz.

In addition, there are multiple bash scripts in the scripts folder. These launch a multiple of ros-packages in the appropiate order, each in its own xterminal. This makes sure that the ROS status messages are easily differentiated between the different nodes.

Robot spawned with pose by modifying /officialROS/turtlebot_simulator/turtlebot_gazebo/launch/includes/kobuki.launch.xml (-Y 1.5708)

The official packages are used to 
- spawn the robot in the environment (turtlebot and turtlebot_simulator)
- interact with robot and get it to move (turtlebot_interactions)
- to create a map (pgm_map_creator)
- for localization (slam_gmapping), that does AMCL