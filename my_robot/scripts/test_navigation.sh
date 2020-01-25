source /opt/ros/kinetic/setup.bash
source /home/workspace/catkin_ws/devel/setup.bash
#!/bin/sh
# Launch turtlebot in gazeboworld
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/map/tore.world" &
sleep 15
# Launch amcl
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/map/tore.yaml" &
sleep 5
# Launch rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"


