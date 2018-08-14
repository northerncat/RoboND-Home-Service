#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch custom_view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node" &
sleep 5
xterm -e "rosrun add_markers add_markers_node" &