#!/bin/sh
xterm  -e  "srcdev; roslaunch my_robot world.launch" &
sleep 5
xterm  -e  "srcdev; roslaunch my_robot view_navigation.launch" & 
sleep 5
xterm  -e  "srcdev; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "srcdev; roslaunch my_robot teleop.launch" &
sleep 5
xterm -e "srcdev; rosrun add_markers add_markers" &
sleep 5
xterm -e "srcdev; rosrun pick_objects pick_objects"
