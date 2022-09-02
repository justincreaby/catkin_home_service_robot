#!/bin/sh
xterm  -e  "srcdev; roslaunch my_robot world.launch" &
sleep 5
xterm  -e  "srcdev; roslaunch my_robot view_navigation.launch" & 
sleep 5
xterm  -e  "srcdev; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "srcdev; roslaunch my_robot teleop.launch"
