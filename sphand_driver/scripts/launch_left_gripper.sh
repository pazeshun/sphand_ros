#!/bin/bash

# Wait until network is ready
while ! ping -c 1 baxter > /dev/null
do
  sleep 1
done

# roslaunch
set -e
source ~/apc_ws/devel/setup.bash
rossetip
rossetmaster baxter
roslaunch sphand_driver setup_gripper_v8.launch left_gripper:=true
