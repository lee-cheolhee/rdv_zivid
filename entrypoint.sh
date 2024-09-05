#!/bin/bash

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/install/setup.bash

roslaunch --wait zivid_samples rdv_zivid.launch

exec "$@"
