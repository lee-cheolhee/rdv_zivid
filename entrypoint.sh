#!/bin/bash

# PC에 맞게 설정해야함
#IP_ADDR=$(ifconfig eno1 | grep 'inet ' | awk '{print $2}')
#echo "$IP_ADDR"
#
#export ROS_IP=$IP_ADDR
#HOSTNAME=$(hostname)
#export ROS_HOSTNAME=$HOSTNAME
#export ROS_MASTER_URI=http://$IP_ADDR:11311

#echo "192.168.101.201	AGX-FRONT" >> /etc/hosts
#echo "192.168.101.202	AGX-REAR" >> /etc/hosts
#---------------------------------------------------------------------
source /root/catkin_ws/install/setup.bash

roslaunch --wait zivid_sample rdv_zivid.launch

exec "$@"
