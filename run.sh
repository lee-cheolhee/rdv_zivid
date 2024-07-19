#!/bin/bash

#PROJECT_NAME="ms"
#remote_url=$(git config --get remote.origin.url)
#REPO_NAME=$(basename $remote_url .git)
#
#git_branch=$(git rev-parse --abbrev-ref HEAD)
#TAG=$(echo "$git_branch" | sed 's/[^a-zA-Z0-9\-_]//g')
PROJECT_NAME="ms/vision"
REPO_NAME="rdv_zivid"
TAG="develop"

IP_ADDR=100.100.100.2
HOSTNAME=$(hostname)
USER=$(id -un)
DISPLAY=:0

ENVS="--env=XAUTHORITY=/home/$USER/.Xauthority
      --env=DISPLAY=$DISPLAY
      --env=LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
      --env=ROS_IP=$IP_ADDR
      --env=ROS_HOSTNAME=$HOSTNAME
      --env=ROS_MASTER_URI=http://$IP_ADDR:11311
      --device=/dev/dri:/dev/dri"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK
         --volume=$XAUTH:/home/$USER/.Xauthority"

LOCAL_INC='/usr/local/include'
LOCAL_LIB='/usr/local/lib'
MOUNTS="--mount type=bind,readonly,source=$LOCAL_INC,target=/usr/local/include
        --mount type=bind,readonly,source=$LOCAL_LIB,target=/usr/local/lib
        --mount type=bind,source=/home/$USER/dataset,target=/root/dataset"

xhost +local:docker

#--gpus all \
docker run \
-d \
--restart always \
$ENVS \
$VOLUMES \
$MOUNTS \
--privileged \
--net host \
--ipc host \
--name=$REPO_NAME \
--workdir /root \
--gpus '"device=0"' \
$PROJECT_NAME/$REPO_NAME:$TAG
