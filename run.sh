#!/bin/bash

xhost +
sudo groupadd docker
sudo usermod -aG docker $USER

#if [ "$(systemctl is-enabled docker)" != "enabled" ]; then
#  exit 1
#fi

docker run -it \
  --user ros \
  --name my_ros \
  --network host \
  --ipc host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $PWD/source:/home/ros/catkin_ws/src \
  --device=/dev/dri \
  --group-add video \
  --env "DISPLAY=$DISPLAY" \
  my_ros
