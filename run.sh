#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 <container_name>"
	exit 1
fi

container_name=$1

xhost +local:root
sudo groupadd docker
sudo usermod -aG docker $USER

#if [ "$(systemctl is-enabled docker)" != "enabled" ]; then
#  exit 1
#fi

docker run -it --user ros --name "$container_name" --network host --ipc host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/source:/catkin_ws/src --env DISPLAY private_ros
