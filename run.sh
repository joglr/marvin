xhost +local:root
sudo groupadd docker
sudo usermod -aG docker $USER

#if [ "$(systemctl is-enabled docker)" != "enabled" ]; then
#  exit 1
#fi

docker run -it --user ros --name my_ros_container --network host --ipc host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/source:/catkin_ws/src --env DISPLAY my_ros
