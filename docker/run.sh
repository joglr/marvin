xhost +local:root
sudo groupadd docker
sudo usermod -aG docker $USER

#if [ "$(systemctl is-enabled docker)" != "enabled" ]; then
#  exit 1
#fi

docker run --name my_ros_container -it --user ros --network host --ipc host -v $PWD/source:/my_source_code -v /tmp/.X11-unix:/tmp/.X11-uniw:rw --env DISPLAY my_ros
