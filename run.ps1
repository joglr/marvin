
docker run -it `
  --user ros `
  --name my_ros_container `
  --network host `
  --ipc host `
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" `
  --volume $PWD/source:/catkin_ws/src `
  --env="DISPLAY=novnc:0.0" `
  --env="QT_X11_NO_MITSHM=1" my_ros
