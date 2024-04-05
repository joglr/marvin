docker run -it --user ros --network host --ipc host -v $PWD/source:/my_source_code -v /temp/.X11-unix:/tmp/.X11-uniw:rw --env DISPLAY my_ros
