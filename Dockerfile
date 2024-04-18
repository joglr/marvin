FROM osrf/ros:noetic-desktop-full

#Installing nano and nvim
RUN apt-get update \
	&& apt-get install -y \
	nano \
	neovim \
	tmux \
	python3-catkin-tools \
	ros-noetic-hector-gazebo-plugins \
	ros-noetic-ros-controllers \
	ros-noetic-gazebo-ros-control \
	ros-noetic-navigation \
	ros-noetic-joy \
	ros-noetic-teleop-twist-keyboard \
	ros-noetic-amcl \
	ros-noetic-neo-local-planner \
	ros-noetic-map-server \
	ros-noetic-move-base \
	ros-noetic-openslam-gmapping \
  git \
	&& rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

#Created files won't have sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir -p /home/$USERNAME/.config \
  && chown -R $USER_UID:$USER_GID /home/$USERNAME

#Setting up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p .config/nvim

RUN mkdir -p home/$USERNAME/catkin_ws/src

COPY entrypoint.sh /entrypoint.sh
COPY config/bashrc /.bashrc
COPY config/docker_tmux /home/$USERNAME/.tmux.conf
COPY config/vimrc /home/$USERNAME/.config/nvim/init.vim

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

#Builds the catkin workspace
RUN /bin/bash -c 'source ./opt/ros/noetic/setup.bash; cd /home/${USERNAME}/catkin_ws; catkin init; catkin build'

USER $USERNAME

CMD ["bash"]
