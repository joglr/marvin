FROM osrf/ros:noetic-desktop-full

#Installing nano and nvim
RUN apt-get update \
	&& apt-get install -y \
	nano \
	neovim \
	tmux \
	&& rm -rf /var/lib/apt/lists/*

COPY config/ /site_config

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

#Created files won't have sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

#Setting up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
COPY /tmux.conf /.tmux.conf

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]
