FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV USER=user
ENV HOME=/home/$USER

# Install additional packages
RUN apt-get update && apt-get install -y \
  sudo \
  curl \
  wget \
  git \
  nano \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  bash-completion \
  software-properties-common \
  && rm -rf /var/lib/apt/lists/*

# Install Webots Simulator
RUN apt-get update && \
  wget -O /tmp/webots.deb https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && \
  dpkg -i /tmp/webots.deb || apt-get install -f -y && \
  apt-get install -y ros-humble-webots-ros2 && \
  rm -f /tmp/webots.deb && \
  rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots

# ROS packages
RUN apt-get update && apt-get install -y \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-rviz-plugins \
  && rm -rf /var/lib/apt/lists/*

# Create user and home directory
RUN useradd -m -d $HOME -s /bin/bash $USER && \
  echo "$USER:$USER" | chpasswd && \
  usermod -aG sudo $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> $HOME/.bashrc

# Initialize rosdep
USER root
RUN rosdep init || true
USER $USER
RUN rosdep update

# Set working directory to home and default command
USER $USER
WORKDIR $HOME
CMD ["/bin/bash"]