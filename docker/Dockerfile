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

# Create user and home directory
RUN useradd -m -d $HOME -s /bin/bash $USER && \
  echo "$USER:$USER" | chpasswd && \
  usermod -aG sudo $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Stage simulator
USER root
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  libjpeg-dev \
  libpng-dev \
  libltdl-dev \
  libfltk1.3-dev \
  libglu1-mesa-dev \
  && rm -rf /var/lib/apt/lists/*

USER $USER
RUN mkdir -p $HOME/simulator && cd $HOME/simulator && \
  git clone https://github.com/sousarbarb/Stage.git && \
  cd Stage && mkdir -p build && cd build && \
  cmake -DCMAKE_BUILD_TYPE=Release .. && \
  make -j$(nproc)

USER root
RUN cd $HOME/simulator/Stage/build && \
  make install && \
  ldconfig

# ROS packages
USER root
RUN apt-get update && apt-get install -y \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-rviz-plugins \
  && rm -rf /var/lib/apt/lists/*

USER $USER
RUN mkdir -p $HOME/ros2_ws/src && cd $HOME/ros2_ws/src/ && \
  git clone https://github.com/sousarbarb/stage_ros.git

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc && \
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