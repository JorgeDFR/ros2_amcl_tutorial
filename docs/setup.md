# Manual Setup

This guide explains how to manually set up your environment for the AMCL tutorial without using a Virtual Machine or Docker.

---

## ROS 2 Installation

This tutorial was designed for **ROS 2 Humble**, which is the latest maintained distribution for **Ubuntu 22.04 (Jammy)** at the time of writing.
However, it should also work with other [ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html).

To install ROS 2 Humble, follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

> ✅ **Recommendation:** Install the **Desktop version**, since it includes essential tools like RViz that are required for this tutorial.

For convenience, the key steps are reproduced here:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install -y ros-humble-desktop

source /opt/ros/humble/setup.bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update --rosdistro humble
```

---

## Workspace Setup

ROS 2 uses a *workspace* folder to build and organize your projects.

1. Follow the official guide: [Creating a ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
2. Make sure `colcon` and its extensions are installed (see above).

To avoid having to source ROS every time you open a new terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Stage Simulator

The **Stage Simulator** is required to run the examples in this tutorial.

A ROS 2-compatible fork of the Stage ROS wrapper is available here:
👉 [sousarbarb/stage_ros](https://github.com/sousarbarb/stage_ros)

The key installation steps are reproduced below:

```bash
sudo apt update
sudo apt dist-upgrade
sudo apt install git build-essential cmake
sudo apt install libjpeg-dev libpng-dev libltdl-dev libfltk1.3-dev libglu1-mesa-dev

mkdir -p ~/dev
cd ~/dev
git clone https://github.com/sousarbarb/Stage.git
cd Stage
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make -j$(nproc)
sudo make install
sudo ldconfig --verbose /opt/stage/lib/
```

Then, install the ROS 2 wrapper in the previously created ROS 2 workspace. Here we consider that the workspace is named `ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
git clone https://github.com/sousarbarb/stage_ros.git
cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers status+ console_direct+ console_start_end+

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ROS 2 Packages

This tutorial requires additional ROS 2 packages from the **Nav2 stack**, including AMCL, the map server, and RViz plugins.

```bash
sudo apt install ros-humble-nav2-amcl
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-rviz-plugins
```

---

## Clone the Tutorial Repository

Finally, clone this repository to access the launch files, configuration files, and simulation worlds used in the tutorial:

```bash
git clone https://github.com/JorgeDFR/ros2_amcl_tutorial.git
```
