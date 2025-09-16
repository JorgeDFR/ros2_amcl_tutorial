# Manual Setup

This guide explains how to manually set up your environment for the AMCL tutorial on a machine running Ubuntu 22.04.

---

## ROS 2 Installation

This tutorial was designed for **ROS 2 Humble**, which is the latest maintained distribution for **Ubuntu 22.04 (Jammy)** at the time of writing.
However, it should also work with other [ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html).

To install ROS 2 Humble, follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

> ✅ **Recommendation:** Install the **Desktop version**, since it includes essential tools like RViz that are required for this tutorial.

For convenience, the key steps are reproduced here:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update --rosdistro humble

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Webots Simulator

The **Webots Simulator** is required to run the examples in this tutorial.

To install Webots, follow the official [Webots installation guide](https://cyberbotics.com/doc/guide/installation-procedure).
Also, to install the interface betwwen ROS 2 and Webots follow the official [installation guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html).

The key installation steps are reproduced below:

```bash
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt update
sudo apt install ./webots_2025a_amd64.deb
sudo apt install ros-humble-webots-ros2
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
