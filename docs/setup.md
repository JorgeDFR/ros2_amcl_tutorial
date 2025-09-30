# Manual Setup

This guide explains how to manually set up your environment for the AMCL tutorial on your own machine.

If you already have ROS 2 and Webots installed, you only need to install some additional [ROS 2 packages](#ros-2-packages) and clone this [Git repository](#clone-and-build-the-tutorial-repository).

If not you also need to install both [ROS 2](#ros-2-installation) and the [Webots simulator](#webots-simulator).

## ROS 2 Installation

This tutorial was tested in both **ROS 2 Humble** and **ROS 2 Jazzy**.

At the time of writing, [ROS 2 Humble](https://docs.ros.org/en/humble) is the latest maintained LTS distribution for **Ubuntu 22.04 (Jammy)**, and [ROS 2 Jazzy](https://docs.ros.org/en/jazzy) is the latest maintained LTS distribution for **Ubuntu 24.04 (Noble)**.

However, the code should also work with other [ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html).

To install ROS 2, follow the official ROS 2 installation guide (e.g. [humble](https://docs.ros.org/en/humble/Installation.html), [jazzy](https://docs.ros.org/en/jazzy/Installation.html)).

> ✅ **Recommendation:** Install the **Desktop version**, since it includes essential tools like RViz that are required for this tutorial.

For convenience, the key steps are reproduced here.
These steps consider a machine running Ubuntu, where `<ros2-distro>` must be substituted by the distro name (e.g. `humble`, `jazzy`, etc.):

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install ros-<ros2-distro>-desktop

source /opt/ros/<ros2-distro>/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update --rosdistro <ros2-distro>

echo "source /opt/ros/<ros2-distro>/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Webots Simulator

The **Webots Simulator** is required to run the examples in this tutorial.

To install Webots, follow the official [Webots installation guide](https://cyberbotics.com/doc/guide/installation-procedure).

Also, to install the interface between ROS 2 and Webots follow the official [installation guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html).

The key installation steps for a machine running Ubuntu are reproduced below:

```bash
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt update
sudo apt install ./webots_2025a_amd64.deb
sudo apt install ros-<ros2-distro>-webots-ros2
```

## ROS 2 Packages

This tutorial also requires a few additional ROS 2 packages from the **Nav2 stack** and related tools, including AMCL, the map server, RViz plugins, and the teleoperation package:

```bash
sudo apt install ros-<ros2-distro>-nav2-amcl
sudo apt install ros-<ros2-distro>-nav2-map-server
sudo apt install ros-<ros2-distro>-nav2-rviz-plugins
sudo apt install ros-<ros2-distro>-teleop-twist-keyboard
```

## Clone and Build the Tutorial Repository

In ROS 2, all your code and simulation files live inside a **workspace**.
A workspace is just a folder where you keep your packages, and it must have a `src` subfolder where the source code goes.

Follow these steps to download the tutorial package and build it:

1. Create a ROS 2 workspace.
```bash
mkdir -p ~/ros2_ws/src
```

2. Download (clone) this tutorial package into the workspace.
```bash
cd ~/ros2_ws/src
git clone https://github.com/JorgeDFR/ros2_amcl_tutorial.git
```

3. Load your ROS 2 installation into the current terminal (replace `<ros2-distro>` with your distribution name, e.g. `humble`, `jazzy`, etc.)
```bash
source /opt/ros/<ros2-distro>/setup.bash
```

4. Build the workspace using colcon (the ROS 2 build tool).
The command `--symlink-install` allows you to edit config files without rebuilding every time.
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

5. Add the workspace overlay to your shell startup. This way, every new terminal knows about the packages you just built.
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```