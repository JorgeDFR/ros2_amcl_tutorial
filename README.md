# AMCL Tutorial using ROS 2 and Stage

This tutorial demonstrates how the **Adaptive Monte Carlo Localization (AMCL)** algorithm works.
We will use the open-source implementation of the algorithm, [`nav2_amcl`](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl), from the [Nav2 ROS stack](https://docs.nav2.org/index.html).

The tutorial environment is based on **ROS 2** and the **Stage Simulator**.
If you are new to these tools, check the following resources first:
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Stage Documentation](https://player-stage-manual.readthedocs.io/en/latest/)

---

## Setup and Prerequisites

To simplify the setup process, this tutorial provides two options:

1. **Pre-configured Virtual Machine (VM)**
2. **Docker Container with docker compose**

If you prefer to install everything directly on your own system, follow the instructions in [manual setup](/docs/setup.md).

---

### Option 1: Using the Virtual Machine

A ready-to-use VM template is available for download:
👉 [Download VM template](https://example.com/error) <!-- TODO: Replace with actual working link -->

Once downloaded, you can import the template into your preferred hypervisor (e.g., [VirtualBox](https://www.virtualbox.org/)).

**VM credentials:**
- **Username:** `user`
- **Password:** `1234`

> ⚠️ **Important:** To ensure you have the latest tutorial files, run the following inside the VM:
```bash
cd ~/ros2_amcl_tutorial
git pull
```

---

### Option 2: Using the Docker Container

This tutorial provides a Docker-based setup that includes all dependencies.

#### Step 1: Install Docker

Make sure Docker is installed on your system. Follow the official guides for your platform:
- [Linux](https://docs.docker.com/engine/install/)
- [Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
- [macOS](https://docs.docker.com/desktop/setup/install/mac-install/)

#### Step 2: Build the container

The provided [`Dockerfile`](/docker/Dockerfile) is located in the `/docker` folder of this repository.

To build the docker image for the container run:
```bash
git clone https://github.com/JorgeDFR/ros2_amcl_tutorial.git
cd ros2_amcl_tutorial/docker
docker compose build
```

#### Step 3: Run with docker compose

Because this tutorial requires **graphical interfaces** (e.g., RViz and the Stage Simulator), a `docker-compose.yml` file is included for convenience.
It handles container setup and display forwarding automatically.

To start the docker container run:
```bash
docker compose up -d
docker exec -it ros2_amcl_tutorial bash
```

---

## TODO