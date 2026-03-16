# shadow_ros_noetic_docker
This repository provides a fully configured, Dockerized ROS Noetic environment to operate **Shadow**, a mobile social robot designed by the RoboLab group (University of Extremadura). 

> **Hardware note:** Although the full version of Shadow features an omnidirectional base with Mecanum wheels, **the software in this repository is currently configured for a differential drive base**.

## Package content (`shadow_robot_node`)

This environment deploys the main metapackage/node `shadow_robot_node`, which centralizes the robot's operation:

* **Hardware & control:** Bridge node that receives velocity commands (`cmd_vel`) and publishes the robot's wheels odometry (`odom`).
* **Robot description (URDF):** Physical modeling of the robot and publication of the Transform Tree (TF).
* **Navigation stack:** * Configuration and filtering files for the two 3D LiDARs (RoboSense Helios 32 and BPearl).
  * Local and Global Costmap configurations (`move_base`).
  * Probabilistic localization system configuration (`AMCL`).
* **Maps and visualization:** `.yaml` files for pre-recorded maps and ready-to-use RViz configurations.
* **Launch files:** Modular `.launch` scripts to start components independently or boot the entire stack.

## 💻 Requirements
* [Docker](https://docs.docker.com/get-docker/) installed.
* (Optional but highly recommended) `nvidia-docker` to leverage hardware acceleration.

---

## 🐳 Docker Installation and Usage

### 1. Build the Image
Clone the repository on your local machine or the robot's computer and build the image:

```bash

git clone [https://github.com/gepegon96/shadow_ros_noetic_docker.git](https://github.com/gepegon96/shadow_ros_noetic_docker.git)
cd shadow_ros_noetic_docker
docker build -t shadow_ros_noetic_container .
2. Run the Container
Before running container, give permission to container for using display:
echo "xhost +local:root > /dev/null 2>&1" >> ~/.bashrc
Start the container, sharing the host network and enabling the graphical interface (required for RViz).

Bash
docker run -it --net=host --privileged \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name shadow_container \
    shadow_ros_noetic_container bash
💡 Tip: The Dockerfile is configured to automatically source the ROS environment variables (source devel/setup.bash). Any new terminal you open using docker exec -it shadow_container bash will be ready to run ROS commands immediately.