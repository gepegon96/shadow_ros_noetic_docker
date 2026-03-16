# Stereolabs image (Ubuntu 20.04 + CUDA + ZED SDK)
FROM stereolabs/zed:4.0-gl-devel-cuda11.4-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    python3-catkin-tools \
    python3-vcstool \
    python3-osrf-pycommon \
    python3-pip \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-slam-toolbox \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-amcl \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-robot-localization \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-xacro \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-tf-tree \
    ros-noetic-rqt-reconfigure \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \ 
    ros-noetic-joint-state-publisher \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-roslint \
    ros-noetic-image-transport-plugins \
    ros-noetic-people-msgs \
    libpcap-dev \
    python3-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial scipy cupy torch torchvision lap cython_bbox pandas matplotlib tqdm seaborn ultralytics imagehash

WORKDIR /root/catkin_ws

# Entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
