ARG ROS_DISTRO=humble

FROM registry.tudelft.nl/samxl/scancontrol:${ROS_DISTRO}-base AS scancontrol-service 

ARG TARGET_WS
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Get build dependencies
RUN sudo apt-get update \
  && sudo apt-get install --no-install-recommends -y \
     build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool     \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt-get autoremove -y \
  && sudo apt-get clean -y

WORKDIR $TARGET_WS 

# perform a copy here so previous layers still remain in cache
COPY . $TARGET_WS/src/scancontrol

RUN sudo chown -R ${USERNAME}:${USERNAME} $TARGET_WS

# install rosdeps
RUN sudo -E rosdep init \
  && rosdep update \
  && sudo apt-get update

RUN rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/*

# # build package
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build
