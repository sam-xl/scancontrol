ARG ROS_DISTRO=humble

FROM samxl/scancontrol:${ROS_DISTRO}-base AS scancontrol-service 

ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Get build dependencies
RUN apt-get update \
  && apt-get install --no-install-recommends -y \
     build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool     \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get autoremove -y \
  && apt-get clean -y


# perform a copy here so previous layers still remain in cache
COPY . /workspaces/scancontrol_ws/src/scancontrol

WORKDIR /workspaces/scancontrol_ws/

# install rosdeps
RUN rosdep init \
  && rosdep update \
  && apt-get update

RUN rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# # build package
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build
