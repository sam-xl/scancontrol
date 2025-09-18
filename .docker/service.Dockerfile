ARG ROS_DISTRO=humble

FROM samxl/scancontrol:${ROS_DISTRO}-base AS scancontrol-service 

ARG ROS_DISTRO
ARG WORKSPACE_DIR=/tmpfs/scancontrol_ws
ARG INSTALL_DIR=/opt/scancontrol_ws

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
  && rosdep init \
  # && rm -rf /var/lib/apt/lists/* \
  && apt-get autoremove -y \
  && apt-get clean -y


RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/scancontrol \
  cd ${WORKSPACE_DIR} \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep update --include-eol-distros \
  && rosdep install --from-paths ${WORKSPACE_DIR}/src -iy \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}

WORKDIR ${INSTALL_DIR}