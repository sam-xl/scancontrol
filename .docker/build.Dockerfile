FROM scancontrol:base AS dev 
ARG TARGET_WS

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user
ENV ROS_USER=ros
ARG ROS_USER_UID=1000
ARG ROS_USER_GID=$ROS_USER_UID

RUN groupadd --gid $ROS_USER_GID $ROS_USER \
  && useradd -s /bin/bash --uid $ROS_USER_UID --gid $ROS_USER_GID -m $ROS_USER \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $ROS_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$ROS_USER\
  && chmod 0440 /etc/sudoers.d/$ROS_USER \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 

# Get build dependencies
RUN apt-get update \
  && apt-get install --no-install-recommends -y     build-essential     git     python3-colcon-common-extensions     python3-colcon-mixin     python3-rosdep     python3-vcstool     \
  && rm -rf /var/lib/apt/lists/* 

WORKDIR $TARGET_WS 

# perform a copy here so previous layers still remain in cache
COPY . $TARGET_WS/src/scancontrol

RUN chown -R ${ROS_USER}:${ROS_USER} /workspaces
USER ${ROS_USER}

# install rosdeps
RUN sudo -E rosdep init \
  && rosdep update \
  && sudo apt-get update

RUN rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/*

# # build package
RUN source /opt/ros/humble/setup.bash && colcon build
