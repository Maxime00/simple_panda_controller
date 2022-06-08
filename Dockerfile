ARG BASE_TAG=galactic-devel
FROM ghcr.io/aica-technology/ros2-modulo:${BASE_TAG}

WORKDIR /tmp
RUN git clone -b feature/cl-develop --depth 1 https://github.com/aica-technology/network-interfaces.git && cd network-interfaces && \
  sudo bash install.sh --auto
RUN sudo rm -rf /tmp/network-interfaces

WORKDIR ${ROS2_WORKSPACE}
#RUN pip3 install pyquaternion
COPY --chown=${USER} simple_controllers ./src/simple_controllers
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

# clean image # is this useful ?
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*