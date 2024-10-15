FROM frankjoshua/ros2

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3 pip \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
RUN pip3 install --break-system-packages pyserial
ENV DEBIAN_FRONTEND=dialog

# Set the working directory to /root
WORKDIR /root

# Copy your existing ROS2 workspace into the container
COPY ros2_ws ./ros2_ws/

# Install all dependencies for the workspace
RUN apt-get update && \
    cd ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace using colcon
RUN cd ros2_ws \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install

# Copy the entrypoint script into the container
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Ensure the entrypoint script is executable
RUN chmod +x /ros_entrypoint.sh
WORKDIR /root/ros2_ws/src/gps_to_ros/gps_to_ros
CMD ["/bin/bash", "-i", "-c", "python3 gps_to_ros.py"]