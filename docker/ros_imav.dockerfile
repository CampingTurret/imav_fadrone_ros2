# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Install build tools and colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake python3-pip python3-colcon-common-extensions \
    python3-setuptools python3-vcstool curl gnupg2 lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN pip3 install numpy
RUN pip3 install RPi.GPIO

# Setup workspace
WORKDIR /home/user/imav_ws
RUN mkdir -p src

# Clone px4_msg
RUN git clone https://github.com/elijahanghw/px4_msgs_minimal.git /home/user/imav_ws/src/px4_msg

# Copy local workspace
COPY ./src /home/user/imav_ws/src

# Build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && \
            cd /home/user/imav_ws && \
            colcon build"

# Source workspace when container starts
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /home/user/imav_ws/install/setup.bash" >> ~/.bashrc

# Add entrypoint script
ADD ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]