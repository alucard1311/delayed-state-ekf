# Stonefish AUV Simulation Environment
# ROS2 Humble + Stonefish simulator with OpenGL rendering support
#
# Build: docker build -t stonefish-auv:latest .
# Run: docker-compose up

ARG ROS_DISTRO=humble
ARG STONEFISH_VERSION=1.5.0

FROM osrf/ros:${ROS_DISTRO}-desktop AS base

LABEL maintainer="Vinay"
LABEL description="ROS2 Humble + Stonefish AUV simulation environment"
LABEL version="1.0"

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Stonefish dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    # Stonefish core dependencies
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    # OpenGL dependencies
    libgl1-mesa-dev \
    libopengl-dev \
    libglew-dev \
    # X11 dependencies for display
    libx11-dev \
    libxrandr-dev \
    # Additional utilities
    wget \
    curl \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Clone and build Stonefish library
WORKDIR /opt
RUN git clone --depth 1 --branch v${STONEFISH_VERSION} \
    https://github.com/patrykcieslak/stonefish.git \
    && cd stonefish \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# Set up ROS2 workspace
ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}
RUN mkdir -p src

# Clone stonefish_ros2 package
RUN cd src \
    && git clone --depth 1 https://github.com/patrykcieslak/stonefish_ros2.git

# Build ROS2 workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
source '${ROS_WS}'/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

# Set working directory to workspace
WORKDIR ${ROS_WS}

# Entrypoint sources ROS2 and workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
