# Stonefish AUV Simulation Environment
# ROS2 Humble + Stonefish simulator with OpenGL rendering support
#
# Build: docker build -t stonefish-auv:latest .
# Run: docker-compose up

ARG ROS_DISTRO=humble

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
    software-properties-common \
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

# Install GCC 13 for C++20 <format> header support (required by Stonefish)
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y gcc-13 g++-13 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 \
    && rm -rf /var/lib/apt/lists/*

# Clone and build Stonefish library
WORKDIR /opt
RUN git clone --depth 1 https://github.com/patrykcieslak/stonefish.git \
    && cd stonefish \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_CXX_STANDARD=20 \
        -DCMAKE_CXX_COMPILER=/usr/bin/g++-13 \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-13 \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# ============================================================================
# UNDERLAY WORKSPACE: stonefish_ros2 (installed to /opt, not affected by mounts)
# ============================================================================
ENV STONEFISH_WS=/opt/stonefish_ws
WORKDIR ${STONEFISH_WS}
RUN mkdir -p src

# Clone stonefish_ros2 package and apply ROS2 Humble compatibility patches
RUN cd src \
    && git clone --depth 1 https://github.com/patrykcieslak/stonefish_ros2.git \
    # Patch for ROS2 Humble: convert builtin_interfaces::msg::Time to rclcpp::Time for arithmetic
    && find stonefish_ros2 -name "*.cpp" -exec sed -i \
        's/msg\.header\.stamp + /rclcpp::Time(msg.header.stamp) + /g' {} \; \
    && find stonefish_ros2 -name "*.cpp" -exec sed -i \
        's/\.header\.stamp + /rclcpp::Time(.header.stamp) + /g' {} \;

# Build stonefish_ros2 underlay workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && CC=/usr/bin/gcc-13 CXX=/usr/bin/g++-13 colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_STANDARD=20 \
            -DCMAKE_C_COMPILER=/usr/bin/gcc-13 \
            -DCMAKE_CXX_COMPILER=/usr/bin/g++-13

# ============================================================================
# OVERLAY WORKSPACE: user packages (mounted from host at runtime)
# ============================================================================
ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}
RUN mkdir -p src

# Create entrypoint script that sources both underlay and overlay workspaces
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
source '${STONEFISH_WS}'/install/setup.bash\n\
if [ -f '${ROS_WS}'/install/setup.bash ]; then\n\
    source '${ROS_WS}'/install/setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

# Add ROS2 sourcing to .bashrc for interactive shells (docker exec)
RUN echo '\n\
# Source ROS2 and workspaces automatically\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
source '${STONEFISH_WS}'/install/setup.bash\n\
if [ -f '${ROS_WS}'/install/setup.bash ]; then\n\
    source '${ROS_WS}'/install/setup.bash\n\
fi\n\
' >> /root/.bashrc

# Set working directory to user workspace
WORKDIR ${ROS_WS}

# Entrypoint sources ROS2 and both workspaces
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

