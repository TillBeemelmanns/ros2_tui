FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

# System deps + ROS2 message packages
RUN apt-get update && apt-get install -y \
    curl \
    python3-colcon-common-extensions \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-msgs \
    ros-humble-visualization-msgs \
    ros-humble-cv-bridge \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /ros2_tui

# Copy source into image
COPY . .

# Build the test publisher ROS2 package
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    mkdir -p /ros2_ws/src && \
    cp -r /ros2_tui/tests/test_publisher /ros2_ws/src/ros2_tui_test && \
    cd /ros2_ws && \
    colcon build --packages-select ros2_tui_test"

# Build the Rust binaries in release mode
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cargo build --release"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["topics"]
