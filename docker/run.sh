#!/bin/bash
# Usage:
#   ./docker/run.sh            # run topics TUI (default)
#   ./docker/run.sh topics     # run topics TUI with dummy publishers
#   ./docker/run.sh params     # run params TUI with dummy publishers
#   ./docker/run.sh shell      # drop into bash inside the container
#   ./docker/run.sh build      # only (re)build the image, don't run

set -e

IMAGE="ros2_tui:dev"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

build_image() {
    echo "==> Building Docker image '$IMAGE'..."
    docker build \
        -t "$IMAGE" \
        -f "$REPO_ROOT/docker/Dockerfile" \
        "$REPO_ROOT"
}

run_tui() {
    local mode="${1:-topics}"
    echo "==> Starting ros2_tui ($mode) inside container..."
    docker run --rm -it \
        -e TERM="${TERM:-xterm-256color}" \
        --name "ros2_tui_dev" \
        "$IMAGE" \
        "$mode"
}

# Parse command
CMD="${1:-topics}"

case "$CMD" in
    build)
        build_image
        ;;
    shell)
        build_image
        echo "==> Dropping into shell (ROS2 + publishers are available)..."
        docker run --rm -it \
            -e TERM="${TERM:-xterm-256color}" \
            --name "ros2_tui_dev" \
            --entrypoint /bin/bash \
            "$IMAGE" \
            -c "source /opt/ros/humble/setup.bash && \
                source /ros2_ws/install/setup.bash && \
                echo 'ROS2 Humble ready. Binaries: topics, params (in /ros2_tui/target/release/)' && \
                exec bash"
    ;;
    topics|params)
        build_image
        run_tui "$CMD"
        ;;
    *)
        echo "Usage: $0 [topics|params|shell|build]"
        exit 1
        ;;
esac
