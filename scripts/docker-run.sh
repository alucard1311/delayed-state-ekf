#!/bin/bash
# Run the Stonefish AUV Docker container
#
# This script ensures X11 forwarding is configured before starting the container.
#
# Usage:
#   ./scripts/docker-run.sh        # Run in foreground (interactive)
#   ./scripts/docker-run.sh -d     # Run in background (detached)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "=========================================="
echo "Starting Stonefish AUV Container"
echo "=========================================="
echo ""

# Configure X11 forwarding for local Docker containers
echo "Configuring X11 access for Docker..."
xhost +local:docker 2>/dev/null || {
    echo "Warning: Could not configure xhost. Display may not work."
    echo "Try running: xhost +local:docker"
}
echo ""

# Check for -d flag (detached mode)
if [[ "$1" == "-d" ]]; then
    echo "Starting container in background..."
    docker compose up -d
    echo ""
    echo "Container started in background."
    echo ""
    echo "Useful commands:"
    echo "  docker compose logs -f    # View logs"
    echo "  docker compose exec stonefish bash  # Enter container"
    echo "  docker compose down       # Stop container"
else
    echo "Starting container in foreground..."
    echo "(Press Ctrl+C to stop)"
    echo ""
    docker compose up
fi

echo ""
