#!/bin/bash
# Build the Stonefish AUV Docker image
#
# Usage:
#   ./scripts/docker-build.sh           # Build with cache
#   ./scripts/docker-build.sh --no-cache  # Build without cache (clean rebuild)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "=========================================="
echo "Building Stonefish AUV Docker Image"
echo "=========================================="
echo ""

# Check for --no-cache flag
if [[ "$1" == "--no-cache" ]]; then
    echo "Building without cache (clean rebuild)..."
    docker compose build --no-cache
else
    echo "Building with cache..."
    docker compose build
fi

echo ""
echo "=========================================="
echo "Build complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Run: xhost +local:docker"
echo "  2. Run: ./scripts/docker-run.sh"
echo ""
