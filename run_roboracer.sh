#!/bin/bash

BUILD_FLAG=""

# Parse arguments
for arg in "$@"; do
  case $arg in
    --build)
      BUILD_FLAG="--build"
      echo "Rebuilding images as requested..."
      ;;
  esac
done

echo "Setting up X11 for GUI applications..."
# Allow X11 forwarding
xhost +local:docker

# Create X11 auth file
XAUTH_FILE=/tmp/.docker.xauth
if [ ! -f $XAUTH_FILE ]; then
    touch $XAUTH_FILE
fi
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH_FILE nmerge -
chmod 644 $XAUTH_FILE

echo "Starting roboracer container..."
cd roboracer_project

# Select compose file based on GPU availability
if command -v nvidia-smi > /dev/null 2>&1; then
    COMPOSE_FILE="docker/docker-roboracer-compose-nvidia.yml"
else
    COMPOSE_FILE="docker/docker-roboracer-compose.yml"
fi

# Stop existing container if running
docker compose -f $COMPOSE_FILE down --remove-orphans 2>/dev/null

# Start fresh container in detached mode
docker compose -f $COMPOSE_FILE up -d $BUILD_FLAG --remove-orphans

echo "Container started. Connecting to shell..."
echo "To exit, type 'exit'. To reconnect later, run: docker compose -f $COMPOSE_FILE exec roboracer_dissertation bash"

# Connect to the running container
docker compose -f $COMPOSE_FILE exec roboracer_dissertation bash

# Always remove the container after exit
echo "Removing container..."
docker compose -f $COMPOSE_FILE down --remove-orphans

cd .. 