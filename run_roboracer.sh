#!/bin/bash

BUILD_FLAG=""
# Check for --build argument
if [ "$1" == "--build" ]; then
  BUILD_FLAG="--build"
  echo "Rebuilding images as requested..."
fi

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
# Run roboracer container
docker compose -f $COMPOSE_FILE run --rm $BUILD_FLAG roboracer_dissertation
cd .. 