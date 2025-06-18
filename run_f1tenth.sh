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

echo "Starting f1tenth_gym_ros container..."
cd f1tenth_gym_ros

# Stop existing container if running
docker compose down --remove-orphans 2>/dev/null

# Start fresh container in detached mode
docker compose up -d $BUILD_FLAG --remove-orphans

# Connect to the running container
docker compose exec sim bash

# Always remove the container after exit
echo "Removing container..."
docker compose down --remove-orphans

cd .. 