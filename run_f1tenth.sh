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

echo "Starting f1tenth_gym_ros container..."
cd f1tenth_gym_ros
docker compose up -d $BUILD_FLAG

cd .. 