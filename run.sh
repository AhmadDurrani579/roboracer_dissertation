#!/bin/bash

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

echo "Building f1tenth_gym_ros image..."
cd f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .

echo "Starting f1tenth_gym_ros container..."
docker compose up -d

cd ..

echo "Building roboracer image..."
cd roboracer_project
docker build -t roboracer -f docker/Dockerfile .

echo "Starting roboracer container..."
# Select compose file based on GPU availability
if command -v nvidia-smi > /dev/null 2>&1; then
    COMPOSE_FILE="docker/docker-roboracer-compose-nvidia.yml"
else
    COMPOSE_FILE="docker/docker-roboracer-compose.yml"
fi
# Run roboracer container
docker compose -f $COMPOSE_FILE run --rm roboracer_dissertation
cd ..
