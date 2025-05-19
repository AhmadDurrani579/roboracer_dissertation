FROM osrf/ros:humble-simulation

ENV DEBIAN_FRONTEND=noninteractive

COPY docker/apt-requirements.txt /apt-requirements.txt
# hadolint ignore=SC2046
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* /apt-requirements.txt

COPY docker/python-requirements.txt /python-requirements.txt
RUN pip install --no-cache-dir -r /python-requirements.txt && \
    rm /python-requirements.txt

# Copy ros requirements and install them
COPY docker/ros-requirements.txt /ros-requirements.txt
# hadolint ignore=SC2046
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /ros-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* && \
    rm /ros-requirements.txt

ARG USER=dev
ARG GROUP=roboracer

RUN addgroup $GROUP && \
    adduser --ingroup $GROUP --disabled-password --gecos "" $USER && \
    usermod -aG sudo $USER && \
    echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER && \
    mkdir -p /home/$USER/roboracer_ws && \
    chown -R $USER:$GROUP /home/$USER

USER $USER
ARG WS_DIR=/home/$USER/workspace
USER $USER


