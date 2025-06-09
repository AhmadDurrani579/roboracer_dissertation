FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Set up UTF-8 locale
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Enable universe repository and install curl
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get install -y curl gnupg2

# Set up ROS2 repository
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Set up Gazebo repository
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

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

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc


