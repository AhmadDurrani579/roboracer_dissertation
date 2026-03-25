FROM ubuntu:22.04

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
    
    
RUN apt-get update && apt-get install -y \
    ros-humble-serial-driver \
    libasio-dev \
    ros-humble-udp-msgs && rm -rf /var/lib/apt/lists/*


RUN echo "forcing rebuild for urg_node install"
RUN apt-get update && apt-get install -y ros-humble-urg-node


RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-teleop-tools \
    ros-humble-ackermann-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN apt-get update && \
    apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs  \
    && rm -rf /var/lib/apt/lists/*


# Copy ros requirements and install them
COPY docker/ros-requirements.txt /ros-requirements.txt
# hadolint ignore=SC2046
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /ros-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* && \
    rm /ros-requirements.txt

# Install Pangolin from source (following official instructions)
RUN apt-get update && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    # Install dependencies using official script with auto-confirmation
    export DEBIAN_FRONTEND=noninteractive && \
    yes | ./scripts/install_prerequisites.sh required && \
    # Configure and build
    cmake -B build && \
    cmake --build build && \
    # Install to system
    cmake --install build && \
    cd / && \
    rm -rf /Pangolin && \
    ldconfig

# # Install Sophus library for Lie groups
# RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
#     cd /tmp/Sophus && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && \
#     make -j$(nproc) && \
#     make install && \
#     cd / && \
#     rm -rf /tmp/Sophus && \
#     ldconfig


# Install ORB_SLAM3 library by cloning it directly to its final destination
RUN git clone -b v1.0-release https://github.com/UZ-SLAMLab/ORB_SLAM3.git /usr/local/ORB_SLAM3 && \
    cd /usr/local/ORB_SLAM3 && \
    sed -i -E 's/make -j[0-9]*/make -j1/g; s/make -j\b/make -j1/g' build.sh && \
    sed -i 's|cmake .. -DCMAKE_BUILD_TYPE=Release|cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14|g' build.sh && \
    chmod +x build.sh && \
    CMAKE_POLICY_VERSION_MINIMUM=3.5 ./build.sh && \
    ln -s /usr/local/ORB_SLAM3/lib/libORB_SLAM3.so /usr/local/lib/libORB_SLAM3.so && \
    ldconfig

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc

WORKDIR /home/dev/roboracer_ws
ENTRYPOINT ["/bin/bash"]



