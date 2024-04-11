FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive

ARG NUM_THREADS=1

# User setup
ARG USER=docker_cartographer
ARG UID=1002
ARG GID=1003
ARG PW=user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -s /bin/bash ${USER} && \
    usermod -a -G sudo ${USER}

COPY context/ /context
WORKDIR /context

# Install dependencies
RUN apt-get update && apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    stow \
    tmux \
    tar \
    vim \
    python3-pip \
    python3-sphinx \
    libgmock-dev \
    ros-noetic-ros-numpy && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip3 install \
    numpy==1.22.3 \
    pandas==1.4.2 \
    matplotlib==3.1.2 \
    tqdm \
    colorama \
    openpyxl \
    seaborn \
    urdf-parser-py \
    transforms3d
    
# Make symlink from python3 to python
RUN PYTHON3_PATH=`which python3` && \
    ln -s $PYTHON3_PATH `dirname $PYTHON3_PATH`/python

# Build and install abseil
ARG ABSEIL_COMMIT=6abc1958562c49d797ea23270a355caf5dc39f94
RUN set -x && \
    tar -zxf abseil-cpp.tar.gz && \
    cd abseil-cpp && \
    git config --global --add safe.directory /context/abseil-cpp && \
    git checkout ${ABSEIL_COMMIT} && \
    mkdir build && \
    cd build && \
    cmake -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
        .. && \
    ninja -j${NUM_THREADS} && \
    ninja install && \
    cd /usr/local/stow && \
    stow absl && \
    cd /context && \
    rm -rf abseil-cpp
    
# Build and install ceres
ARG CERES_VERSION=1.13.0
RUN set -x && \
    tar -zxf ceres-solver.tar.gz && \
    cd ceres-solver && \
    git config --global --add safe.directory /context/ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja -DCXX11=ON && \
    ninja -j${NUM_THREADS} && \
    CTEST_OUTPUT_ON_FAILURE=1 ninja test && \
    ninja install && \
    cd /context && \
    rm -rf ceres-solver
    
# Build and install proto3
ARG PROTOBUF_VERSION=v3.4.1
RUN set -x && \
    tar -zxf protobuf.tar.gz && \
    cd protobuf && \
    git config --global --add safe.directory /context/protobuf && \
    git checkout tags/${PROTOBUF_VERSION} && \
    mkdir build && \
    cd build && \
    cmake -G Ninja \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -Dprotobuf_BUILD_TESTS=OFF \
        ../cmake && \
    ninja -j${NUM_THREADS} && \
    ninja install && \
    cd /context && \
    rm -rf protobuf
    
# Install ros_utils
ARG ROS_UTILS_COMMIT=66972c65a6819e3f03ea7b230bdb5f8342967961
RUN set -x && \
    tar -zxf ros_utils.tar.gz && \
    cd ros_utils && \
    git config --global --add safe.directory /context/ros_utils && \
    git checkout ${ROS_UTILS_COMMIT} && \
    cd .. && \
    mkdir -p /home/${USER}/ros_utils_ws/src && \
    mv ros_utils /home/${USER}/ros_utils_ws/src && \
    cd /home/${USER}/ros_utils_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make \
        -j${NUM_THREADS} \
        -DCMAKE_BUILD_TYPE=Release"

# Install slam_validation
# ARG SLAM_VALIDATION_COMMIT=b940270d909d67a1023760d4363dee2e0fd6fccf
# RUN set -x && \
#     tar -zxf slam_validation.tar.gz && \
#     cd slam_validation && \
#     git config --global --add safe.directory /context/slam_validation && \
#     git checkout ${SLAM_VALIDATION_COMMIT} && \
#     cd .. && \
#     mv slam_validation /home/${USER}

RUN mkdir -p /home/${USER}/catkin_ws/src && \
    chown -R ${UID}:${GID} /home/${USER}

WORKDIR /home/${USER}
USER ${UID}:${GID}
