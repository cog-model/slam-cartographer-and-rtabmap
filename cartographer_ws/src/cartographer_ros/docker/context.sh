#!/bin/bash

set -e

docker_dir=$(dirname $0)

to_archive () {
    tar --remove-files -czf $1.tar.gz $1 
}

if [[ -d "$docker_dir/context" ]]
then
    echo "$(realpath ${docker_dir}/context) already exists. Delete it if you want to update docker context."
else
    mkdir $docker_dir/context
    cd $docker_dir/context
    echo "Download docker context to ${PWD}"

    echo "Cloning"
    # git clone https://gitlab.com/sdbcs-nio3/itl_mipt/slam/slam_validation.git
    # to_archive slam_validation
    git clone https://github.com/abseil/abseil-cpp.git
    to_archive abseil-cpp
    git clone https://ceres-solver.googlesource.com/ceres-solver.git
    to_archive ceres-solver
    git clone https://github.com/google/protobuf.git
    to_archive protobuf
    git clone https://github.com/andrey1908/ros_utils.git
    to_archive ros_utils
fi

