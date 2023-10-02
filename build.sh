#!/usr/bin/env bash
set -e

function build_source_install() {
    pushd .
    catkin_make
    source devel/setup.bash
    cd build
    make install
    popd
}

source setup.sh

make run-script SCRIPT=dev/compile.py

clear_ros

cd "${PROJECT_ROOT}/external/drivers"
# Dirty fix
mkdir -p src/bosch_imu/src
build_source_install

cd "${PROJECT_ROOT}/ros"
build_source_install
