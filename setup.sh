source /opt/ros/noetic/setup.sh

toplevel_dir=`git rev-parse --show-toplevel`

echo "Root is - ${toplevel_dir}"

export PROJECT_ROOT=${toplevel_dir}

# Setup sscripts
export PROJECT_SCRIPTS_ROOT=${PROJECT_ROOT}/scripts
source ${PROJECT_SCRIPTS_ROOT}/_utils/_base.sh

# Dirs variabless
export PATH=${PROJECT_SCRIPTS_ROOT}:$PATH
export DSDL_OUTPUT=${PROJECT_ROOT}/common
export EXTERNAL_DIR=${PROJECT_ROOT}/external
export ROS_DIR=${PROJECT_ROOT}/ros

# Python config
export PYTHONPATH=${PROJECT_ROOT}/tools:${DSDL_OUTPUT}:${PROJECT_SCRIPTS_ROOT}:$PYTHONPATH

# Yakut defaults
export YAKUT_FORMAT=json
export YAKUT_COMPILE_OUTPUT=${DSDL_OUTPUT}
export YAKUT_PATH=${YAKUT_COMPILE_OUTPUT}
# and some more in .bashrc

pushd . > /dev/null
cd $ROVER_PROJECT_ROOT
# Any commands relative to top dir
popd > /dev/null

# Usefull functions

function clear_ros() {
    rm -rf "${EXTERNAL_DIR}/drivers/build" "${EXTERNAL_DIR}/drivers/devel" "${EXTERNAL_DIR}/drivers/install"
    rm -rf "${ROS_DIR}/build" "${ROS_DIR}/devel" "${ROS_DIR}/install"
}

function link_common() {
    cwd=`pwd`
    ln -s $(realpath --relative-to=. "$DSDL_OUTPUT") ${1:-$cwd}/include
}

function link_libcyphal() {
    cwd=`pwd`
    mkdir -p ${1:-$cwd}/libs
    ln -s $(realpath --relative-to=./libs "$EXTERNAL_DIR"/libcyphal) ${1:-$cwd}/libs/libcyphal
}
