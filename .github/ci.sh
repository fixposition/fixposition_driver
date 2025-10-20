#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace

SCRIPTDIR=$(dirname $(readlink -f $0))

# ----------------------------------------------------------------------------------------------------------------------

# Variables set in our Dockerfiles. Try to make it work in other environments.
if [ -z "${FPSDK_IMAGE:-}" ]; then
    export FPSDK_IMAGE=other
    if [ -z "${ROS_DISTRO:-}" ]; then
        if [ -d /opt/ros/noetic ]; then
            export ROS_DISTRO=noetic
        elif [ -d /opt/ros/humble ]; then
            export ROS_DISTRO=humble
        elif [ -d /opt/ros/jazzy ]; then
            export ROS_DISTRO=jazzy
        else
            export ROS_DISTRO=
        fi
    fi

    echo "Not using a Fixposition SDK Docker image (ROS_DISTRO=${ROS_DISTRO})"

else
    echo "Using Fixposition SDK Docker image (FPSDK_IMAGE=${FPSDK_IMAGE}, ROS_DISTRO=${ROS_DISTRO})"
fi

# Sources will be here
if [ -n "${GITHUB_WORKSPACE:-}" ]; then
    FP_SRC_DIR=${GITHUB_WORKSPACE}
# For running it locally via docker.sh...
else
    FP_SRC_DIR=${SCRIPTDIR}/..
fi
echo "FP_SRC_DIR=${FP_SRC_DIR}"

# Optional single command-line argument to ci.sh to select only one (or some) step(s)
STEP_FILT=${1:-}

# ----------------------------------------------------------------------------------------------------------------------

ERROR_COUNT=0
ERROR_NAMES=
NSTEPS=0
declare -A TITLES
function do_step
{
    local func=$1

    # Skip?
    if [[ ! ${func} =~ ${STEP_FILT} ]]; then
        echo "Skip ${TITLES[$func]}"
        return 0
    fi

    local res=0
    echo "::group::${TITLES[$func]} ($func)"
    echo "----- $func: ${TITLES[$func]} -----"
    ((NSTEPS=${NSTEPS} + 1))

    if ! ${func}; then
        res=1
        ((ERROR_COUNT=${ERROR_COUNT} + 1))
        ERROR_NAMES="${ERROR_NAMES} ${func}"
    fi

    echo "::endgroup::"
    if [ ${res} -ne 0 ]; then
        echo "::warning title=${FPSDK_IMAGE} ${func} failed::${TITLES[$func]} ($func)"
    fi

    return ${res}
}


########################################################################################################################

TITLES["pre_commit_check"]="Pre-commit checks"
function pre_commit_check
{
    ${FP_SRC_DIR}/precommit.sh
}

########################################################################################################################

TITLES["build_catkin_release"]="Build catkin (release, with ROS1)"
function build_catkin_release
{
    local buildname=${FPSDK_IMAGE}_build_catkin_release
    ${FP_SRC_DIR}/create_ros_ws.sh ${buildname}
    cd ${FP_SRC_DIR}/${buildname}
    catkin build || return 1
    set +u
    source devel/setup.bash
    set -u
    rospack find fixposition_driver_ros1
}

TITLES["build_catkin_debug"]="Build catkin (debug, with ROS1)"
function build_catkin_debug
{
    local buildname=${FPSDK_IMAGE}_build_catkin_debug
    ${FP_SRC_DIR}/create_ros_ws.sh -d ${buildname}
    cd ${FP_SRC_DIR}/${buildname}
    catkin build || return 1
    set +u
    source devel/setup.bash
    set -u
    rospack find fixposition_driver_ros1
}

########################################################################################################################

TITLES["build_colcon_release"]="Build colcon (release, with ROS2)"
function build_colcon_release
{
    local buildname=${FPSDK_IMAGE}_build_colcon_release
    ${FP_SRC_DIR}/create_ros_ws.sh ${buildname} || return 1
    cd ${FP_SRC_DIR}/${buildname}
    colcon build || return 1
    set +u
    source install/setup.bash
    set -u
    ros2 pkg executables fixposition_driver_ros2
}

TITLES["build_colcon_debug"]="Build colcon (debug, with ROS2)"
function build_colcon_debug
{
    local buildname=${FPSDK_IMAGE}_build_colcon_debug
    ${FP_SRC_DIR}/create_ros_ws.sh -d ${buildname} || return 1
    cd ${FP_SRC_DIR}/${buildname}
    colcon build || return 1
    set +u
    source install/setup.bash
    set -u
    ros2 pkg executables fixposition_driver_ros2
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_colcon_release_clang"]="Build colcon (release, with ROS2, clang instead of GCC)"
function build_colcon_release_clang
{
    local buildname=${FPSDK_IMAGE}_build_colcon_release_clang
    ${FP_SRC_DIR}/create_ros_ws.sh ${buildname} || return 1
    cd ${FP_SRC_DIR}/${buildname}
    export CC=clang CXX=clang++
    colcon build || return 1
    set +u
    source install/setup.bash
    set -u
    ros2 pkg executables fixposition_driver_ros2
    unset CC CXX
}

TITLES["build_colcon_debug_clang"]="Build colcon (debug, with ROS2, clang instead of GCC)"
function build_colcon_debug_clang
{
    local buildname=${FPSDK_IMAGE}_build_colcon_debug_clang
    ${FP_SRC_DIR}/create_ros_ws.sh -d ${buildname} || return 1
    cd ${FP_SRC_DIR}/${buildname}
    export CC=clang CXX=clang++
    colcon build || return 1
    set +u
    source install/setup.bash
    set -u
    ros2 pkg executables fixposition_driver_ros2
    unset CC CXX
}

########################################################################################################################

# Always
do_step pre_commit_check               || true # continue

# ROS 1
if [ "${ROS_DISTRO}" = "noetic" ]; then
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    do_step build_catkin_release          || true # continue
    do_step build_catkin_debug            || true # continue

# ROS 2
elif [ "${ROS_DISTRO}" = "humble" -o "${ROS_DISTRO}" = "jazzy" ]; then
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    do_step build_colcon_release          || true # continue
    do_step build_colcon_debug            || true # continue

    do_step build_colcon_release_clang    || true # continue
    do_step build_colcon_debug_clang      || true # continue
fi

########################################################################################################################

# Are we happy?
if [ ${ERROR_COUNT} -eq 0 ]; then
    echo "::notice TITLES=CI success::Successfully completed ${NSTEPS} steps"
    exit 0
else
    echo "::error TITLES=CI failure::Failed ${ERROR_COUNT} of ${NSTEPS} steps: ${ERROR_NAMES}"
    exit 1
fi

########################################################################################################################
